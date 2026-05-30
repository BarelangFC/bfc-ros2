#include "field_segmentation.h"
#include <algorithm>
#include <queue>
#include <random>
#include <iostream>
#include <opencv2/ml.hpp>

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
FieldSegmentation::FieldSegmentation(int num_gmm_components,
                                     int process_scale,
                                     double mahal_threshold)
    : num_components_(num_gmm_components)
    , process_scale_(process_scale)
    , mahal_threshold_(mahal_threshold)
    , gmm_trained_(false)
    , field_detected_(false)
    , field_area_percent_(0)
    , frame_counter_(0)
    , seg_every_n_(3)          // run segmentation every 3 frames
    , proc_w_(0), proc_h_(0)
    , orig_w_(0), orig_h_(0)
{
    gmm_ = cv::ml::EM::create();
    gmm_->setClustersNumber(num_components_);
    gmm_->setCovarianceMatrixType(cv::ml::EM::COV_MAT_GENERIC);
    gmm_->setTermCriteria(
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                         50, 0.01));
}

// ---------------------------------------------------------------------------
// Main pipeline
// ---------------------------------------------------------------------------
bool FieldSegmentation::process(const cv::Mat& bgr_frame)
{
    orig_w_ = bgr_frame.cols;
    orig_h_ = bgr_frame.rows;
    proc_w_ = bgr_frame.cols / process_scale_;
    proc_h_ = bgr_frame.rows / process_scale_;

    frame_counter_++;

    // Re-use cached result on non-segmentation frames
    if (field_detected_ && (frame_counter_ % seg_every_n_ != 0)) {
        return field_detected_;
    }

    // --- Down-sample --------------------------------------------------------
    cv::Mat small;
    cv::resize(bgr_frame, small, cv::Size(proc_w_, proc_h_),
               0, 0, cv::INTER_LINEAR);

    cv::Mat hsv;
    cv::cvtColor(small, hsv, cv::COLOR_BGR2HSV);

    // --- Step 1 : Green detection (GMM) -------------------------------------
    cv::Mat green_mask;

    if (!gmm_trained_) {
        // Bootstrap with simple HSV range, then train GMM
        cv::Mat initial = initialGreenDetect(hsv);
        int count = cv::countNonZero(initial);
        if (count > 100) {
            trainGMM(hsv, initial);
        }
        green_mask = gmm_trained_ ? classifyGMM(hsv) : initial;
    } else {
        green_mask = classifyGMM(hsv);
    }

    // Morphological clean-up
    cv::Mat kern = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(green_mask, green_mask, cv::MORPH_CLOSE, kern);
    cv::morphologyEx(green_mask, green_mask, cv::MORPH_OPEN,  kern);

    // --- Step 2 : Flood fill (connected components) -------------------------
    int num_labels = 0;
    cv::Mat labels = labelComponents(green_mask, num_labels);
    if (num_labels <= 1) {                   // only background
        field_detected_ = false;
        return false;
    }

    // --- Step 3 : Region growing --------------------------------------------
    cv::Mat field_mask = growFieldRegion(labels, num_labels, green_mask);

    // --- Step 4 : Convex hull -----------------------------------------------
    std::vector<cv::Point> hull = computeConvexHull(field_mask);
    if (hull.size() < 3) {
        field_detected_ = false;
        return false;
    }

    // Store results
    field_mask_proc_ = field_mask;
    field_hull_       = hull;

    // Scale hull back to input resolution
    field_hull_scaled_.clear();
    field_hull_scaled_.reserve(hull.size());
    for (const auto& pt : hull) {
        field_hull_scaled_.emplace_back(pt.x * process_scale_,
                                        pt.y * process_scale_);
    }

    // Field-area percentage
    int field_px = cv::countNonZero(field_mask);
    int total_px = proc_w_ * proc_h_;
    field_area_percent_ = (field_px * 100) / std::max(total_px, 1);

    field_detected_ = (field_area_percent_ > 5);   // at least 5 %
    return field_detected_;
}

// ---------------------------------------------------------------------------
// Step 1a – broad HSV thresholding (bootstrap for GMM training)
// ---------------------------------------------------------------------------
cv::Mat FieldSegmentation::initialGreenDetect(const cv::Mat& hsv)
{
    // Define base HSV colors from your clicks here
    std::vector<cv::Vec3b> target_hsvs = {
        {33, 251, 133},
        {37, 253, 131},
        {37, 253, 126},
        {37, 232, 147},
        {39, 193, 143},
        {35, 221, 76},
        {34, 193, 205},
        {33, 204, 185},
        {37, 223, 167},
        {37, 250, 105},
        {32, 255, 85},
        {36, 255, 105},
        {32, 226, 139},
        {40, 198, 135},
        {42, 198, 102}
    };

    cv::Mat combined = cv::Mat::zeros(hsv.size(), CV_8UC1);

    for (const auto& color : target_hsvs) {
        int h = color[0];
        int s = color[1];
        int v = color[2];

        // Apply tolerances (H +- 10, S +- 40, V +- 40)
        cv::Scalar lower(std::max(0, h - 10), std::max(0, s - 40), std::max(0, v - 40));
        cv::Scalar upper(std::min(179, h + 10), std::min(255, s + 40), std::min(255, v + 40));

        cv::Mat m;
        cv::inRange(hsv, lower, upper, m);
        cv::bitwise_or(combined, m, combined);
    }

    // Fallback if target_hsvs is totally empty
    if (target_hsvs.empty()) {
        cv::inRange(hsv, cv::Scalar(35, 40, 30), cv::Scalar(85, 255, 255), combined);
    }

    return combined;
}

// ---------------------------------------------------------------------------
// Step 1b – train the Gaussian Mixture Model on sampled green pixels
// ---------------------------------------------------------------------------
void FieldSegmentation::trainGMM(const cv::Mat& hsv,
                                 const cv::Mat& green_mask)
{
    // Collect green-pixel HSV values
    std::vector<cv::Vec3b> pixels;
    pixels.reserve(green_mask.rows * green_mask.cols / 4);

    for (int y = 0; y < hsv.rows; ++y) {
        const uchar*    mrow = green_mask.ptr<uchar>(y);
        const cv::Vec3b* hrow = hsv.ptr<cv::Vec3b>(y);
        for (int x = 0; x < hsv.cols; ++x) {
            if (mrow[x] > 0) pixels.push_back(hrow[x]);
        }
    }

    if ((int)pixels.size() < 50) return;

    // Sub-sample for speed (increased for better 1-time training)
    const int kMaxSamples = 10000;
    if ((int)pixels.size() > kMaxSamples) {
        std::mt19937 rng(42);
        std::shuffle(pixels.begin(), pixels.end(), rng);
        pixels.resize(kMaxSamples);
    }

    // Build Nx3 sample matrix (CV_64F)
    cv::Mat samples((int)pixels.size(), 3, CV_64F);
    for (int i = 0; i < (int)pixels.size(); ++i) {
        samples.at<double>(i, 0) = pixels[i][0];
        samples.at<double>(i, 1) = pixels[i][1];
        samples.at<double>(i, 2) = pixels[i][2];
    }

    try {
        cv::Mat logL, lbls, probs;
        gmm_->trainEM(samples, logL, lbls, probs);

        // Cache means / inverse covariances for fast classify
        cv::Mat means = gmm_->getMeans();     // K×3
        std::vector<cv::Mat> covs;
        gmm_->getCovs(covs);
        cv::Mat weights = gmm_->getWeights(); // 1×K

        gmm_means_.clear();
        gmm_cov_invs_.clear();
        gmm_weights_.clear();

        for (int k = 0; k < num_components_; ++k) {
            gmm_means_.push_back(means.row(k).clone());
            gmm_cov_invs_.push_back(covs[k].inv());
            gmm_weights_.push_back(weights.at<double>(0, k));
        }

        gmm_trained_ = true;
        std::cout << "[FieldSeg] GMM trained – " << pixels.size()
                  << " samples, " << num_components_ << " components"
                  << std::endl;

    } catch (const cv::Exception& e) {
        std::cerr << "[FieldSeg] GMM training failed: " << e.what() << "\n";
        gmm_trained_ = false;
    }
}

// ---------------------------------------------------------------------------
// Step 1c – classify every pixel with the trained GMM (vectorised)
//
// For each component k we compute the squared Mahalanobis distance:
//   d²_k(x) = (x − μ_k)ᵀ  Σ_k⁻¹  (x − μ_k)
// A pixel is "green" if min_k d²_k < threshold.
// ---------------------------------------------------------------------------
cv::Mat FieldSegmentation::classifyGMM(const cv::Mat& hsv)
{
    if (!gmm_trained_ || gmm_means_.empty())
        return initialGreenDetect(hsv);          // fallback

    const int rows = hsv.rows;
    const int cols = hsv.cols;
    const int N    = rows * cols;

    // Reshape image → N×3  (CV_64F)
    cv::Mat flat;
    hsv.reshape(1, N).convertTo(flat, CV_64F);   // N×3

    cv::Mat green_flat = cv::Mat::zeros(N, 1, CV_8UC1);

    for (int k = 0; k < num_components_; ++k) {
        // diff = pixels − μ_k   (broadcast via cv::repeat)
        cv::Mat mean_rep;
        cv::repeat(gmm_means_[k], N, 1, mean_rep);   // N×3
        cv::Mat diff = flat - mean_rep;                // N×3

        // temp = diff × Σ⁻¹_k              (N×3  ×  3×3  =  N×3)
        cv::Mat temp = diff * gmm_cov_invs_[k];

        // d²[i] = Σ_j temp[i][j] · diff[i][j]
        cv::Mat product;
        cv::multiply(temp, diff, product);             // element-wise

        cv::Mat d2;
        cv::reduce(product, d2, 1, cv::REDUCE_SUM, CV_64F); // N×1

        // Mark pixels below threshold
        for (int i = 0; i < N; ++i) {
            if (d2.at<double>(i, 0) < mahal_threshold_)
                green_flat.at<uchar>(i, 0) = 255;
        }
    }

    return green_flat.reshape(1, rows);  // back to H×W
}

// ---------------------------------------------------------------------------
// Step 2 – connected-component labelling ("flood fill")
// ---------------------------------------------------------------------------
cv::Mat FieldSegmentation::labelComponents(const cv::Mat& binary,
                                           int& num_labels)
{
    cv::Mat labels;
    num_labels = cv::connectedComponents(binary, labels, 8, CV_32S);
    return labels;
}

// ---------------------------------------------------------------------------
// Step 3 – region growing from the largest component
//
// Build an adjacency graph between labelled regions, then BFS outward from
// the biggest one.  Isolated green patches that are NOT reachable are
// discarded (not considered field).
// ---------------------------------------------------------------------------
cv::Mat FieldSegmentation::growFieldRegion(const cv::Mat& labels,
                                           int num_labels,
                                           const cv::Mat& /*green_mask*/)
{
    if (num_labels <= 1)
        return cv::Mat::zeros(labels.size(), CV_8UC1);

    // --- pixel count per label (skip 0 = background) ---
    std::vector<int> count(num_labels, 0);
    for (int y = 0; y < labels.rows; ++y) {
        const int* row = labels.ptr<int>(y);
        for (int x = 0; x < labels.cols; ++x) {
            if (row[x] > 0) count[row[x]]++;
        }
    }

    // Check if the overall largest blob is at least 3% of the screen
    int largest_n = 0;
    for (int i = 1; i < num_labels; ++i) {
        if (count[i] > largest_n) { largest_n = count[i]; }
    }

    int total = labels.rows * labels.cols;
    if (largest_n < total * 0.03)               // < 3 % → no field
        return cv::Mat::zeros(labels.size(), CV_8UC1);

    // --- Keep all green regions that are not tiny noise ---
    std::vector<bool> is_field(num_labels, false);
    for (int i = 1; i < num_labels; ++i) {
        if (count[i] > 100) {   // include all disconnected green parts > 100 px noise threshold
            is_field[i] = true;
        }
    }

    // --- build mask ---
    cv::Mat mask = cv::Mat::zeros(labels.size(), CV_8UC1);
    for (int y = 0; y < labels.rows; ++y) {
        const int* lrow = labels.ptr<int>(y);
        uchar*     mrow = mask.ptr<uchar>(y);
        for (int x = 0; x < labels.cols; ++x) {
            if (lrow[x] > 0 && is_field[lrow[x]])
                mrow[x] = 255;
        }
    }
    return mask;
}

// ---------------------------------------------------------------------------
// Step 4 – convex hull
// ---------------------------------------------------------------------------
std::vector<cv::Point> FieldSegmentation::computeConvexHull(
    const cv::Mat& field_mask)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(field_mask, contours,
                     cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return {};

    // Merge all contour points
    std::vector<cv::Point> all;
    for (const auto& c : contours)
        all.insert(all.end(), c.begin(), c.end());
    if (all.size() < 3) return {};

    std::vector<cv::Point> hull;
    cv::convexHull(all, hull);
    return hull;
}

// ---------------------------------------------------------------------------
// Point / BBox inside-field tests
// ---------------------------------------------------------------------------
bool FieldSegmentation::isInsideField(int x, int y) const
{
    if (!field_detected_ || field_hull_scaled_.empty())
        return true;               // no field → optimistic (assume in-field)

    double d = cv::pointPolygonTest(field_hull_scaled_,
                                    cv::Point2f((float)x, (float)y), false);
    return d >= 0;                 // 0 = on edge, +1 = inside
}

bool FieldSegmentation::isBBoxInsideField(float x1, float y1,
                                          float x2, float y2) const
{
    if (!field_detected_ || field_hull_scaled_.empty())
        return true;               // no field → optimistic (assume in-field)

    // Mengecek 25 titik (grid 5x5) yang tersebar rata di seluruh area Bounding Box bola.
    // Jika ada SATU titik saja yang menyentuh hijau lapangan, maka bola dianggap "IN".
    // Bola akan dinyatakan "OUT" murni jika ke-25 titiknya 100% meleset dari lapangan.
    for (int i = 0; i <= 4; ++i) {
        float px = x1 + i * (x2 - x1) / 4.0f;
        for (int j = 0; j <= 4; ++j) {
            float py = y1 + j * (y2 - y1) / 4.0f;
            double d = cv::pointPolygonTest(field_hull_scaled_,
                                            cv::Point2f(px, py), false);
            if (d >= 0) {
                return true; // Sentuh lapangan sedikit pun = IN
            }
        }
    }
    return false; // Keluar dari lapangan sepenuhnya = OUT
}

// ---------------------------------------------------------------------------
// Debug overlay
// ---------------------------------------------------------------------------
void FieldSegmentation::drawOverlay(cv::Mat& frame, float alpha) const
{
    if (!field_detected_ || field_hull_scaled_.empty()) return;

    // Semi-transparent fill
    cv::Mat overlay = frame.clone();
    std::vector<std::vector<cv::Point>> hulls = {field_hull_scaled_};
    cv::fillPoly(overlay, hulls, cv::Scalar(0, 80, 0));
    cv::addWeighted(overlay, alpha, frame, 1.0f - alpha, 0, frame);

    // Boundary
    cv::polylines(frame, hulls, true, cv::Scalar(0, 255, 0), 2);
}

