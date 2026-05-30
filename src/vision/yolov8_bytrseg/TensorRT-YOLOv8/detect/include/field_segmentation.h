#ifndef FIELD_SEGMENTATION_H
#define FIELD_SEGMENTATION_H

#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <vector>
#include <set>

/**
 * Field segmentation for RoboCup humanoid soccer.
 *
 * Pipeline:
 *   1. Gaussian Mixture Model (GMM) – detects varying shades of green
 *   2. Flood Fill (connected components) – labels separate green regions
 *   3. Region Growing – keeps only regions connected to the main field
 *   4. Convex Hull – smooths the final field boundary polygon
 *
 * Usage:
 *   FieldSegmentation seg;          // create once
 *   seg.process(bgr_frame);         // call every frame (internally skips if cached)
 *   bool ok = seg.isBBoxInsideField(x1, y1, x2, y2);
 */
class FieldSegmentation {
public:
    /**
     * @param num_gmm_components  K for the EM model (default 3)
     * @param process_scale       down-scale factor applied to the input (default 2)
     * @param retrain_interval    retrain GMM every N segmentation runs (default 90)
     * @param mahal_threshold     squared Mahalanobis distance threshold (default 12.0)
     */
    FieldSegmentation(int num_gmm_components  = 3,
                      int process_scale       = 2,
                      //int retrain_interval    = 90,
                      double mahal_threshold  = 12.0);
    ~FieldSegmentation() = default;

    /** Run the full segmentation pipeline.  Returns true when a field is found. */
    bool process(const cv::Mat& bgr_frame);

    /** Point-in-field test (coordinates in the *input* frame space). */
    bool isInsideField(int x, int y) const;

    /** Convenience: test the centre of an (x1,y1)–(x2,y2) bounding box. */
    bool isBBoxInsideField(float x1, float y1, float x2, float y2) const;

    /** Draw a translucent overlay + boundary on the frame. */
    void drawOverlay(cv::Mat& frame, float alpha = 0.15f) const;

    bool isFieldDetected() const { return field_detected_; }
    const std::vector<cv::Point>& getHull() const { return field_hull_scaled_; }
    int getFieldAreaPercent() const { return field_area_percent_; }

private:
    /* ---- GMM ---- */
    cv::Ptr<cv::ml::EM> gmm_;
    int   num_components_;
    bool  gmm_trained_;
    //int   retrain_counter_;
    //int   retrain_interval_;
    double mahal_threshold_;

    // Extracted parameters (used for fast vectorised classification)
    std::vector<cv::Mat>   gmm_means_;       // each 1×3 (H,S,V) CV_64F
    std::vector<cv::Mat>   gmm_cov_invs_;    // each 3×3 inverse covariance
    std::vector<double>    gmm_weights_;

    /* ---- resolution ---- */
    int process_scale_;
    int proc_w_, proc_h_;
    int orig_w_, orig_h_;

    /* ---- results ---- */
    cv::Mat                   field_mask_proc_;      // processing resolution
    std::vector<cv::Point>    field_hull_;            // processing resolution
    std::vector<cv::Point>    field_hull_scaled_;     // input-frame resolution
    bool  field_detected_;
    int   field_area_percent_;

    /* ---- frame skip ---- */
    int frame_counter_;
    int seg_every_n_;          // run full pipeline every N frames

    /* ---- pipeline steps ---- */
    cv::Mat initialGreenDetect(const cv::Mat& hsv);
    void    trainGMM(const cv::Mat& hsv, const cv::Mat& green_mask);
    cv::Mat classifyGMM(const cv::Mat& hsv);
    cv::Mat labelComponents(const cv::Mat& binary, int& num_labels);
    cv::Mat growFieldRegion(const cv::Mat& labels, int num_labels,
                            const cv::Mat& green_mask);
    std::vector<cv::Point> computeConvexHull(const cv::Mat& field_mask);
};

#endif // FIELD_SEGMENTATION_H
