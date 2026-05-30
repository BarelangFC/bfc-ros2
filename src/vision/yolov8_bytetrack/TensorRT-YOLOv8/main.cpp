#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#include "infer.h"
#include "BYTETracker.h"

#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>
#include "od_msgs/msg/ball_detection_array.hpp"
#include "od_msgs/msg/ball_info.hpp"
#include "std_msgs/msg/header.hpp"

// Need to track the ROS node
class YoloTrackerNode : public rclcpp::Node {
public:
    YoloTrackerNode() : Node("yolo_tracker_node") {
        // Create publisher for ball detection array
        ball_publisher_ = this->create_publisher<od_msgs::msg::BallDetectionArray>("yolo/ball_detections", 10);
        
        // Parameters for tracking mode
        use_tracking_ = this->declare_parameter("use_tracking", true);
        detection_only_mode_ = this->declare_parameter("detection_only", false);
        
        RCLCPP_INFO(this->get_logger(), "YoloTrackerNode initialized with tracking: %s", 
                   use_tracking_ ? "ON" : "OFF");
    }

    void publish_ball_detections(const std::vector<Detection>& detections, 
                                const std::vector<STrack>& tracks = std::vector<STrack>()) {
        auto msg = od_msgs::msg::BallDetectionArray();
        msg.header.stamp = this->now();
        msg.header.frame_id = "camera_frame";  // You can customize this
        
        msg.detection_count = 0;
        msg.tracking_count = 0;
        
        // Process detections
        for (const auto& det : detections) {
            if (det.conf >= 0.25f && det.classId == 0) { // Assuming class 0 is ball
                od_msgs::msg::BallInfo ball_info;
                ball_info.x = static_cast<int64_t>((det.bbox[0] + det.bbox[2]) / 2);
                ball_info.y = static_cast<int64_t>((det.bbox[1] + det.bbox[3]) / 2);
                ball_info.confidence = det.conf;
                ball_info.track_id = -1; // No track ID for detection-only
                ball_info.status = "detected";
                
                msg.balls.push_back(ball_info);
                msg.detection_count++;
            }
        }
        
        // Process tracks if tracking is enabled
        if (use_tracking_) {
            for (const auto& track : tracks) {
                // Assuming ball class tracking
                if (track.is_activated && track.state == TrackState::Tracked) {
                    std::vector<float> tlwh = track.tlwh;
                    if (tlwh[2] * tlwh[3] > 20) { // Minimum area threshold
                        od_msgs::msg::BallInfo ball_info;
                        ball_info.x = static_cast<int64_t>(tlwh[0] + tlwh[2] / 2);
                        ball_info.y = static_cast<int64_t>(tlwh[1] + tlwh[3] / 2);
                        ball_info.confidence = track.score;
                        ball_info.track_id = track.track_id;
                        ball_info.status = "tracked";
                        
                        msg.balls.push_back(ball_info);
                        msg.tracking_count++;
                    }
                }
            }
        }
        
        ball_publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<od_msgs::msg::BallDetectionArray>::SharedPtr ball_publisher_;
    bool use_tracking_;
    bool detection_only_mode_;
};

int run(int mode, const std::string& sourcePath, bool use_tracking, bool show_detailed_fps) {
    // Initialize ROS if not already initialized
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    // Create ROS node
    auto ros_node = std::make_shared<YoloTrackerNode>();
    
    // Set the tracking mode parameter
    ros_node->set_parameter(rclcpp::Parameter("use_tracking", use_tracking));

    cv::VideoCapture cap;
    cv::VideoWriter writer;
    bool is_webcam = (mode == 2);

    // Open input
    if (is_webcam) {
        int device_id = std::stoi(sourcePath);
        cap.open(device_id, cv::CAP_V4L2); // Use V4L2 for Linux
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
        cap.set(cv::CAP_PROP_FPS, 30);
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    	// Set camera parameters
    	/*cap.set(cv::CAP_PROP_BRIGHTNESS, 130);              // Brightness
    	cap.set(cv::CAP_PROP_CONTRAST, 129);                // Contrast
    	cap.set(cv::CAP_PROP_SATURATION, 152);             // Saturation
    	cap.set(cv::CAP_PROP_GAIN, 255);                    // Gain
    	cap.set(cv::CAP_PROP_AUTO_WB, true);              // Auto White Balance
    	//cap.set(cv::CAP_PROP_WB_TEMPERATURE, 4750);       // White Balance
    	cap.set(cv::CAP_PROP_AUTO_EXPOSURE, true);        // Auto Exposure
    	//cap.set(cv::CAP_PROP_EXPOSURE, 270);              // Exposure
    	cap.set(cv::CAP_PROP_AUTOFOCUS, true);            // Auto Focus
    	//cap.set(cv::CAP_PROP_FOCUS, 0);                   // Focus
    	//cap.set(cv::CAP_PROP_FPS, 35);
    	cap.set(cv::CAP_PROP_GAMMA, 270); 
    	cap.set(cv::CAP_PROP_SHARPNESS, 128);*/
    	
    	//widelens
    	cap.set(cv::CAP_PROP_BRIGHTNESS, 0);              // Brightness
    	cap.set(cv::CAP_PROP_CONTRAST, 15);               // Contrast
    	cap.set(cv::CAP_PROP_SATURATION, 30);             // Saturation
    	cap.set(cv::CAP_PROP_GAIN, 1);                    // Gain
    	cap.set(cv::CAP_PROP_AUTO_WB, 44);              // Auto White Balance
    	// cap.set(cv::CAP_PROP_WB_TEMPERATURE, 5346);    // White Balance
    	//cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 21);      // Auto Exposure
    	//cap.set(cv::CAP_PROP_EXPOSURE, 2600);             // Exposure
    	cap.set(cv::CAP_PROP_AUTOFOCUS, 39);            // Auto Focus
    	// cap.set(cv::CAP_PROP_FOCUS, 0);                // Focus
    	cap.set(cv::CAP_PROP_FPS, 30);
    	cap.set(cv::CAP_PROP_GAMMA, 270); 
    	cap.set(cv::CAP_PROP_SHARPNESS, 20);
        
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open webcam " << device_id << std::endl;
            return -1;
        }
        std::cout << "Webcam opened successfully." << std::endl;
    } else {
        cap.open(sourcePath);
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open video " << sourcePath << std::endl;
            return -1;
        }

        int img_w = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        int img_h = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        int fps = static_cast<int>(cap.get(cv::CAP_PROP_FPS));
        writer.open("result.mp4", cv::VideoWriter::fourcc('m', 'p', 'v', '4'), fps, cv::Size(img_w, img_h));
        if (!writer.isOpened()) {
            std::cerr << "Error: Could not create output video file." << std::endl;
            return -1;
        }
    }

    // Detector
    std::string trtFile = "./detect/bestyolov8nlaptopangga.plan";
    YoloDetector detector(trtFile, 0, 0.45f, 0.01f);

    // Tracker (optional)
    BYTETracker* tracker = nullptr;
    if (use_tracking) {
        int fps = static_cast<int>(cap.get(cv::CAP_PROP_FPS));
        tracker = new BYTETracker(fps > 0 ? fps : 30, 30);
        std::cout << "ByteTrack enabled." << std::endl;
    } else {
        std::cout << "Tracking disabled. Running detection only." << std::endl;
    }

    // CUDA setup
    cv::cuda::GpuMat gpu_frame, resized_gpu;
    cv::Mat resized_cpu, frame;

    int target_width = 640;
    int target_height = 360; // 1280x720 -> 16:9 aspect ratio

    auto start_time = std::chrono::steady_clock::now();
    int frame_count = 0;
    int total_inference_ms = 0;
    int total_capture_ms = 0;
    int total_upload_ms = 0;
    int total_resize_ms = 0;
    int total_draw_ms = 0;
    int total_display_ms = 0;

    // For camera FPS calculation
    auto prev_frame_time = std::chrono::steady_clock::now();
    double camera_fps = 0.0;
    int camera_frame_count = 0;
    auto camera_start_time = std::chrono::steady_clock::now();

    while (rclcpp::ok()) { // Changed from true to rclcpp::ok()
        auto capture_start = std::chrono::steady_clock::now();
        cap >> frame;
        auto capture_end = std::chrono::steady_clock::now();
        int capture_ms = std::chrono::duration_cast<std::chrono::microseconds>(capture_end - capture_start).count() / 1000;
        total_capture_ms += capture_ms;

        if (frame.empty()) break;

        // Calculate camera FPS
        auto current_frame_time = std::chrono::steady_clock::now();
        double frame_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_frame_time - prev_frame_time).count();
        if (frame_time_ms > 0) {
            camera_fps = 1000.0 / frame_time_ms;
        }
        prev_frame_time = current_frame_time;
        camera_frame_count++;

        frame_count++;

        // Upload to GPU
        auto upload_start = std::chrono::steady_clock::now();
        gpu_frame.upload(frame);
        auto upload_end = std::chrono::steady_clock::now();
        int upload_ms = std::chrono::duration_cast<std::chrono::microseconds>(upload_end - upload_start).count() / 1000;
        total_upload_ms += upload_ms;

        // Resize using CUDA
        auto resize_start = std::chrono::steady_clock::now();
        cv::cuda::resize(gpu_frame, resized_gpu, cv::Size(target_width, target_height));
        resized_gpu.download(resized_cpu);
        auto resize_end = std::chrono::steady_clock::now();
        int resize_ms = std::chrono::duration_cast<std::chrono::microseconds>(resize_end - resize_start).count() / 1000;
        total_resize_ms += resize_ms;

        auto start_infer = std::chrono::steady_clock::now();

        // Run YOLO inference
        std::vector<Detection> detections = detector.inference(resized_cpu);

        std::vector<Object> objects;
        if (use_tracking) {
            objects.clear();
            for (const auto& det : detections) {
                if (det.conf >= 0.1f && det.classId == 0) { // Only track balls (class 0)
                    cv::Rect_<float> rect(
                        det.bbox[0],
                        det.bbox[1],
                        det.bbox[2] - det.bbox[0],
                        det.bbox[3] - det.bbox[1]
                    );
                    Object obj{rect, det.classId, det.conf};
                    objects.push_back(obj);
                }
            }
        }

        // Update tracker or just draw raw detections
        std::vector<STrack> tracks;
        if (use_tracking) {
            tracks = tracker->update(objects);
        }

        auto end_infer = std::chrono::steady_clock::now();
        int inference_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_infer - start_infer).count();
        total_inference_ms += inference_ms;

        // Publish detections and tracks to ROS
        ros_node->publish_ball_detections(detections, tracks);

        // Draw results
        auto draw_start = std::chrono::steady_clock::now();
        for (size_t i = 0; i < detections.size(); ++i) {
            const auto& det = detections[i];
            float conf = det.conf;
            int classId = det.classId;

            if (conf < 0.25f) continue;

            int x1 = static_cast<int>(det.bbox[0]);
            int y1 = static_cast<int>(det.bbox[1]);
            int x2 = static_cast<int>(det.bbox[2]);
            int y2 = static_cast<int>(det.bbox[3]);

            cv::Scalar color;
            if (use_tracking && classId == 0) {
                continue; // Will be drawn below with ID
            } else {
                color = cv::Scalar(55 * classId % 255, 113 * classId % 255, 171 * classId % 255);
            }

            cv::rectangle(resized_cpu, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);

            std::string label = vClassNames[classId] + " " + std::to_string(conf).substr(0, 4);
            int baseline;
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseline);
            cv::rectangle(resized_cpu, cv::Point(x1, y1 - textSize.height - 4),
                          cv::Point(x1 + textSize.width, y1), color, -1);
            cv::putText(resized_cpu, label, cv::Point(x1, y1 - 2),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }

        // Draw tracking info
        if (use_tracking) {
            for (const auto& track : tracks) {
                std::vector<float> tlwh = track.tlwh;
                if (tlwh[2] * tlwh[3] <= 20) continue;

                cv::Scalar color = tracker->get_color(track.track_id);
                cv::Point p1(tlwh[0], tlwh[1]);
                cv::Point p2(tlwh[0] + tlwh[2], tlwh[1] + tlwh[3]);
                cv::rectangle(resized_cpu, p1, p2, color, 2);

                std::string label = "ID:" + std::to_string(track.track_id);
                cv::putText(resized_cpu, label, cv::Point(p1.x, p1.y - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
            }
        }
        auto draw_end = std::chrono::steady_clock::now();
        int draw_ms = std::chrono::duration_cast<std::chrono::microseconds>(draw_end - draw_start).count() / 1000;
        total_draw_ms += draw_ms;

        // Calculate overall FPS
        double avg_fps = frame_count * 1000.0 / (total_inference_ms + 1);
        double avg_camera_fps = camera_frame_count * 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(current_frame_time - camera_start_time).count();

        // Display FPS information
        std::string fps_text = cv::format("FPS: %.1f | Cam: %.1f | Trk: %s", avg_fps, avg_camera_fps, use_tracking ? "ON" : "OFF");
        cv::putText(resized_cpu, fps_text, cv::Point(10, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

        if (show_detailed_fps) {
            // Print detailed timing information to terminal
            std::cout << "Frame " << frame_count << " - ";
            std::cout << "Cap: " << capture_ms << "ms | ";
            std::cout << "Upload: " << upload_ms << "ms | ";
            std::cout << "Resize: " << resize_ms << "ms | ";
            std::cout << "Infer: " << inference_ms << "ms | ";
            std::cout << "Draw: " << draw_ms << "ms | ";
            std::cout << "Cam: " << camera_fps << "fps | ";
            std::cout << "Overall: " << avg_fps << "fps" << std::endl;
        }

        // Show in window
        auto display_start = std::chrono::steady_clock::now();
        cv::imshow("YOLOv8 + ByteTrack (resized 640x360)", resized_cpu);
        char key = cv::waitKey(1);
        if (key == 27 || key == 'q') break; // ESC or 'q' to quit
        auto display_end = std::chrono::steady_clock::now();
        int display_ms = std::chrono::duration_cast<std::chrono::microseconds>(display_end - display_start).count() / 1000;
        total_display_ms += display_ms;

        // Write full-size result only if processing video
        if (!is_webcam) {
            cv::Mat out_frame = frame.clone();
            YoloDetector::draw_image(out_frame, detections);
            writer.write(out_frame);
        }
    }

    auto total_time = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start_time
    ).count();

    std::cout << "\n--- Performance Summary ---" << std::endl;
    std::cout << "Processed " << frame_count << " frames in " << total_time << " seconds." << std::endl;
    if (total_time > 0) {
        std::cout << "Average throughput: " << static_cast<double>(frame_count) / total_time << " FPS" << std::endl;
    }
    std::cout << "Average capture time: " << (double)total_capture_ms / frame_count << " ms" << std::endl;
    std::cout << "Average upload time: " << (double)total_upload_ms / frame_count << " ms" << std::endl;
    std::cout << "Average resize time: " << (double)total_resize_ms / frame_count << " ms" << std::endl;
    std::cout << "Average inference time: " << (double)total_inference_ms / frame_count << " ms" << std::endl;
    std::cout << "Average draw time: " << (double)total_draw_ms / frame_count << " ms" << std::endl;
    std::cout << "Average display time: " << (double)total_display_ms / frame_count << " ms" << std::endl;

    if (tracker) delete tracker;
    cap.release();
    if (writer.isOpened()) writer.release();
    cv::destroyAllWindows();

    // Shutdown ROS if we initialized it
    rclcpp::shutdown();
    
    return 0;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: ./main [mode] [source] [track?] [show_details?]\n";
        std::cerr << "Modes:\n";
        std::cerr << "  1: Video file     --> ./main 1 ./videos/demo.mp4 [1|0] [0|1]\n";
        std::cerr << "  2: Webcam         --> ./main 2 0 [1|0] [0|1]\n";
        std::cerr << "track?: 1=enable tracking, 0=disable\n";
        std::cerr << "show_details?: 1=show detailed FPS info, 0=hide\n";
        std::cerr << "Example (webcam + tracking + details): ./main 2 0 1 1\n";
        std::cerr << "Example (webcam + tracking + no details): ./main 2 0 1 0\n";
        return -1;
    }

    int mode = std::stoi(argv[1]);
    std::string source = (argc >= 3) ? argv[2] : "";
    bool use_tracking = (argc >= 4) ? (std::stoi(argv[3]) != 0) : true;
    bool show_detailed_fps = (argc >= 5) ? (std::stoi(argv[4]) != 0) : false;

    return run(mode, source, use_tracking, show_detailed_fps);
}
