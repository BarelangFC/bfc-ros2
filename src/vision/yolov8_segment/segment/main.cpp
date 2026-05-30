#include "utils.h"
#include "infer.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "yolo_msgs/msg/midpoints.hpp"

#include <filesystem> // Untuk membuat direktori jika belum ada
#include <iomanip>    // Untuk format timestamp
#include <sstream>    // Untuk membuat nama file

class InferenceNode : public rclcpp::Node {
public:
    InferenceNode() : Node("inference_node") {
        //field_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("field_midpoint", 10);
        //ball_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("ball_midpoint", 10);
        midpoints_publisher_ = this->create_publisher<yolo_msgs::msg::Midpoints>("yolo/midpoints", 10);
    }

    /*void publish_field_midpoint(float x, float y) {
        geometry_msgs::msg::Point msg;
        msg.x = x;
        msg.y = y;
        field_publisher_->publish(msg);
    }

    void publish_ball_midpoint(float x, float y) {
        geometry_msgs::msg::Point msg;
        msg.x = x;
        msg.y = y:
        ball_publisher_->publish(msg);
    }*/
    
    void publishMidpoints(const std::vector<Detection>& detections) {
        yolo_msgs::msg::Midpoints msg;
        msg.header.stamp = this->now();
        
        for (const auto& det : detections) {
            if (det.conf < kConfThresh) continue;
            
            yolo_msgs::msg::Midpoint mp;
            mp.class_id = vClassNames[det.classId];
            mp.midpoint_x = static_cast<int>((det.bbox[0] + det.bbox[2]) / 2);
            mp.midpoint_y = static_cast<int>((det.bbox[1] + det.bbox[3]) / 2);
            msg.midpoints.push_back(mp);
        }
        
        midpoints_publisher_->publish(msg);
    }

private:
    //rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr field_publisher_;
    //rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ball_publisher_;
    rclcpp::Publisher<yolo_msgs::msg::Midpoints>::SharedPtr midpoints_publisher_;
};

// Fungsi untuk membuat direktori jika belum ada
void ensure_directory_exists(const std::string& path) {
    if (!std::filesystem::exists(path)) {
        std::filesystem::create_directories(path);
    }
}

int runWebcam(int webcamIndex, std::shared_ptr<InferenceNode> ros_node, bool saveMode, int saveIntervalMs) {
    // Open the webcam
    cv::VideoCapture cap(webcamIndex, cv::CAP_V4L2);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open webcam." << std::endl;
        return -1;
    }
    
    // Set camera parameters
    cap.set(cv::CAP_PROP_BRIGHTNESS, 0);              // Brightness
    cap.set(cv::CAP_PROP_CONTRAST, 15);               // Contrast
    cap.set(cv::CAP_PROP_SATURATION, 30);             // Saturation
    cap.set(cv::CAP_PROP_GAIN, 1);                    // Gain
    cap.set(cv::CAP_PROP_AUTO_WB, 44);              // Auto White Balance
    // cap.set(cv::CAP_PROP_WB_TEMPERATURE, 5346);    // White Balance
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 21);      // Auto Exposure
    //cap.set(cv::CAP_PROP_EXPOSURE, 2000);             // Exposure
    cap.set(cv::CAP_PROP_AUTOFOCUS, 39);            // Auto Focus
    // cap.set(cv::CAP_PROP_FOCUS, 0);                // Focus
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_GAMMA, 270); 
    cap.set(cv::CAP_PROP_SHARPNESS, 20); 
    //cap.set(cv::CAP_PROP_SETTINGS, 1);
    
    // Create detector and load engine plan
    std::string trtFile = "./yolo11s-seg.plan";
    YoloDetector detector(trtFile);
    
    // Directory to save resized frames
    std::string saveDir = "/home/barelangfc3/Pictures/datalatihanchina";
    ensure_directory_exists(saveDir);

    cv::Mat frame;
    cv::cuda::GpuMat gpuFrame, resizedGpuFrame;
    
    // Variables for saving frames
    auto lastSaveTime = std::chrono::steady_clock::now();
    int frameCount = 0;

    while (rclcpp::ok()) {
    	auto startCapture = std::chrono::system_clock::now();
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Error: Empty frame captured." << std::endl;
            break;
        }
        
        auto endCapture = std::chrono::system_clock::now();
        int captureTime = std::chrono::duration_cast<std::chrono::milliseconds>(endCapture - startCapture).count();

        auto startResize = std::chrono::system_clock::now();

        // Upload frame to GPU
        gpuFrame.upload(frame);

        // Resize frame
        int targetWidth = 640;
        int targetHeight = static_cast<int>(frame.rows * (targetWidth / static_cast<double>(frame.cols)));
        cv::cuda::resize(gpuFrame, resizedGpuFrame, cv::Size(targetWidth, targetHeight));
        cv::Mat resizedFrame;
        resizedGpuFrame.download(resizedFrame);
        
        // Save resized frame if saveMode is enabled
        if (saveMode) {
            auto currentTime = std::chrono::steady_clock::now();
            auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastSaveTime).count();

            if (elapsedMs >= saveIntervalMs) {
                // Generate unique filename with timestamp
                auto now = std::chrono::system_clock::now();
                auto in_time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
                std::string timestamp = ss.str();
                std::string filename = saveDir + "/frame_" + timestamp + ".jpg";

                // Save the resized frame
                cv::imwrite(filename, resizedFrame);
                std::cout << "Saved frame: " << filename << std::endl;

                // Update last save time
                lastSaveTime = currentTime;
            }
        }
        
        auto endResize = std::chrono::system_clock::now();
        int resizeTime = std::chrono::duration_cast<std::chrono::milliseconds>(endResize - startResize).count();

        auto startInference = std::chrono::system_clock::now();

        // Perform inference
        std::vector<Detection> res = detector.inference(resizedFrame);
        ros_node->publishMidpoints(res);

        /*cv::Point2f fieldMidpoint(-1, -1);  // Default values if not detected
        cv::Point2f ballMidpoint(-1, -1);

        for (const auto& det : res) {
            if (det.classId == 1) {  // Field
                fieldMidpoint.x = (det.bbox[0] + det.bbox[2]) / 2.0;
                fieldMidpoint.y = (det.bbox[1] + det.bbox[3]) / 2.0;
                ros_node->publish_field_midpoint(fieldMidpoint.x, fieldMidpoint.y);
            } else if (det.classId == 0) {  // Ball
                ballMidpoint.x = (det.bbox[0] + det.bbox[2]) / 2.0;
                ballMidpoint.y = (det.bbox[1] + det.bbox[3]) / 2.0;
                ros_node->publish_ball_midpoint(ballMidpoint.x, ballMidpoint.y);
            }
        }*/
        
        auto endInference = std::chrono::system_clock::now();
        int inferenceTime = std::chrono::duration_cast<std::chrono::milliseconds>(endInference - startInference).count();

        // Draw results
        YoloDetector::draw_image(resizedFrame, res);
        
        // Calculate FPS for inference
        double inferenceFPS = 1000.0 / inferenceTime;

        // Calculate total time (capture + resize + inference)
        int totalTime = captureTime + resizeTime + inferenceTime;
        double totalFPS = 1000.0 / totalTime;

        // Display timing information
        std::cout << "Capture time: " << captureTime << " ms, "
                  << "Resize time: " << resizeTime << " ms, "
                  << "Inference time: " << inferenceTime << " ms, "
                  << "Inference FPS: " << inferenceFPS << ", "
                  << "Total FPS: " << totalFPS << std::endl;


        // Display the results
        cv::imshow("Webcam Inference", resizedFrame);

        // Break on key press
        if (cv::waitKey(1) == 27) {  // Escape key
            break;
        }
    }

    return 0;
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<InferenceNode>();

    if (argc < 2) {
        printf("Usage: ./main [mode] [optional: image dir or webcam index] [save_mode] [save_interval_ms]\n");
        printf("Modes:\n");
        printf("  1: Image directory inference\n");
        printf("  2: Webcam inference\n");
        printf("Example for image inference: ./main 1 ./images\n");
        printf("Example for webcam inference: ./main 2 0 1 500\n");
        return 1;
    }

    int mode = std::stoi(argv[1]);
    if (mode == 1) {
        std::cerr << "ROS integration is only available for webcam mode.\n";
        return 1;
    } else if (mode == 2) {
        int webcamIndex = (argc >= 3) ? std::stoi(argv[2]) : 0;
        bool saveMode = false;
        int saveIntervalMs = 500; // Default interval

        if (argc >= 4) {
            saveMode = (std::stoi(argv[3]) == 1);
        }
        if (argc >= 5) {
            saveIntervalMs = std::stoi(argv[4]);
        }

        return runWebcam(webcamIndex, ros_node, saveMode, saveIntervalMs);
    } else {
        std::cerr << "Error: Invalid mode." << std::endl;
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
