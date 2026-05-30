# BFC ROS2 Vision

This directory contains the computer vision modules for the BarelangFC ROS2 soccer robot system. The vision system is responsible for detecting the ball, field, and other objects of interest using YOLO models accelerated with TensorRT.

## Directories

There are three main subdirectories in this vision module, each serving a different purpose:

### 1. `yolov8_bytetrack`
This package contains a standard object detection pipeline using **YOLOv8** combined with **ByteTrack** for object tracking.
- **Purpose**: Fast and reliable ball detection and tracking.
- **Key Features**: Uses TensorRT for high-performance inference and ByteTrack for associating detections across frames to track the ball's trajectory.

### 2. `yolov8_bytrseg`
This package extends the object detection and tracking pipeline with **Field Segmentation**.
- **Purpose**: Detect and track the ball, but filter out false positives that occur outside the playing field.
- **Key Features**: 
  - **YOLOv8 + ByteTrack** for detection and tracking.
  - **Field Segmentation** using a Gaussian Mixture Model (GMM) combined with flood fill and region growing to segment the green soccer field.
  - **Camera Calibration** integration to undistort images before processing.
  - Only balls detected *inside* the segmented field are published to the ROS2 network.

### 3. `yolov8_segment`
This package contains a **YOLO instance segmentation** pipeline.
- **Purpose**: Detect objects and provide pixel-level masks (segmentation) for them.
- **Key Features**: Uses YOLOv8 or YOLOv11 segmentation models to accurately segment the ball and potentially the field boundaries at a pixel level.

## Model Configuration
Make sure you update the `config.h` and `main.cpp` files in their respective folders to point to your ONNX or TensorRT `.plan` files correctly.
