# YOLOv8 + TensorRT + ByteTrack

This directory contains the standard ball detection and tracking module using YOLOv8 and ByteTrack.

## Features
- **YOLOv8** object detection accelerated with TensorRT.
- **ByteTrack** multi-object tracking.
- ROS2 Node publishing tracked ball midpoints (`yolo/ball_detections`).

## RUN

1. Go to the `detect` directory and convert the model to TensorRT engine (`.plan`) according to the `detect` directory instructions.
2. Based on the `detect` environment, install the following libraries:

```bash
apt install libeigen3-dev
```

3. Switch to current directory and start tracking:

```bash
mkdir build
cd build
cmake ..
make
./main 2 0 1 0  # Run webcam (device 0) with tracking enabled and no details
```

### Usage Arguments
```
Usage: ./main [mode] [source] [track?] [show_details?]
Modes:
  1: Video file     --> ./main 1 ./videos/demo.mp4 [1|0] [0|1]
  2: Webcam         --> ./main 2 0 [1|0] [0|1]
track?: 1=enable tracking, 0=disable
show_details?: 1=show detailed FPS info, 0=hide
```
