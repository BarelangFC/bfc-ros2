# YOLOv8 + TensorRT + ByteTrack + Field Segmentation

This directory contains an advanced vision module that combines YOLOv8 detection, ByteTrack tracking, and classical Field Segmentation (GMM + flood fill).

## Features
- **YOLOv8** object detection (TensorRT).
- **ByteTrack** for tracking the ball.
- **Field Segmentation** to filter out ball detections that are outside the field lines.
- **Camera Calibration** to undistort images using pre-calculated `npz` calibration matrices.
- ROS2 Node publishing filtered ball midpoints (`bytrseg/midpoints`).

## RUN

1. Go to the `detect` directory and convert the model to TensorRT engine (`.plan`). Make sure the `.plan` and `.npz` camera calibration files are present.
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
Usage: ./main [mode] [source] [track?] [show_details?] [save_mode?] [save_interval_ms?]
Modes:
  1: Video file     --> ./main 1 ./videos/demo.mp4 [1|0] [0|1]
  2: Webcam         --> ./main 2 0 [1|0] [0|1]
track?: 1=enable tracking, 0=disable
show_details?: 1=show detailed FPS info, 0=hide
save_mode?: 1=enable dataset auto-capture
save_interval_ms?: Interval between saves (e.g. 2000)
```
