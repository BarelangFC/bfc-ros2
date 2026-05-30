# TensorRT deploy YOLOv8 / YOLO11 segment

This directory contains the YOLO Instance Segmentation module for pixel-level masking and detection.

## Get onnx

```bash
pip install ultralytics
pip install onnx==1.14.0
pip install onnxslim==0.1.34
```

write `export_onnx.py`, as follow:

```python
from ultralytics import YOLO

model = YOLO("./weights/yolo11s-seg.pt")
path = model.export(format="onnx", simplify=True, device=0, opset=12, dynamic=False, imgsz=640)
```

run `export_onnx.py`

```bash
python export_onnx.py
```

`yolo11s-seg.onnx` will be generated.

## To TensorRT

1. Switch to the current project directory.
2. If the model is trained on your own dataset, remember to check `include/config.h`. Ensure `onnxFile` points to the correct relative path (e.g., `./onnx_model/best.onnx`).
3. Confirm `cuda` and `tensorrt` paths in `CMakeLists.txt`.
4. Create an `onnx_model` directory and put the exported `onnx` model inside.
5. Run as follow to build and start inference:

```bash
mkdir build
cd build
cmake ..
make
./main 2 0 1 500  # Mode 2 (Webcam), device 0, save_mode 1, interval 500ms
```

### Usage Arguments
```
Usage: ./main [mode] [optional: image dir or webcam index] [save_mode] [save_interval_ms]
Modes:
  1: Image directory inference
  2: Webcam inference
```
