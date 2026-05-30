# BarelangFC ROS 2 KidSize Soccer Robot

<p align="center">
  <img width="30%" src="Logo.png" alt="Barelang FC Logo" />
</p>

This repository contains the ROS 2 (Foxy) software stack for the BarelangFC KidSize Humanoid Soccer Robot. The architecture is designed to handle computer vision, game state management, motion control, and high-level strategic behavior autonomously.

## Repository Structure

The `src/` directory contains the essential ROS 2 packages and modules that form the complete robotic system:

* **`bfc_msgs`**
  Contains all the custom ROS 2 message (`.msg`) and service (`.srv`) definitions used across the different nodes for inter-process communication.

* **`game_controller`**
  Interfaces with the official RoboCup GameController. It listens to UDP broadcasts for the current game state (e.g., Initial, Ready, Set, Playing, Penalized) and parses this data for the robot's behavior system.

* **`main`**
  The core decision-making component of the robot. It contains the state machines and high-level strategy logic. It aggregates inputs from the vision system and the game controller to determine what the robot should do (e.g., search for the ball, align to kick, walk to a position) and sends the corresponding commands to the motion bridge.

* **`motion_bridge`**
  Acts as the bridge between the high-level ROS 2 behavior commands and the low-level physical robot motion controller. It translates strategic movement requests into actionable walking engine parameters or specific keyframe joint commands.

* **`vision`**
  The computer vision subsystem. It relies on TensorRT-accelerated YOLO models (YOLOv8/YOLO11) combined with ByteTrack. It includes pipelines for ball detection, object tracking, field segmentation, and instance segmentation. (See [`src/vision/README.md`](src/vision/README.md) for detailed vision documentation).

## Building the Workspace

This project is structured as a standard ROS 2 workspace. To build it, you need ROS 2 Foxy and `colcon` installed on your system. Note that some vision packages might require building natively using `cmake` before running them via ROS 2.

```bash
# 1. Navigate to the root of the workspace
cd bfc-ros2

# 2. Build the workspace
colcon build --symlink-install

# 3. Source the workspace overlay
source install/setup.bash
```

## Prerequisites & Dependencies
* **OS:** Ubuntu 20.04 (Recommended for ROS 2 Foxy)
* **ROS 2:** Foxy Fitzroy
* **Vision Dependencies:** CUDA, TensorRT, OpenCV, Eigen3
