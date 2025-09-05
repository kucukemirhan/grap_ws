# GRAP (ROS 2 Humble)

ROS 2 Humble-based GRAP workspace. The system integrates robot description, control, planning, camera drivers, and perception nodes. With the bringup launch file, the entire stack can be started at once.

<p align="center">
ROS 2 Humble • MoveIt 2 • ros2_control • RealSense • YOLOv8 • ArUco
</p>

---

![System Picture](docs/img1.gif)  
*GAZEBO simulation of the system*

![System Picture](docs/img2.gif)  
*Tracking YOLOv8 object (Water Bottle)*

## Package Architecture

| Package | Responsibility |
|---------|----------------|
| **grap_description** | Defines the robot’s URDF/Xacro model: links, joints, sensors (camera mount), end-effector frames. Provides visualization in RViz and kinematic description to MoveIt. |
| **grap_moveit** | MoveIt configuration: SRDF, planning groups, kinematic solvers, and motion planning launch files. |
| **grap_controller** | `ros_control` configuration: controller manager, hardware interface plugin, and YAML configs (e.g., `joint_position_controller`). |
| **grap_bringup** | Top-level launch files: robot_state_publisher, controllers, MoveIt, perception, camera drivers. |
| **move_program** | Application-level nodes that orchestrate task-specific behaviors. |
| **realsense_ros** | NVIDIA RealSense driver wrapper: publishes color/depth topics, camera info, and point clouds. [More info](https://github.com/mgonzs13/yolo_ros.git) |
| **aruco_ros** | ArUco marker detection: for workspace calibration, publishes marker poses relative to the camera. [More info](https://github.com/pal-robotics/aruco_ros.git) ||

---

## Installation

```bash
# Create workspace
mkdir -p ~/grap_ws/src
cd ~/grap_ws

# Clone repository
git clone https://github.com/kucukemirhan/grap_ws.git .

# Source ROS 2 Humble environment
source /opt/ros/humble/setup.bash

# Install dependencies
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Python dependencies for YOLO
python3 -m pip install --user ultralytics opencv-python
```

---

## Build

```bash
cd ~/grap_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Launch Sequence

Follow this order to ensure all subsystems are initialized correctly:

### 1. Full robot bringup
```bash
ros2 launch grap_bringup real_robot.launch.py
```
> Starts: `robot_description`, controller_manager, STM32 bridge, MoveIt, perception pipeline.

---

### 2. RealSense camera
```bash
ros2 launch realsense2_camera rs_launch.py publish_tf:=false   depth_module.profile:=640x480x30   rgb_camera.profile:=640x480x30
```
> Publishes color + depth streams (TF already handled by URDF).

---

### 3. YOLO perception
```bash
ros2 launch yolo_bringup yolo_distance.launch.py   input_image_topic:=/camera/color/image_raw   input_depth_topic:=/camera/depth/image_rect_raw   input_depth_info_topic:=/camera/depth/camera_info   device:=cpu   target_frame:=camera_link   use_tracking:=true
```
> Detects objects in the camera feed and estimates 3D positions.

---

### 4. Image view (manual inspection)
```bash
ros2 run rqt_image_view rqt_image_view
```
> Visualize any available image topic (e.g., `/camera/color/image_raw`, `/detection/image`).

---

### 5. Task program (automated movements)
```bash
ros2 launch move_program move_program.launch.py
```
> Executes high-level grasping tasks using perception and MoveIt.

---

## Sending Commands

To command the robot to a specific target frame (e.g., `home`, `stop`, `chair1`), publish to `/target_frame`:

```bash
# Move to “home”
ros2 topic pub /target_frame std_msgs/msg/String "{data: 'home'}" --once

# Move to “stop”
ros2 topic pub /target_frame std_msgs/msg/String "{data: 'stop'}" --once

# Move to “chair1”
ros2 topic pub /target_frame std_msgs/msg/String "{data: 'chair1'}" --once
```

---