# Robotic Arm for Color-Based Cube Sorting Using ROS1

## Project Description

This project implements an intelligent robotic system in a simulation environment using a 5-DOF (Degrees of Freedom) robotic arm equipped with a camera and gripper. The robot autonomously:

1. **Detects** colored cubes (red, blue, green) placed in front of it
2. **Estimates** their 3D positions using computer vision and TF transformations
3. **Plans** motion trajectories using inverse kinematics
4. **Picks** each cube using the gripper
5. **Sorts** cubes into designated color zones

The entire system is implemented using **ROS1 (Robot Operating System)**, **Gazebo** for physics simulation, and **RViz** for visualization.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    GAZEBO SIMULATION                        │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────┐       │
│  │  5-DOF Arm  │  │ Colored Cubes│  │   Camera    │       │
│  │  + Gripper  │  │ (R, G, B)    │  │   Sensor    │       │
│  └─────────────┘  └──────────────┘  └─────────────┘       │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                      ROS1 NODES                             │
│  ┌──────────────────┐  ┌─────────────────┐                │
│  │ Color Detector   │  │ Position        │                │
│  │ (OpenCV)         │─▶│ Estimator (TF)  │                │
│  └──────────────────┘  └─────────────────┘                │
│           │                     │                           │
│           └─────────┬───────────┘                           │
│                     ▼                                       │
│  ┌──────────────────────────────────────┐                 │
│  │   Sorting Controller (Main Logic)    │                 │
│  └──────────────────────────────────────┘                 │
│           │                     │                           │
│           ▼                     ▼                           │
│  ┌─────────────────┐  ┌─────────────────┐                │
│  │ Motion Planner  │  │ Gripper         │                │
│  │ (IK + Trajectry)│  │ Controller      │                │
│  └─────────────────┘  └─────────────────┘                │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
                    ┌───────────────┐
                    │  RVIZ (Visual)│
                    └───────────────┘
```

---

## Project Structure

```
color_sorting_arm/
├── CMakeLists.txt
├── package.xml
├── README.md
│
├── urdf/
│   ├── arm.urdf.xacro
│   └── arm.gazebo.xacro
│
├── worlds/
│   └── sorting_world.world
│
├── config/
│   ├── controllers.yaml
│   └── sorting_arm.rviz
│
├── launch/
│   ├── complete_system.launch
│   ├── gazebo.launch
│   ├── rviz.launch
│   └── nodes.launch
│
├── scripts/
│   ├── color_detector.py
│   ├── position_estimator.py
│   ├── motion_planner.py
│   ├── gripper_controller.py
│   ├── sorting_controller.py
│   └── test_arm_movement.py
│
└── msg/
    ├── DetectedObject.msg
    ├── DetectedObjectArray.msg
    ├── Object3D.msg
    └── Object3DArray.msg
```

---

## Installation & Dependencies

### Prerequisites

- **Ubuntu 20.04** (recommended)
- **ROS Noetic** (ROS1)
- **Python 3**
- **Gazebo 11**

### Required ROS Packages

```bash
sudo apt update
sudo apt install -y \
    ros-noetic-desktop-full \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-joint-state-controller \
    ros-noetic-position-controllers \
    ros-noetic-effort-controllers \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-xacro \
    python3-opencv \
    python3-numpy
```

### Setup Catkin Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

git clone https://github.com/Abdallah-Labeb/arm.git
mv arm/color_sorting_arm .
rm -rf arm

cd ~/catkin_ws
catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Make Python Scripts Executable

```bash
cd ~/catkin_ws/src/color_sorting_arm/scripts
chmod +x *.py
```

---

## Usage

### Launch Complete System

```bash
roslaunch color_sorting_arm complete_system.launch
```

### Launch Components Separately

**Terminal 1 - Gazebo:**
```bash
roslaunch color_sorting_arm gazebo.launch
```

**Terminal 2 - RViz:**
```bash
roslaunch color_sorting_arm rviz.launch
```

**Terminal 3 - ROS Nodes:**
```bash
roslaunch color_sorting_arm nodes.launch
```

---

## How It Works

### 1. Color Detection (`color_detector.py`)
- Subscribes to `/camera/image_raw`
- Uses OpenCV HSV color segmentation
- Publishes detected objects to `/detected_objects`
- Publishes annotated image to `/detection_image`

### 2. Position Estimation (`position_estimator.py`)
- Converts 2D pixel coordinates to 3D world coordinates
- Uses TF transformations
- Publishes 3D positions to `/object_positions`

### 3. Motion Planning (`motion_planner.py`)
- Implements inverse kinematics for 5-DOF arm
- Plans trajectories for pick-and-place operations
- Publishes joint commands to position controllers

### 4. Gripper Control (`gripper_controller.py`)
- Controls gripper fingers
- Subscribes to `/gripper_command`
- Publishes to gripper joint controllers

### 5. Sorting Controller (`sorting_controller.py`)
- Main orchestration node
- Coordinates all other nodes
- Implements pick-and-place sorting logic

---

## Color Zones

| Color  | Zone Position (x, y, z) |
|--------|-------------------------|
| Red    | (0.6, 0.25, 0.05)      |
| Blue   | (0.6, -0.25, 0.05)     |
| Green  | (0.6, 0.0, 0.05)       |

---

## ROS Topics

### Published Topics:
- `/detected_objects` - DetectedObjectArray
- `/detection_image` - Image
- `/object_positions` - Object3DArray
- `/gripper_command` - Bool
- `/sorting_arm/joint_states` - JointState
- `/sorting_arm/joint{1-5}_position_controller/command` - Float64
- `/sorting_arm/gripper_{left,right}_position_controller/command` - Float64

### Subscribed Topics:
- `/camera/image_raw` - Image
- `/camera/camera_info` - CameraInfo

---

## Monitoring & Debugging

### View Camera Feed
```bash
rosrun image_view image_view image:=/detection_image
```

### Monitor Topics
```bash
rostopic list
rostopic echo /detected_objects
rostopic echo /object_positions
rostopic echo /sorting_arm/joint_states
```

### Check TF Tree
```bash
rosrun tf view_frames
evince frames.pdf
```

### RQT Graph
```bash
rqt_graph
```

---

## Configuration

### Color Detection Ranges
Edit `scripts/color_detector.py`:
```python
self.color_ranges = {
    'red': [...],
    'blue': [...],
    'green': [...]
}
```

### Sorting Zones
Edit `scripts/sorting_controller.py`:
```python
BIN_LOCATIONS = {
    'red': (0.6, 0.25, 0.05),
    'blue': (0.6, -0.25, 0.05),
    'green': (0.6, 0.0, 0.05)
}
```

### PID Controllers
Edit `config/controllers.yaml`:
```yaml
joint1_position_controller:
  type: position_controllers/JointPositionController
  joint: joint1
  pid: {p: 100.0, i: 0.01, d: 10.0}
```

---

## Troubleshooting

### Gazebo doesn't start or crashes
```bash
killall gzserver gzclient
roslaunch color_sorting_arm gazebo.launch
```

### Controllers fail to load
```bash
rosrun controller_manager controller_manager list
```

### Camera not publishing images
```bash
rostopic hz /camera/image_raw
```

### TF transform errors
```bash
rosrun tf tf_echo base_link camera_optical_frame
```

---

## Technical Specifications

### Robot Arm
- **DOF:** 5 (5 revolute joints)
- **Gripper:** Parallel gripper with 2 prismatic joints
- **Link Lengths:** L1=0.11m, L2=0.15m, L3=0.13m, L4=0.16m
- **Total Reach:** ~0.5m
- **Joint Limits:** Defined in URDF

### Camera
- **Type:** RGB Camera
- **Resolution:** 640x480
- **Format:** RGB8
- **FOV:** 1.2 radians (~68°)
- **Position:** Overhead at (0.35, 0, 1.0)
- **Orientation:** Pitch=90° (looking down)

### Cubes
- **Size:** 0.05m x 0.05m x 0.05m
- **Colors:** Red, Blue, Green
- **Mass:** 0.08 kg each
- **Position:** On table at z=0.725m

---

## License

MIT License

---

## References

- [ROS Wiki](http://wiki.ros.org/)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [OpenCV Documentation](https://docs.opencv.org/)
- [ROS Control](http://wiki.ros.org/ros_control)

---
