# army_gpd — JetArm GPD Grasping System

A 6-DOF robotic arm grasping system based on [GPD (Grasp Pose Detection)](https://github.com/atenpas/gpd), running on the **ROS Melodic + JetArm (Hiwonder)** platform. It consists of three main modules: point cloud processing, grasp planning, and an interactive UI.

---

## Directory Structure

```
army_gpd/
├── gpd/                  # GPD Core Library (C++, includes pre-trained models)
│   ├── cfg/              #   ros_eigen_params.cfg — Main configuration file
│   ├── models/           #   LeNet / Caffe / OpenVINO model weights
│   ├── src/              #   C++ source code
│   └── include/          #   Header files
├── gpd_ros/              # ROS wrapper for GPD
│   ├── launch/           #   gpd_once.launch, ur5.launch
│   ├── msg/              #   GraspConfig, GraspConfigList, CloudIndexed …
│   ├── srv/              #   detect_grasps.srv
│   └── src/              #   C++ ROS nodes
├── jetarm_ui/            # Control UI + Grasp Node + Point Cloud Bridge
│   ├── scripts/          #   Python scripts (UI, grasping, point cloud transformation)
│   ├── launch/           #   Launch files
│   └── config/           #   YAML configurations
├── user_config/          # User-level independent configs (not overwritten by deployment)
├── start_tf_ui.sh        # One-click startup script (deployed to the robotic arm desktop)
└── README.md
```

---

## System Architecture

```
┌──────────────────────────────────────────────────────────┐
│                    RGB-D Camera (Astra)                  │
│  /rgbd_cam/depth/points    /rgbd_cam/color/camera_info   │
│  /rgbd_cam/depth/image_raw /color_detection/image_result │
└──────────┬──────────────────────────┬────────────────────┘
           │                          │
     PointCloud2                  CameraInfo / Image
           │                          │
           ▼                          ▼
┌─────────────────────┐   ┌───────────────────────────────┐
│ pointcloud_to_base  │   │  tf_calibration_control_ui    │
│ (TF Transform +     │   │  (PyQt5 Interactive Control UI)│
│  Downsampling)      │   │  · TF Calibration             │
│                     │   │  · Candidate Grasp Visualizer │
│ depth → base_link   │   │    & Manual Selection         │
└────────┬────────────┘   │  · Speed/Offset/Param Tuning  │
         │                │  · Single GPD Scan Trigger    │
    PointCloud2           └──────┬────────────────────────┘
    (base_link)                  │ Trigger service
         │                       ▼
         │            ┌─────────────────────┐
         └───────────▶│   gpd_once_server   │  (gpd_ros)
                      │ detect_grasps Srv   │
                      └────────┬────────────┘
                               │ GraspConfigList
                               ▼
                      ┌─────────────────────┐
                      │   gpd_grasp_node    │
                      │ Score/Sort/IK/Exec  │
                      │ Angled Push+Edge Avoid│
                      └────────┬────────────┘
                               │ MultiRawIdPosDur
                               ▼
                      ┌─────────────────────┐
                      │ JetArm Servo Ctrl   │
                      │ /controllers/...    │
                      └─────────────────────┘
```

---

## Required External Interfaces

### 1. ROS Topics (Provided Externally)

| Topic | Message Type | Provider | Description |
|-------|---------|--------|------|
| `/rgbd_cam/depth/points` | `sensor_msgs/PointCloud2` | RGB-D Camera Driver | Raw depth point cloud |
| `/rgbd_cam/depth/image_raw` | `sensor_msgs/Image` | RGB-D Camera Driver | Depth image (UI display) |
| `/rgbd_cam/color/camera_info` | `sensor_msgs/CameraInfo` | RGB-D Camera Driver | Camera intrinsics (for projection) |
| `/color_detection/image_result` | `sensor_msgs/Image` | color_detection node | RGB image (UI display) |
| `/object/pixel_coords` | `hiwonder_interfaces/ObjectsInfo` | color_detection node | Object detection results |

### 2. ROS Topics (Published by this System)

| Topic | Message Type | Publisher | Description |
|-------|---------|--------|------|
| `/rgbd_cam/depth/points_base` | `sensor_msgs/PointCloud2` | `pointcloud_to_base` | Point cloud in base_link frame |
| `/detect_grasps/clustered_grasps` | `gpd_ros/GraspConfigList` | `gpd_once_server` | GPD candidate grasp list |
| `/controllers/multi_id_pos_dur` | `hiwonder_interfaces/MultiRawIdPosDur` | `gpd_grasp_node` / UI | Servo motion commands |
| `/gpd_grasp/debug_markers` | `visualization_msgs/MarkerArray` | `gpd_grasp_node` | RViz debug markers |
| `/gpd_once_server/plot_grasps` | `visualization_msgs/MarkerArray` | `gpd_once_server` | GPD gripper visualization |

### 3. ROS Services (Provided Externally)

| Service | Type | Provider | Description |
|---------|------|--------|------|
| `/color_detection/enter` | `std_srvs/Empty` | color_detection pkg | Enter color detection mode |
| `/color_detection/start` | `std_srvs/Empty` | color_detection pkg | Start detection |
| `/color_detection/stop` | `std_srvs/Empty` | color_detection pkg | Stop detection |
| `/color_detection/exit` | `std_srvs/Empty` | color_detection pkg | Exit detection mode |
| `/rgbd_cam/set_uvc_auto_exposure` | `std_srvs/SetBool` | astra_camera driver | Auto-exposure toggle |
| `/rgbd_cam/set_uvc_exposure` | `astra_camera/SetInt32` | astra_camera driver | Manual exposure value |
| `/rgbd_cam/set_uvc_auto_white_balance` | `std_srvs/SetBool` | astra_camera driver | Auto white-balance toggle |
| `/lab_config_manager/enter` | `std_srvs/Trigger` | lab_config pkg | Load LAB color config |

### 4. ROS Services (Provided by this System)

| Service | Type | Provider | Description |
|---------|------|--------|------|
| `/gpd_once_server/detect_grasps` | `gpd_ros/detect_grasps` | `gpd_once_server` | Single grasp detection |
| `/gpd_grasp/trigger` | `std_srvs/Trigger` | `gpd_grasp_node` | Trigger a single grasp execution |
| `/gpd_grasp/clear_cache` | `std_srvs/Trigger` | `gpd_grasp_node` | Clear cached grasp results |

### 5. TF Coordinate Transformations (Provided Externally)

| Parent | Child | Provider | Description |
|--------|-------|--------|------|
| `base_link` | `link1` … `link5` | Robot URDF / `robot_state_publisher` | Robotic arm joint chain |
| `rgbd_cam_link` | `rgbd_cam_color_optical_frame` | Camera Driver (astra) | Camera optical frame |
| `rgbd_cam_link` | `rgbd_cam_depth_optical_frame` | Camera Driver (astra) | Depth optical frame |

> **Note**: `link5 → rgbd_cam_link` is published by the internal `static_transform_publisher` or the UI's TF calibration module, configured in `tf_calibration.yaml`.

### 6. ROS Actions (Provided Externally)

| Action | Type | Provider | Description |
|--------|------|--------|------|
| `/grasp` | `hiwonder_interfaces/MoveAction` | JetArm Motion Controller | Robotic arm trajectory execution |

### 7. Servo Interfaces

| Topic | Message Type | Description |
|-------|---------|------|
| `/controllers/multi_id_pos_dur` | `hiwonder_interfaces/MultiRawIdPosDur` | Send Servo ID + Position + Duration |

Servo IDs: ID 1–5 are joint servos, ID 10 is the gripper servo.

### 8. External ROS Package Dependencies

| Package Name | Purpose | Installation |
|------|------|---------|
| `jetarm_bringup` | Robot base launch (base.launch) | JetArm SDK |
| `jetarm_peripherals` | Camera launch (camera.launch) | JetArm SDK |
| `jetarm_driver` | Servo driver | JetArm SDK |
| `hiwonder_interfaces` | Custom messages/actions | JetArm SDK |
| `color_detection` | Color detection node | JetArm SDK |
| `lab_config` | LAB color config management | JetArm SDK |
| `astra_camera` | Astra RGB-D driver | `ros-melodic-astra-camera` |
| `jetarm_6dof_moveit_config` | MoveIt config (RViz visualization) | JetArm SDK |
| `tf2_ros` / `tf2_sensor_msgs` | TF coordinate transformations | `ros-melodic-tf2-*` |

### 9. System-Level Dependencies

| Library | Version Requirement | Purpose |
|----|---------|------|
| **PCL** | ≥ 1.9 | Point cloud processing |
| **Eigen** | ≥ 3.0 | Linear algebra operations |
| **OpenCV** | ≥ 3.4 | Image processing / CNN inference |
| **Python 3** + PyQt5 | — | UI interface |
| **ROS Melodic** | — | Base framework |

---

## Quick Deployment

### 1. Compile GPD Core Library

```bash
cd gpd
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 2. Compile ROS Packages

Place `gpd_ros` and `jetarm_ui` into your catkin workspace:

```bash
cp -r gpd_ros jetarm_ui ~/jetarm/src/
cd ~/jetarm
catkin_make
source devel/setup.bash    # or setup.zsh
```

### 3. Configuration

- **Camera Calibration**: Edit the `transform` field in `jetarm_ui/config/tf_calibration.yaml`
- **Grasp Offset**: Edit the `grasp_offset` field, or adjust in real-time via the UI
- **GPD Parameters**: Edit `gpd/cfg/ros_eigen_params.cfg` (workspace, sample count, etc.)
- **Model Path**: Ensure `weights_file` points to the correct `models/lenet/15channels/params/` path

### 4. Startup

```bash
# Method 1: Use the startup script (Recommended)
chmod +x start_tf_ui.sh
./start_tf_ui.sh

# Method 2: Manual roslaunch
roslaunch jetarm_ui tf_calibration_control_ui.launch
```

---

## Key Configuration Files

| File | Description |
|------|------|
| `gpd/cfg/ros_eigen_params.cfg` | Core GPD parameters (workspace, sample count, CNN channels, etc.) |
| `jetarm_ui/config/tf_calibration.yaml` | TF calibration values + grasp offsets + GPD tuning params |
| `jetarm_ui/config/ui_config.yaml` | Default UI configuration (camera topic, arm poses, etc.) |
| `user_config/ui_config.user.yaml` | User-independent configuration (not overwritten by code updates) |

---

## Core GPD Parameter Description

| Parameter | Default Value | Description |
|------|--------|------|
| `num_samples` | 40 | Number of candidate points sampled from the point cloud |
| `num_threads` | 4 | Number of CPU threads |
| `workspace` | `-0.05 0.36 -0.30 0.30 -0.05 0.40` | Point cloud cropping volume (m) |
| `workspace_grasps` | (Same as above) | Bounding box for filtering grasp candidates |
| `num_selected` | 20 | Number of final grasp candidates output |
| `image_num_channels` | 15 | Number of CNN input channels |
| `weights_file` | `models/lenet/15channels/params/` | Path to model weights |

---

## Custom Messages/Services (gpd_ros)

### Messages

| Message | Fields |
|------|------|
| `GraspConfig` | `position`, `approach`, `binormal`, `axis`, `width`, `score`, `sample` |
| `GraspConfigList` | `header`, `grasps[]` |
| `CloudIndexed` | `cloud_sources`, `indices[]` |
| `CloudSources` | `cloud`, `camera_source[]`, `view_points[]` |

### Services

| Service | Request | Response |
|------|------|------|
| `detect_grasps` | `CloudIndexed cloud_indexed` | `GraspConfigList grasp_configs` |

---

## License

The GPD core library is licensed under the BSD License. The remaining code is for learning and research purposes only.
