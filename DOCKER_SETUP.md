# Build Your Own Docker Environment

This tutorial creates a reusable Docker environment for the **FR3 Vision-Guided Sorting Cell**.

The design is simple:

> Docker installs ROS 2, MoveIt, Gazebo, RealSense and OpenCV. Your ROS 2 source code remains in a mounted workspace, so normal code changes do not require rebuilding the Docker image.

## Names used

| Item | Name |
|---|---|
| Project folder | `fr3_vision_sorting` |
| Docker image | `fr3-vision-sorting:jazzy` |
| Container | `fr3_vision_sorting` |
| ROS workspace | `ros2_ws` |
| ROS package | `fr3_vision_sorting` |

## 1. Clone the repository

```bash
cd ~
git clone https://github.com/ytang19-glitch/FR3-Vision-Guided-Sorting-Cell.git fr3_vision_sorting
cd ~/fr3_vision_sorting
```

The Docker files are already included:

```text
fr3_vision_sorting/
├── Dockerfile
├── compose.yaml
├── start_container.sh
├── DOCKER_SETUP.md
└── ros2_ws/
    └── src/
```

## 2. Understand the files

| File | Purpose |
|---|---|
| `Dockerfile` | Selects ROS 2 Jazzy and installs the dependencies |
| `compose.yaml` | Defines the container name, USB access, network and mounted folders |
| `start_container.sh` | Sources ROS 2 and the local workspace automatically |
| `.dockerignore` | Prevents temporary build files from entering the image |
| `ros2_ws/src/` | Stores your ROS 2 source packages |

The important bind mount is:

```yaml
- .:/workspace
```

It maps:

```text
Host:      ~/fr3_vision_sorting
Container: /workspace
```

Therefore, files created in `/workspace/ros2_ws/src` inside Docker are also saved on the host.

## 3. Build the Docker image

From the repository root:

```bash
cd ~/fr3_vision_sorting
docker compose build
```

Confirm it:

```bash
docker image ls | grep fr3-vision
```

Expected:

```text
fr3-vision-sorting   jazzy
```

## 4. Start the container

Allow GUI applications such as RViz:

```bash
xhost +local:docker
```

Start Docker:

```bash
docker compose up -d
```

Confirm:

```bash
docker ps
```

Enter the container:

```bash
docker exec -it fr3_vision_sorting bash
```

The prompt should now show that you are inside `/workspace`.

## 5. Create the ROS 2 package

Run these commands **inside the container**:

```bash
cd /workspace/ros2_ws/src

ros2 pkg create fr3_vision_sorting \
  --build-type ament_python \
  --dependencies \
  rclpy \
  geometry_msgs \
  sensor_msgs \
  visualization_msgs \
  moveit_msgs \
  shape_msgs \
  tf2_ros \
  tf2_geometry_msgs \
  cv_bridge
```

Create the future project folders:

```bash
cd /workspace/ros2_ws/src/fr3_vision_sorting
mkdir -p launch config worlds urdf models
```

Create node placeholders:

```bash
touch fr3_vision_sorting/color_detector.py
touch fr3_vision_sorting/depth_to_point.py
touch fr3_vision_sorting/object_pose_transformer.py
touch fr3_vision_sorting/planning_scene_node.py
touch fr3_vision_sorting/pick_place_node.py
touch fr3_vision_sorting/metrics_logger.py
```

## 6. Install package dependencies

Inside the container:

```bash
cd /workspace/ros2_ws

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y
```

## 7. Build the ROS workspace

```bash
cd /workspace/ros2_ws

colcon build \
  --symlink-install \
  --packages-select fr3_vision_sorting

source install/setup.bash
```

Confirm:

```bash
ros2 pkg prefix fr3_vision_sorting
```

Expected:

```text
/workspace/ros2_ws/install/fr3_vision_sorting
```

## 8. Test the RealSense

Connect the camera to the Ubuntu host. Inside Docker:

```bash
lsusb | grep -Ei 'intel|realsense|8086'
rs-enumerate-devices
```

If the host cannot see the camera, Docker cannot see it either. Check on the host with:

```bash
watch -n 1 lsusb
```

Reconnect the camera and look for a new USB line.

Start the ROS 2 RealSense driver:

```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  pointcloud.enable:=true \
  enable_sync:=true
```

Open a second host terminal and enter the same container:

```bash
docker exec -it fr3_vision_sorting bash
```

Check topics:

```bash
ros2 topic list | grep camera
```

Expected topics include:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/depth/color/points
```

Check the rates:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

## 9. View the camera

RGB image:

```bash
rqt_image_view
```

Point cloud:

```bash
rviz2
```

In RViz:

1. Set Fixed Frame to `camera_link`.
2. Add `PointCloud2`.
3. Select `/camera/camera/depth/color/points`.

If the frame is different:

```bash
ros2 topic echo /camera/camera/depth/color/points --once \
  --field header.frame_id
```

## 10. Daily workflow

Start:

```bash
cd ~/fr3_vision_sorting
xhost +local:docker
docker compose up -d
docker exec -it fr3_vision_sorting bash
```

Build the ROS package:

```bash
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Stop:

```bash
exit
docker compose down
```

## Docker build versus ROS build

| Command | Purpose |
|---|---|
| `docker compose build` | Builds the operating environment and installs dependencies |
| `docker compose up -d` | Starts the container |
| `colcon build --symlink-install` | Builds your ROS 2 packages |
| Editing a Python file | Changes your perception or robot program |

Rebuild Docker only after changing the `Dockerfile`. For normal ROS 2 Python changes, use `colcon build --symlink-install`.

## First milestone

Before controlling the Franka, complete this pipeline:

```text
RealSense
  → ROS 2 RGB image
  → aligned depth
  → point cloud in RViz
```

The next milestone is:

```text
RGB image
  → detect one red object
  → find center pixel (u, v)
  → read aligned depth Z
  → publish camera-frame position (X, Y, Z)
```

> The container does not automatically command the real Franka. Validate perception and simulation first.
