# Stage 5 — RealSense D405 Perception

## Objective

Stage 4 established a working fixed-position pick-and-place baseline. Stage 5 adds the Intel RealSense D405 as a separate perception subsystem:

```text
Color image
    ↓
Detect the cube and obtain center pixel (u, v)
    ↓
Read the aligned depth Z at (u, v)
    ↓
Use the color-camera intrinsic parameters
    ↓
Calculate camera-frame point (X, Y, Z)
    ↓
Publish /object_point_camera
    ↓
Visualize and validate the point
```

At this stage, the camera only calculates and publishes the object's position. **Do not use the detected point to command the real FR3 yet.** The successful fixed-position pick-and-place program remains the control baseline.

## 1. Start the RealSense D405

Source ROS 2 and the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash
```

Launch the D405 with color-depth alignment, stream synchronization and the point cloud enabled:

```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_sync:=true \
  pointcloud.enable:=true
```

Depth alignment is essential because the object is detected in the color image. After alignment, color pixel `(u, v)` can be used to access the corresponding depth measurement.

## 2. Verify the Required Topics

List the camera topics:

```bash
ros2 topic list | grep camera
```

The perception node requires these inputs:

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

Depending on the installed RealSense ROS wrapper configuration, the exact namespace may differ. Always confirm it with `ros2 topic list` rather than hard-coding an unverified topic name.

Check the color and aligned-depth publication rates:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

Inspect the color-camera calibration message:

```bash
ros2 topic echo \
  /camera/camera/color/camera_info \
  --once
```

## 3. Inspect the Images

Open the ROS image viewer:

```bash
ros2 run rqt_image_view rqt_image_view
```

Inspect both topics:

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
```

Place the cube inside the D405 working range and verify that:

- The complete cube is visible in the color image.
- The aligned-depth image contains valid measurements on the cube.
- The camera is rigidly mounted and does not move during testing.
- The cube is not hidden by the robot or gripper.
- Depth values remain reasonably stable across multiple frames.

## 4. Camera Intrinsic Matrix

The `sensor_msgs/msg/CameraInfo` message contains the intrinsic matrix `K`:

```text
fx  0  cx
0  fy  cy
0   0   1
```

The important values are:

- `fx`, `fy`: focal lengths in pixels.
- `cx`, `cy`: optical center in pixels.

Use the `frame_id` supplied in the message header as the camera optical frame. Do not assume the frame name without checking the published message.

## 5. Pixel and Depth to Camera-Frame XYZ

For an object center at pixel `(u, v)` with depth `Z` in metres:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = aligned depth at pixel (u, v)
```

The result is the object's 3D position in the camera optical frame:

```text
P_camera = [X, Y, Z]
```

Confirm the depth image encoding before conversion:

```bash
ros2 topic echo \
  /camera/camera/aligned_depth_to_color/image_raw \
  --once \
  --field encoding
```

A `16UC1` depth image is commonly expressed in millimetres and must be converted to metres. A `32FC1` image is commonly expressed in metres. The implementation must verify the actual encoding instead of assuming the units.

A single center pixel can be noisy or invalid. A safer implementation samples a small region around `(u, v)`, rejects zero/invalid values and uses the median valid depth.

## 6. First Perception Node

Create:

```text
fr3_vision_sorting/camera_object_localizer.py
```

The first version should:

1. Subscribe to the color image.
2. Subscribe to the aligned-depth image.
3. Subscribe to the color `CameraInfo`.
4. synchronize the color and depth messages.
5. Detect the cube using OpenCV initially; YOLO can be added later.
6. Calculate the cube center pixel `(u, v)`.
7. Read a filtered aligned-depth value around the center.
8. Convert `(u, v, Z)` to camera-frame `(X, Y, Z)`.
9. Publish the result as `geometry_msgs/msg/PointStamped`.
10. Publish or display an annotated image containing the detection and depth.

Recommended output topic:

```text
/object_point_camera
```

The published message must contain the image timestamp and optical frame:

```python
point.header.stamp = image_msg.header.stamp
point.header.frame_id = image_msg.header.frame_id
point.point.x = x_camera
point.point.y = y_camera
point.point.z = z_camera
```

## 7. Stage 5 Validation Criteria

Stage 5 is complete only when:

- The cube is detected reliably in the color image.
- The center pixel is drawn on the correct object.
- Invalid and zero depth measurements are rejected.
- Camera-frame XYZ is published continuously.
- The reported Z distance agrees with a physical measurement.
- Moving the cube left, right, forward and backward changes XYZ consistently.
- The result is visualized and verified before any robot motion is permitted.

## 8. Software Boundary

Keep the working motion and perception systems separate:

```text
automatic_pick_place_demo.py
    └── Fixed-position manipulation baseline

camera_object_localizer.py
    └── Perception-only Stage 5 experiment
```

Do not modify the saved `GRASP` pose or `automatic_pick_place_demo.py` during this stage.

Stage 6 will estimate the transform between the camera optical frame and `fr3_link0`. Only after that transform is validated in RViz should camera measurements be converted into robot-base coordinates and used to generate a dynamic pre-grasp pose.

---

[Development Process Index](README.md) · [Previous Stage](stage_04_fixed_pick_place.md) · [Next Stage](stage_06_calibration_tf2.md)
