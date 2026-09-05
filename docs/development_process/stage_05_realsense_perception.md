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

For the currently tested D405 color stream:

```text
fx = 426.5189 pixels
fy = 426.0117 pixels
cx = 425.4981 pixels
cy = 240.6794 pixels
```

These values are an experimental reference only. Read them from `CameraInfo`
at runtime because changing the resolution or stream profile can change the
intrinsics.

Use the `frame_id` supplied in the message header as the camera optical frame. Do not assume the frame name without checking the published message.

## 5. Pixel and Depth to Camera-Frame XYZ

For an object center at pixel `(u, v)` with depth `Zc` in metres:

```text
Xc = (u - cx) * Zc / fx
Yc = (v - cy) * Zc / fy
Zc = aligned depth at pixel (u, v)
```

The subscript `c` means that `Xc`, `Yc` and `Zc` are expressed in the
**camera optical coordinate frame**.

| Variable | Unit | Physical meaning |
|---|---:|---|
| `u` | pixels | Horizontal coordinate (image column) of the detected object point |
| `v` | pixels | Vertical coordinate (image row) of the detected object point |
| `cx` | pixels | Horizontal coordinate where the optical axis intersects the image |
| `cy` | pixels | Vertical coordinate where the optical axis intersects the image |
| `fx` | pixels | Horizontal focal length; converts horizontal viewing direction into pixel displacement |
| `fy` | pixels | Vertical focal length; converts vertical viewing direction into pixel displacement |
| `Xc` | metres | Object's physical left-right displacement from the camera optical axis |
| `Yc` | metres | Object's physical up-down displacement from the camera optical axis |
| `Zc` | metres | Object's forward distance from the camera |

The formulas are the inverse of the pinhole-camera projection equations:

```text
u = fx * Xc / Zc + cx
v = fy * Yc / Zc + cy
```

Their physical meaning is:

1. `u - cx` and `v - cy` measure the pixel displacement from the optical
   center.
2. Dividing by `fx` or `fy` converts the pixel displacement into a
   normalized camera-ray direction.
3. Multiplying by the measured depth `Zc` scales that direction into a
   physical displacement in metres.

The signs follow the ROS optical-frame convention:

- If `u > cx`, then `Xc > 0`: the point is to the image's right.
- If `u < cx`, then `Xc < 0`: the point is to the image's left.
- If `v > cy`, then `Yc > 0`: the point is below the optical center.
- If `v < cy`, then `Yc < 0`: the point is above the optical center.
- If `(u, v) = (cx, cy)`, then `Xc = Yc = 0`: the point lies on the
  camera's optical `+Z` axis.

In matrix form:

```text
                 [u]
P_camera = Zc K⁻¹[v]
                 [1]

P_camera = [Xc, Yc, Zc]ᵀ
```

One RGB pixel describes a ray rather than a unique 3D point. The aligned-depth
measurement `Zc` determines where the object lies along that ray.

Using the measured D405 values and the example

```text
u = 500 pixels
v = 300 pixels
Zc = 0.4000 m
```

gives

```text
Xc = (500 - 425.4981) * 0.4000 / 426.5189 = 0.0699 m
Yc = (300 - 240.6794) * 0.4000 / 426.0117 = 0.0557 m
Zc = 0.4000 m
```

Therefore:

```text
P_camera = [0.0699, 0.0557, 0.4000] m
```

This point is approximately **6.99 cm to the right**, **5.57 cm down**, and
**40.00 cm in front** of the camera optical origin. It is not yet expressed in
the FR3 base frame `fr3_link0`.

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

## 7. How to Test the Perception Node

Keep the FR3 stationary during these tests. This procedure validates perception
only; it must not send any motion command to the robot.

### 7.1 Install the required packages

Inside the ROS 2 container:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-cv-bridge \
  ros-jazzy-message-filters \
  python3-opencv \
  python3-numpy
```

The package must also declare `rclpy`, `sensor_msgs`, `geometry_msgs`,
`cv_bridge` and `message_filters` in `package.xml`.

### 7.2 Build and source the package

```bash
cd /workspace/ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install \
  --packages-select fr3_vision_sorting

source install/setup.bash
```

Confirm that ROS 2 can find the executable:

```bash
ros2 pkg executables fr3_vision_sorting | grep camera_object_localizer
```

Expected output:

```text
fr3_vision_sorting camera_object_localizer
```

If nothing appears, confirm that `setup.py` contains:

```python
"camera_object_localizer = "
"fr3_vision_sorting.camera_object_localizer:main",
```

Then rebuild and source the workspace again.

### 7.3 Start the D405

Terminal 1:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_sync:=true
```

Confirm the three required topics:

```bash
ros2 topic list | grep -E \
  "color/image_raw|aligned_depth_to_color/image_raw|color/camera_info"
```

Check that color and aligned depth are publishing:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

### 7.4 Verify the raw camera view

Terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

First select:

```text
/camera/camera/color/image_raw
```

The overhead view should clearly show the complete target object, as in the
current experiment. Ensure that the target is not merged visually with the
robot, cardboard platform or black table covering. Next select:

```text
/camera/camera/aligned_depth_to_color/image_raw
```

Verify that the target area contains valid, stable depth rather than zero or
flickering measurements.

### 7.5 Run the localizer

Terminal 3:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash

ros2 run fr3_vision_sorting camera_object_localizer
```

Expected startup information includes the received camera intrinsics and
optical-frame name. The node should then print detected camera-frame
coordinates similar to:

```text
Xc=0.0123 m, Yc=0.0410 m, Zc=0.4580 m
```

The exact values depend on the object's actual position.

### 7.6 Inspect the published 3D point

Terminal 4:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash

ros2 topic echo /object_point_camera
```

A valid message should resemble:

```yaml
header:
  frame_id: camera_color_optical_frame
point:
  x: 0.0123
  y: 0.0410
  z: 0.4580
```

Confirm the message rate:

```bash
ros2 topic hz /object_point_camera
```

If no message appears, check the localizer terminal. Common causes are an
incorrect HSV color range, a contour smaller than the minimum-area threshold,
invalid depth, mismatched topic names or image synchronization failure.

### 7.7 Inspect the annotated result

In `rqt_image_view`, select:

```text
/camera_object_localizer/annotated_image
```

A successful detection must show:

- A contour around the intended target only.
- A center marker at the target's visual center.
- The detected pixel `(u, v)`.
- The calculated `(Xc, Yc, Zc)` in metres.
- A stable detection across multiple frames.

If the contour detects the cardboard, table or robot instead of the target,
adjust the HSV limits under the actual laboratory lighting. Do not proceed by
accepting the wrong contour.

### 7.8 Validate the coordinate directions

Move the target by hand while keeping the camera fixed:

| Target movement in the image | Expected coordinate change |
|---|---|
| Move right | `u` increases and `Xc` increases |
| Move left | `u` decreases and `Xc` decreases |
| Move down | `v` increases and `Yc` increases |
| Move up | `v` decreases and `Yc` decreases |
| Move closer to the camera | `Zc` decreases |
| Move farther from the camera | `Zc` increases |

These are camera optical-frame directions. With an overhead camera, they must
not be interpreted as FR3 base-frame directions until Stage 6 calibration is
complete.

### 7.9 Check metric accuracy

1. Place the target near the optical center.
2. Measure the physical camera-to-target distance with a ruler or tape measure.
3. Record at least 30 published `Zc` samples.
4. Compare their median with the physical distance.
5. Repeat at several positions within the intended workspace.

A useful test record is:

| Test | Physical distance | Median reported `Zc` | Absolute error | Stable detection? |
|---|---:|---:|---:|---|
| Center | ___ m | ___ m | ___ m | Yes/No |
| Left | ___ m | ___ m | ___ m | Yes/No |
| Right | ___ m | ___ m | ___ m | Yes/No |

Also verify that `Xc` and `Yc` approach zero when the selected target point
is placed near `(cx, cy)`.

### 7.10 Test failure handling

Deliberately perform perception-only failure tests:

- Remove the target: the node should publish no new object point.
- Cover the target: detection should stop rather than jump to another object.
- Place the target outside the depth range: invalid depth should be rejected.
- Temporarily block the camera: the node should not publish fabricated XYZ.
- Move the target partly outside the image: small or incomplete contours should
  be rejected.
- Change the lighting: verify whether HSV segmentation remains reliable.

The node must fail safely by withholding the 3D point. It must never reuse a
stale detection as though it were current.

## 8. Stage 5 Validation Criteria

Stage 5 is complete only when:

- The cube is detected reliably in the color image.
- The center pixel is drawn on the correct object.
- Invalid and zero depth measurements are rejected.
- Camera-frame XYZ is published continuously.
- The reported Z distance agrees with a physical measurement.
- Moving the cube left, right, forward and backward changes XYZ consistently.
- The result is visualized and verified before any robot motion is permitted.

## 9. Software Boundary

Keep the working motion and perception systems separate:

```text
automatic_pick_place_demo.py
    └── Fixed-position manipulation baseline

camera_object_localizer.py
    └── Perception-only Stage 5 experiment
```

Do not modify the saved `GRASP` pose or `automatic_pick_place_demo.py` during this stage.

Stage 6 will estimate the transform between the camera optical frame and `fr3_link0`. Only after that transform is validated in RViz should camera measurements be converted into robot-base coordinates and used to generate a dynamic pre-grasp pose.

## 10. Troubleshooting — RealSense D405 in Docker

During Stage 5 testing, the D405 may be visible to the Linux USB layer but still unavailable to librealsense and the ROS 2 RealSense driver.

### 10.1 Observed failure

The host Ubuntu machine detected the D405 correctly:

```text
Bus 001 Device 016: ID 8086:0b5b Intel Corp. Intel(R) RealSense(TM) Depth Camera 405
```

Inside the Docker container, `lsusb` also detected the camera, and `/dev/video0` through `/dev/video5` were present. However, librealsense still failed:

```bash
rs-enumerate-devices
```

returned:

```text
No device detected. Is it plugged in?
```

The ROS 2 driver reported:

```text
[WARN] [camera.camera]: No RealSense devices were found!
```

The warning

```text
No valid configuration file found at : /root/.realsense-config.json loading defaults
```

was also present, but this was not the root cause. The important failure was that librealsense inside the container could not enumerate the physical D405.

### 10.2 Host-vs-container result that isolated the problem

Running `rs-enumerate-devices` directly on the Ubuntu host succeeded and returned the D405 information, including:

```text
Name                : Intel RealSense D405
Product Id          : 0B5B
Firmware Version    : 5.16.0.1
Connection Type     : USB
Usb Type Descriptor : 2.1
```

The host device path also referenced a RealSense V4L2 node such as:

```text
.../video4linux/video6
```

Therefore the final diagnostic state was:

```text
Physical D405                      ✅
Ubuntu host USB                    ✅
Ubuntu host librealsense           ✅
Docker USB visibility              ✅
Docker /dev/video* visibility      partial / inconsistent
Docker librealsense                ❌
ROS realsense2_camera in Docker    ❌
```

This is important because it rules out a defective camera, bad firmware and a broken host librealsense installation. The remaining problem is isolated to **container device/sysfs visibility or Docker runtime configuration**.

### 10.3 Why `lsusb` alone was misleading

`lsusb` only proves that the container can read the USB descriptor. It does not prove that librealsense can correctly associate the USB device with its V4L2 and sysfs interfaces.

The host saw the RealSense through a path involving `video6`, while the container initially exposed only `video0` through `video5`. This mismatch suggested that the container did not have the same device-node/sysfs view as the host.

Therefore, do not debug `camera_object_localizer.py` or ROS topic names until `rs-enumerate-devices` works inside the container.

### 10.4 Dockerfile cleanup vs actual fix

The Dockerfile was cleaned by removing duplicate package entries such as repeated `ros-jazzy-cv-bridge` and `python3-opencv` lines.

This cleanup is useful because it keeps the image definition simpler and avoids redundant package declarations, but it **does not fix the RealSense enumeration failure**.

The relevant packages remain:

```text
usbutils
v4l-utils
ros-jazzy-realsense2-camera
ros-jazzy-realsense2-description
ros-jazzy-cv-bridge
python3-opencv
```

The actual RealSense issue is at runtime, where Docker must expose the same hardware interfaces that work on the host.

### 10.5 Check the host first

On the Ubuntu host:

```bash
lsusb | grep -i realsense
rs-enumerate-devices
```

If `rs-enumerate-devices` succeeds on the host, do not reinstall the host RealSense stack. Move immediately to Docker runtime debugging.

If needed, install V4L2 tools on the host with:

```bash
sudo apt install v4l-utils
```

and then inspect:

```bash
v4l2-ctl --list-devices
ls -l /dev/video*
```

### 10.6 Check the container

Inside the ROS 2 container:

```bash
lsusb | grep -i realsense
ls -l /dev/video*
ls -l /sys/class/video4linux
v4l2-ctl --list-devices
rs-enumerate-devices
```

Use the following interpretation:

| Result | Meaning |
|---|---|
| Host `rs-enumerate-devices` fails | Host/USB/librealsense problem |
| Host works, Docker `lsusb` fails | USB passthrough problem |
| Docker `lsusb` works, `/dev/video*` missing | V4L2 device passthrough problem |
| Docker sees video nodes but host and container numbering/sysfs differ | Container `/dev` or `/sys` visibility problem |
| Docker `rs-enumerate-devices` works, ROS fails | ROS wrapper/configuration problem |

### 10.7 Docker configuration used for RealSense access

The updated container configuration uses host networking and broad device access for the FR3 and D405:

```yaml
services:
  fr3_ros:
    build:
      context: .
      dockerfile: Dockerfile

    container_name: fr3_ros

    privileged: true
    network_mode: host

    stdin_open: true
    tty: true

    environment:
      DISPLAY: ${DISPLAY}
      ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}

    volumes:
      - ./ros2_ws:/workspace/ros2_ws
      - /dev:/dev
      - /sys:/sys:ro
      - /tmp/.X11-unix:/tmp/.X11-unix:rw

    device_cgroup_rules:
      - "c 81:* rmw"
      - "c 189:* rmw"

    working_dir: /workspace
    command: bash
```

Relevant device classes:

```text
81  → V4L2/video devices
189 → USB devices
```

The `/dev:/dev` mount exposes the host device nodes. The `/sys:/sys:ro` mount gives userspace tools a consistent read-only view of the host device topology used to associate USB and V4L2 interfaces.

A narrow mount such as:

```yaml
- /dev/bus/usb:/dev/bus/usb
```

may be enough for `lsusb` while still being insufficient for librealsense.

### 10.8 Recreate the container after changing runtime access

Changes to `docker-compose.yml` do not fully apply to an already-created container. Recreate it:

```bash
docker compose down
docker compose build
docker compose up -d
```

Then enter it again:

```bash
docker exec -it fr3_ros bash
```

Run:

```bash
lsusb | grep -i realsense
ls -l /dev/video*
ls -l /sys/class/video4linux
rs-enumerate-devices
```

Only when `rs-enumerate-devices` succeeds should the ROS driver be tested.

### 10.9 USB speed observation

The D405 was observed on the host as:

```text
480M
Usb Type Descriptor : 2.1
```

This means it was negotiating as USB 2.x rather than SuperSpeed USB 3.x. The camera still enumerated successfully on the host, so this was **not the root cause of the Docker failure**. However, a USB 3.x port/cable is preferable for higher camera bandwidth and more reliable stream profiles.

### 10.10 Validate librealsense before ROS

Once the SDK sees the device inside Docker, launch:

```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_sync:=true \
  pointcloud.enable:=true
```

Then verify:

```bash
ros2 topic list | grep camera
```

The important Stage 5 topics are:

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
```

### 10.11 Troubleshooting rule

Use this order whenever the RealSense camera stops working:

```text
1. Physical connection
        ↓
2. Host lsusb
        ↓
3. Host rs-enumerate-devices
        ↓
4. Docker lsusb
        ↓
5. Docker /dev/video* and /sys/class/video4linux
        ↓
6. Docker rs-enumerate-devices
        ↓
7. realsense2_camera
        ↓
8. ROS camera topics
        ↓
9. camera_object_localizer.py
```

This debugging order separates hardware problems from Docker problems and prevents wasting time modifying higher-level ROS or perception code when the underlying camera SDK cannot see the device.

### 10.12 Troubleshooting `/object_point_camera`

If `ros2 topic echo /object_point_camera` or `ros2 topic hz /object_point_camera` shows no data, debug the ROS graph from the node outward instead of immediately changing the perception code.

Use this flow:

```text
Is node running?
    ↓
ros2 node list
    ↓
Does node advertise publisher?
    ↓
ros2 node info /camera_object_localizer
    ↓
Does topic exist?
    ↓
ros2 topic list
    ↓
Does data arrive?
    ↓
ros2 topic echo /object_point_camera
```

Recommended commands:

```bash
ros2 node list
ros2 node info /camera_object_localizer
ros2 topic list | grep object
ros2 topic echo /object_point_camera
ros2 topic hz /object_point_camera
```

Interpret the result in this order:

- If `/camera_object_localizer` is missing from `ros2 node list`, the node is not running or it exited with an error.
- If the node is present but `/object_point_camera` is not listed under **Publishers** in `ros2 node info`, inspect the publisher creation and topic name in `camera_object_localizer.py`.
- If the topic exists but `echo` receives no messages, the node may be withholding output because no valid object/depth detection is available.
- Check the localizer terminal for HSV segmentation failure, minimum-contour filtering, invalid depth, wrong input topic names, or synchronization failure.
- ROS 2 topic names must not contain hyphens. Use `/object_point_camera`, not `/object_point-camera`.

This node/topic flowchart should be used only after the D405 driver and required camera topics are already confirmed working.

## 11. Relevant Information — Pixels and 3D Perception

### What is a pixel?

A **pixel** is the smallest addressable element of a digital image. In computer vision, pixels are the raw measurements from which algorithms infer color, edges, shapes, objects and object locations.

Each pixel has an image coordinate:

```text
(u, v)
```

where:

- `u` is the horizontal image coordinate (column), increasing to the right.
- `v` is the vertical image coordinate (row), increasing downward.
- `(0, 0)` is normally the top-left corner of the image.

For example, an object detected at

```text
(u, v) = (430, 310)
```

means that the object's selected image point, such as its center, lies at column 430 and row 310. These values are measured in **pixels**, not metres.

### What information does a pixel contain?

For a color image, a pixel normally contains RGB intensity values:

```text
P(u, v) = [R, G, B]
```

Computer-vision algorithms process groups of pixels to detect useful visual structure:

```text
Pixels
   ↓
Color / intensity / edges / texture
   ↓
Object region
   ↓
Object detection
   ↓
Object center (u, v)
```

The pixel coordinate tells us **where the object appears in the image**, but by itself it does not tell us the physical 3D position of the object.

### Why depth is required

For the RealSense D405, the aligned depth image provides a depth measurement corresponding to the color-image pixel.

For example:

```text
Object center: (u, v) = (430, 310)
Depth:         Z = 0.400 m
```

The combination `(u, v, Z)` provides enough information, together with the camera intrinsics, to deproject the image point into a 3D point in the camera optical frame.

Using

```text
Xc = (u - cx) * Z / fx
Yc = (v - cy) * Z / fy
Zc = Z
```

we obtain

```text
P_camera = [Xc, Yc, Zc]
```

in metres.

For example, a measurement may produce approximately:

```text
Xc = 0.0699 m
Yc = 0.0557 m
Zc = 0.4000 m
```

### ROS camera optical-frame convention

For the standard ROS optical frame:

```text
+x → image right
+y → image down
+z → forward from camera
```

Therefore:

- positive `Xc` means the point is to the right of the optical center,
- positive `Yc` means the point is below the optical center,
- positive `Zc` means the point is in front of the camera.

### From pixels to robot manipulation

The FR3 cannot directly use a command such as "pick the object at pixel `(430, 310)`". Robot motion requires a physical position expressed in a robot coordinate frame such as `fr3_link0`.

The complete perception-to-manipulation chain is therefore:

```text
Physical object
      ↓
RealSense color image
      ↓
Pixels
      ↓
Object detection
      ↓
Center pixel (u, v)
      ↓
Aligned depth Z
      ↓
Camera intrinsics (fx, fy, cx, cy)
      ↓
Camera-frame 3D point (Xc, Yc, Zc) [m]
      ↓
Camera-to-robot calibration / TF
      ↓
Robot-base point (Xb, Yb, Zb) [m]
      ↓
Generate PRE_GRASP / GRASP pose
      ↓
MoveIt motion planning
      ↓
FR3 manipulation
```

The key idea is:

> **A pixel provides a 2D location in the image. Depth and camera calibration convert that 2D observation into a physical 3D point. A validated camera-to-robot transform then converts that point into coordinates the FR3 can use for manipulation.**

This is the conceptual bridge between **computer vision** and **robot manipulation** in this project.

---

[Development Process Index](README.md) · [Previous Stage](stage_04_fixed_pick_place.md) · [Next Stage](stage_06_calibration_tf2.md)