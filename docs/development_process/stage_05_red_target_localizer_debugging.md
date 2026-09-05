# Stage 5 Debugging Note — Why the New Red-Target Localizer Works

## Purpose

This note records the debugging result from the RealSense D405 perception experiment and compares the original `camera_object_localizer.py` with the revised version.

The key result is that ROS 2, the RealSense streams, the camera intrinsics, depth deprojection, and the `/object_point_camera` publisher were fundamentally working. The original failure came from the **visual detection assumptions**: the code searched for a blue region with a relatively large minimum contour area, while the physical experiment used a **small red label on top of a cardboard object**.

The revised code matches the real target and therefore reaches the depth and 3D publishing stages.

## Confirmed Successful Result

After the revised detector was used, the node reported values similar to:

```text
Red target detected: pixel=(388, 289), area=390.0 px
Depth=1.0720 m
PUBLISHED /object_point_camera: X=-0.0942 m, Y=0.1216 m, Z=1.0720 m
```

`ros2 topic echo /object_point_camera` also returned a valid `PointStamped` message:

```yaml
header:
  frame_id: camera_color_optical_frame
point:
  x: -0.0942466
  y: 0.1215922
  z: 1.072
```

This proves that the following pipeline is now executing successfully:

```text
Color image
    ↓
Detect red target
    ↓
Center pixel (u, v)
    ↓
Aligned depth at the target
    ↓
Camera intrinsics
    ↓
Pixel + depth deprojection
    ↓
Camera-frame XYZ
    ↓
Publish /object_point_camera
```

## Old Code vs New Code

| Part | Original code | Revised code | Why the change matters |
|---|---|---|---|
| Physical target | Small red label on cardboard | Small red label on cardboard | The software must match the real experiment. |
| Target assumption | Function/documentation described a blue cube | Explicitly treats the target as a red marker/target | Avoids pretending the simple color detector performs true cube recognition. |
| HSV detection | One blue HSV interval | Two red HSV intervals | Red wraps around the ends of OpenCV's HSV hue range, so two intervals are normally required. |
| HSV range | Approximately `H=90..130` | Approximately `H=0..10` and `H=170..179` | The old range could not select the red label. |
| Minimum contour area | `500 px` | `50 px` initial threshold | The observed red marker contour was only about `390 px`, so the old `500 px` threshold would reject it even if its color were detected. |
| Morphology kernel | `5x5` | `3x3` | A smaller kernel is less aggressive for a small target and is less likely to erase the marker. |
| Detection diagnostics | Limited indication of why detection failed | Logs pixel location and contour area | Makes it obvious whether color segmentation is succeeding. |
| Depth diagnostics | Depth rejection could happen silently from the user's point of view | Prints depth and explicit invalid-depth warnings | Separates object-detection problems from depth problems. |
| Publishing diagnostics | XYZ log only after successful publish | Explicit `PUBLISHED /object_point_camera` message | Makes it immediately clear that the callback reached the final publishing stage. |
| Annotated failure view | Raw image published if no detection | Adds `NO RED TARGET` annotation | Makes camera-versus-detector debugging easier in `rqt_image_view`. |

## Root Cause 1 — The Original Detector Was Looking for Blue

The original detector used approximately:

```python
self.lower_hsv = np.array([90, 80, 50], dtype=np.uint8)
self.upper_hsv = np.array([130, 255, 255], dtype=np.uint8)
```

This is a blue-color range in OpenCV HSV space.

However, the physical test target was a red square/label attached to the top of the cardboard object.

Therefore the old pipeline behaved like this:

```text
Camera works
    ↓
RGB frame arrives
    ↓
Search for BLUE pixels
    ↓
Physical target is RED
    ↓
No valid target contour
    ↓
Detection returns None
    ↓
Callback exits before depth/deprojection
    ↓
No /object_point_camera message
```

The problem was not the intrinsic matrix. The intrinsic parameters correctly describe the camera and should remain nearly constant while an object moves.

## Root Cause 2 — The Original Minimum Contour Area Was Too Large

The original code used:

```python
self.minimum_contour_area = 500.0
```

The successful revised experiment measured approximately:

```text
area = 385–395 px
```

For example:

```text
Red target detected: pixel=(388, 289), area=390.0px
```

Therefore, even if the old code had used the correct red HSV range, the following condition would still have rejected the target:

```python
if contour_area < self.minimum_contour_area:
    return None
```

because:

```text
390 px < 500 px
```

The revised initial threshold is:

```python
self.minimum_contour_area = 50.0
```

This allows the current marker to pass while the HSV mask and largest-contour selection still reject much of the background noise.

The threshold should later be tuned upward once the marker size, camera mounting position, and working distance are fixed.

## Root Cause 3 — Red Requires Two HSV Ranges

OpenCV represents hue using approximately `0..179` rather than `0..360` degrees. Red lies close to the wrap-around boundary, so robust red segmentation commonly uses two masks:

```python
self.lower_red_1 = np.array([0, 100, 70], dtype=np.uint8)
self.upper_red_1 = np.array([10, 255, 255], dtype=np.uint8)

self.lower_red_2 = np.array([170, 100, 70], dtype=np.uint8)
self.upper_red_2 = np.array([179, 255, 255], dtype=np.uint8)
```

Then:

```python
mask_1 = cv2.inRange(hsv_image, self.lower_red_1, self.upper_red_1)
mask_2 = cv2.inRange(hsv_image, self.lower_red_2, self.upper_red_2)
mask = cv2.bitwise_or(mask_1, mask_2)
```

Using both sides of the hue boundary makes the detector more tolerant of illumination and camera color variation.

## Root Cause 4 — The Object Was Not Actually Being Recognized as a Cube

The original function name `detect_cube()` could be misleading.

The algorithm did not perform geometric cube recognition or object classification. Its actual logic was:

```text
BGR image
    ↓
HSV conversion
    ↓
Color threshold
    ↓
Morphological filtering
    ↓
Find contours
    ↓
Select largest acceptable colored contour
    ↓
Use contour centroid as (u, v)
```

Therefore a small colored marker is a valid target for this Stage 5 experiment.

The purpose of Stage 5 is to validate:

```text
2D detection
    +
aligned depth
    +
camera intrinsics
    ↓
3D camera-frame localization
```

True semantic object recognition can be added later using YOLO, segmentation, AprilTags, or another perception method.

## Why `/object_point_camera` Was Previously Empty

The ROS graph showed that `/object_point_camera` existed and had one publisher. That proved the publisher object had been created.

However, creating a publisher does **not** mean the code is publishing a message every frame.

The callback contains conditional stages. In simplified form:

```python
detection = detect_target(image)

if detection is None:
    return

depth = get_filtered_depth(...)

if depth is None:
    return

xyz = deproject_pixel(...)
publish_object_point(xyz)
```

The old detector failed before the final publish call, so:

```text
ros2 topic info /object_point_camera
```

could correctly show:

```text
Publisher count: 1
```

while:

```bash
ros2 topic echo /object_point_camera
```

showed no messages.

This distinction is important when debugging ROS 2:

> **Publisher exists** means the node advertises the topic.  
> **Messages arrive** means the callback logic actually reached `publish()`.

## Why the New XYZ Values Are Reasonable

The experiment used camera intrinsics approximately:

```text
fx = 426.5189
fy = 426.0117
cx = 425.4981
cy = 240.6794
```

A successful frame reported approximately:

```text
u = 388
v = 289
Z = 1.072 m
```

Using:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

produces approximately:

```text
X ≈ -0.094 m
Y ≈  0.122 m
Z =   1.072 m
```

which matches the published result.

The negative `X` is expected because:

```text
u = 388 < cx = 425.5
```

so the target lies to the left of the camera optical center.

The positive `Y` is expected because:

```text
v = 289 > cy = 240.7
```

so the target lies below the optical center in the ROS camera optical-frame convention.

This agreement is an important validation that the pixel-to-3D calculation is operating consistently.

## What Was Already Working in the Old Code

The debugging result also shows that several parts of the original implementation were already correct and did not need to be redesigned:

- Subscription to `/camera/camera/color/image_raw`.
- Subscription to `/camera/camera/aligned_depth_to_color/image_raw`.
- Subscription to `/camera/camera/color/camera_info`.
- Approximate synchronization of RGB and aligned depth.
- Reading `fx`, `fy`, `cx`, and `cy` from `CameraInfo`.
- Median filtering of a small depth window.
- Conversion of `16UC1` millimetres to metres.
- Pixel/depth deprojection into camera-frame XYZ.
- Publishing `geometry_msgs/msg/PointStamped`.
- Using `camera_color_optical_frame` as the frame of the published point.

The main lesson is that **a failure at the final ROS topic can originate much earlier in the perception pipeline**.

## Practical Debugging Method Learned

For this type of perception node, debug from upstream to downstream:

```text
1. Is RGB publishing?
        ↓
2. Is aligned depth publishing?
        ↓
3. Are camera intrinsics received?
        ↓
4. Is the intended target actually segmented?
        ↓
5. Is contour area above the threshold?
        ↓
6. Is valid depth available at the detected pixel?
        ↓
7. Is XYZ calculated?
        ↓
8. Is publish() reached?
        ↓
9. Does /object_point_camera contain messages?
```

Useful commands:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic info /object_point_camera -v
ros2 topic echo /object_point_camera
```

For the annotated image viewer, use:

```bash
ros2 run rqt_image_view rqt_image_view
```

If the package/executable is missing inside the container:

```bash
apt update
apt install -y ros-jazzy-rqt-image-view
```

Then source ROS again:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash
```

## Current Stage 5 Status

The current experiment has now demonstrated:

```text
RealSense RGB                  ✅
Aligned depth topic            ✅
CameraInfo / intrinsics        ✅
Red-marker detection           ✅
Center pixel (u, v)            ✅
Depth at target                ✅
Pixel-to-camera XYZ            ✅
/object_point_camera publish   ✅
```

The next major task is **not** to change the camera intrinsics. The next robotics step is to establish and validate the transform:

```text
camera_color_optical_frame
            ↓
      calibration / TF
            ↓
        fr3_link0
```

so that:

```text
P_base = T_base_camera · P_camera
```

can be used to generate a dynamic `PRE_GRASP`/`GRASP` position only after the transform has been validated safely.

---

[Stage 5 — RealSense D405 Perception](stage_05_realsense_perception.md) · [Development Process Index](README.md)
