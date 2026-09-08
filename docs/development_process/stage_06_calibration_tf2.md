# Stage 6 — Camera-to-FR3 Calibration and TF2

## Objective and current confirmed setup

The Intel RealSense D405 is mounted on a fixed overhead support, separate from
the FR3. This is an **eye-to-hand** configuration.

Stage 5 has now been experimentally validated far enough to provide a live
camera-frame 3D target point. The current perception node successfully detects
the small **red marker/target** used in the experiment, reads aligned depth,
deprojects the detected pixel into camera-frame XYZ, and publishes:

```text
/object_point_camera
```

A representative successful result was:

```text
Red target detected: pixel=(388, 289), area≈390 px
Depth≈1.072 m
PUBLISHED /object_point_camera:
X≈-0.094 m
Y≈ 0.122 m
Z≈ 1.072 m
```

The message frame is:

```text
camera_color_optical_frame
```

Therefore the current pipeline is:

```text
RealSense RGB
    ↓
red-target detection
    ↓
center pixel (u, v)
    ↓
aligned depth Z
    ↓
camera intrinsics
    ↓
P_camera = [Xc, Yc, Zc]
    ↓
/object_point_camera
```

Stage 6 must build the bridge from this camera-frame point to the FR3 base
frame:

```text
P_camera
    ↓
T_base_camera
    ↓
P_base
    ↓
/object_point_base
```

Only after this transform is calibrated and validated should the detected point
be used to generate robot pickup poses.

Calibration **estimates** the fixed camera pose relative to the robot. TF2 then
**stores and applies** that transform. TF2 does not discover the calibration by
itself.

---


## Start here — eight-step practical walkthrough

This walkthrough explains what to do in the lab and what must work before each
next step. The detailed sections below contain the transform conventions and
implementation notes.

**Method used here:** the D405 remains fixed overhead, and the calibration
board is rigidly attached to the robot end effector. The board moves with the
robot. Calibration has not been completed merely because the red-target
detector works.

A board placed on the table uses a different procedure: its full pose relative
to `fr3_link0` must be independently established. Do not combine table-mounted
board measurements with the moving-board hand-eye procedure below.

### Step 1 — Verify cube detection and depth stability

Keep the camera driver running. In separate terminals, source the environment:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash
```

Run the localizer and leave it running:

```bash
ros2 run fr3_vision_sorting camera_object_localizer
```

Open the viewer in another terminal:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/camera_object_localizer/annotated_image`. In another terminal, inspect:

```bash
ros2 topic echo /object_point_camera
```

With the robot stationary:

1. Move the cube by hand and confirm the outline follows it.
2. Remove or cover the separate red label if the detector selects it.
3. Leave the cube still for about ten seconds and record XYZ variation.
4. Remove the cube and confirm new valid cube points stop appearing.

The viewer and `topic echo` do not perform detection. Stopping the localizer
stops new annotated images and target points; a displayed old image or point
does not prove detection is still live.

**Checkpoint:** the intended cube is selected consistently, depth is valid,
and stationary variation is small enough for the intended grasp clearance.
Also verify absolute depth against a physical reference; stability alone does
not establish accuracy. Depth Z is the optical-axis coordinate, not generally
the straight-line distance from the camera origin.

Your raw color calibration has nonzero distortion coefficients. Before
precision localization, use distortion-aware deprojection with the matching
camera model. Aligning depth to color does not rectify the color image.

### Step 2 — Prepare a rigid, measured board
The main purpose of a ChArUco board in your project is to help your D405 camera and FR3 robot agree on where things are in the real world.

Print a checkerboard or ChArUco board at actual size, attach it to flat rigid
backing, and measure the printed pattern.

| Record | Meaning |
|---|---|
| Board type | Checkerboard or ChArUco |
| Pattern dimensions | Square counts and/or inner-corner counts required by the detector |
| Square size | Measured side length of a printed square, in metres |
| ChArUco settings | Marker size and dictionary, when applicable |
| Board origin and axes | The reference used for all board coordinates |

For example, **if measured** square size is 20 mm:

```python
square_size_m = 0.020
```

This is an example, not a measurement of your board. For a checkerboard,
square counts and inner-corner counts are different: a pattern with 8 by 6
squares has 7 by 5 inner corners.

First check that the camera resolves the required corners clearly. Then arrange
a rigid end-effector attachment with suitable clearance. The board must not
slip relative to the selected tool frame during collection.

**Checkpoint:** printed dimensions are measured, the board is flat, its pose
can be observed clearly, and its attachment is rigid.

In package.xml:

| Dependency       | Used for                              |
| ---------------- | ------------------------------------- |
| `std_msgs`       | Publishing reprojection error         |
| `python3-numpy`  | Matrix calculations                   |
| `python3-opencv` | ChArUco detection and pose estimation |
| `python3-scipy`  | Rotation-to-quaternion conversion     |



### Step 3 — Detect the board's position and orientation

Implement the separate `calibration_board_detector.py` described below.
The existing red-target localizer is not a board-pose detector.

The board detector must find ordered corners, associate them with known metric
board coordinates, and estimate `T_c_t` using the matching color `K` and
distortion coefficients `D`. Draw the detected corners and board axes,
preserve the image timestamp, and report reprojection error.

Here, `T_c_t` means **board coordinates expressed in the camera frame**.
Unlike a single centroid, a board pose contains both position and orientation.

**Checkpoint:** axes stay attached to the same board origin while stationary,
and bad or ambiguous detections are rejected. See the
[OpenCV calibration documentation](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)
for corner detection and pose estimation.

### Step 4 — Record paired camera and robot poses

Check the chosen tool frame:

```bash
ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand_tcp
```

Use the actual validated tool frame if your configuration differs.
This terminal display is a diagnostic, not a synchronized sample recorder.

For every sample, save:

| Measurement | Meaning |
|---|---|
| `T_c_t` | Board pose observed by the camera |
| `T_b_g` | Tool pose relative to `fr3_link0` at the image timestamp |
| Quality and timestamp | Evidence that the pair is usable |

Use your established supervised robot controls to move to a collision-checked
pose, stop, wait for settling, and trigger a sample save. The recorder should
look up robot TF at the image timestamp.

Start with approximately 15–25 useful pose pairs, including different
positions and tilts about multiple axes. Keep the board visible. Reserve
additional poses for validation. This sample count is a starting point, not a
guarantee; nearly identical orientations give little calibration information.

**Checkpoint:** each sample contains a synchronized pair, and the dataset has
meaningful rotational diversity.

### Step 5 — Calculate and validate the camera-to-base transform

Run the offline solver described below using the measured pose pairs and the
fixed-camera frame substitutions.

The result converts a camera point into a base point:

$
\mathbf p_b = {}^bR_c\mathbf p_c + {}^bt_c
$

| Variable | Practical meaning |
|---|---|
| `p_c = [Xc, Yc, Zc]` | Detected point relative to the camera optical frame, in metres |
| `R_b_c` | Rotation expressing the camera axes in the base frame |
| `t_b_c` | Camera optical origin's position relative to the base, in metres |
| `p_b = [Xb, Yb, Zb]` | The same physical point relative to `fr3_link0`, in metres |

Check physical plausibility and the reserved validation poses. The estimated
board-to-tool mounting transform should remain consistent because its
attachment is rigid. Save the result and validation errors.

**Checkpoint:** the measured transform passes validation. Do not substitute
guessed translations or quaternions.

### Step 6 — Connect the result to the existing TF tree

Inspect the RealSense TF tree, identify its actual root frame, and calculate
the base-to-root transform from the calibrated base-to-optical transform as
described in Section 13. Publish that fixed relationship while preserving the
driver's internal transforms.

Do not give the optical frame a second parent.

Check:

```bash
ros2 run tf2_ros tf2_echo fr3_link0 camera_color_optical_frame
```

**Checkpoint:** TF resolves the relationship, its composed transform matches
the calibration, and it remains fixed as the robot moves. Moving the camera
or its support invalidates this calibration.

### Step 7 — Publish and verify /object_point_base

Implement `object_tf_transformer.py` using the outline in Section 14:

- Subscribe to `/object_point_camera`.
- Transform the actual coordinates into `fr3_link0` at the measurement timestamp.
- Reject stale/nonfinite points and unavailable transforms.
- Publish `/object_point_base`.

These are implementation tasks; this guide does not establish that the new
node already exists or is installed.

Once implemented and running:

```bash
ros2 topic echo /object_point_base
```

The output frame should be `fr3_link0`. Set the RViz fixed frame to
`fr3_link0` and display the point. Compare several locations against
independent physical references, using the same physical surface point for
both measurements.

**Checkpoint:** measured position errors and stationary variation satisfy the
task's grasp clearance across the intended pickup area. A plausible RViz
picture alone is insufficient.

### Step 8 — Test PRE_GRASP, then one supervised cycle

Convert the detected surface point into a grasp TCP pose using object
dimensions, marker location, finger geometry, the chosen TCP, and the verified
grasp orientation.

1. Generate PRE_GRASP above the intended grasp TCP pose.
2. Inspect the collision-checked plan in RViz.
3. Execute only the supervised approach and stop above the cube.
4. Verify alignment and clearance at several pickup locations.
5. Add a planned descent, close the gripper, and verify the object is held.
6. Lift and use saved PRE_BIN, BIN, and POST_BIN poses for placement.

A pair of endpoints does not guarantee a straight vertical trajectory.
Validate the actual approach and retreat paths.

Dynamic pickup requires pose-based planning; new detected XYZ values do not
modify saved joint poses automatically. Keep the labeled destination fixed
for this first experiment.

**Checkpoint:** complete one supervised cycle with a fresh validated
detection, correct alignment, confirmed grasp, and successful release.

### What to do in the next lab session

Complete Steps 1 and 2 first. Record the stationary target variation and your
board type, pattern dimensions, and measured square size. Those measurements
are needed to configure and test the board detector in Step 3.

---

## 1. Why Stage 6 is required for vision-guided pick-and-place

The camera currently reports the target in its own optical frame:

```text
camera_color_optical_frame
```

For example:

```text
Xc = -0.094 m
Yc =  0.122 m
Zc =  1.072 m
```

These values tell us where the target is relative to the camera, not where it
is relative to the FR3 base.

The robot plans motion using frames such as:

```text
fr3_link0
```

Therefore the FR3 cannot directly use `/object_point_camera` as a Cartesian
pickup position.

The required relation is:

$$
\mathbf p_b = {}^bR_c\mathbf p_c + {}^bt_c
$$

or, in homogeneous form,

$$
\begin{bmatrix}
\mathbf p_b\\
1
\end{bmatrix}
=
{}^bT_c
\begin{bmatrix}
\mathbf p_c\\
1
\end{bmatrix}
$$

where:

- `b` = `fr3_link0`
- `c` = `camera_color_optical_frame`
- `T_b_c` = fixed transform from camera coordinates into robot-base coordinates

This is the transform Stage 6 must determine.

---

## 2. What the current Stage 5 outputs are useful for

The current detector prints several quantities:

```text
pixel=(u, v)
contour area
depth
Xc, Yc, Zc
```

These quantities have different roles.

### 2.1 Pixel `(u, v)`

This is the target's 2D image position.

```text
u → horizontal image coordinate
v → vertical image coordinate
```

When the target moves:

| Target movement in image | Expected change |
|---|---|
| Move right | `u` increases and `Xc` increases |
| Move left | `u` decreases and `Xc` decreases |
| Move down | `v` increases and `Yc` increases |
| Move up | `v` decreases and `Yc` decreases |

These are useful perception-validation checks.

### 2.2 Depth

The D405 aligned-depth image supplies the target distance from the camera.

```text
move closer to camera  → Zc decreases
move farther from camera → Zc increases
```

The revised Stage 5 node prints depth explicitly so that detection problems and
depth problems can be separated during debugging.

### 2.3 Camera-frame XYZ

The final Stage 5 result is:

```text
P_camera = [Xc, Yc, Zc]
```

This is the important quantity passed into Stage 6.

The chain is:

```text
(u, v) + aligned depth + camera intrinsics
                ↓
          camera-frame XYZ
                ↓
         /object_point_camera
```

The next chain is:

```text
/object_point_camera
        ↓
validated T_base_camera
        ↓
/object_point_base
```

---

## 3. Re-verify Stage 5 before calibration

Before collecting calibration data, keep the FR3 stationary and verify the
perception pipeline.

Source ROS 2 and the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash
```

Start the RealSense D405:

```bash
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_sync:=true \
  pointcloud.enable:=true
```

Run the revised localizer:

```bash
ros2 run fr3_vision_sorting camera_object_localizer
```

Inspect the output:

```bash
ros2 topic echo /object_point_camera
```

Confirm the publisher:

```bash
ros2 topic info /object_point_camera -v
```

Expected state:

```text
Type: geometry_msgs/msg/PointStamped
Publisher count: 1
```

The important distinction learned in Stage 5 is:

> `Publisher count: 1` means the publisher exists. It does **not** prove that
> the callback reaches `publish()` on every frame.

The revised detector now logs when the target is found, when depth is valid and
when `/object_point_camera` is actually published.

If `rqt_image_view` is available, inspect:

```text
/camera_object_localizer/annotated_image
```

Run it with:

```bash
ros2 run rqt_image_view rqt_image_view
```

If it is missing inside the container:

```bash
apt update
apt install -y ros-jazzy-rqt-image-view
```

Then source ROS again.

### Stage 5 validation before continuing

Move the target by hand while the camera remains fixed and verify:

| Motion | Expected camera-frame behavior |
|---|---|
| Right | `u ↑`, `Xc ↑` |
| Left | `u ↓`, `Xc ↓` |
| Down | `v ↑`, `Yc ↑` |
| Up | `v ↓`, `Yc ↓` |
| Closer to camera | `Zc ↓` |
| Farther from camera | `Zc ↑` |

Also confirm that:

- the detector follows the intended red target only;
- the target is not confused with another red object;
- the detected contour is stable;
- depth remains stable on a stationary target;
- the measured point corresponds to the intended physical surface/marker.

Do not command robot motion from the camera-frame point yet.

---

## 4. Understand the eye-to-hand transform

Use notation `T_A_B` for a transform that maps coordinates expressed in frame
`B` into frame `A`.

For the fixed overhead D405:

| Symbol | Meaning |
|---|---|
| `b` | Robot base frame `fr3_link0` |
| `c` | Camera optical frame `camera_color_optical_frame` |
| `g` | Chosen robot tool frame, e.g. `fr3_hand_tcp` |
| `t` | Calibration-target frame |
| `T_b_c` | Unknown fixed camera pose in robot base |
| `T_b_g` | Robot tool pose in base frame |
| `T_c_t` | Calibration-target pose measured by camera |
| `T_g_t` | Fixed target mounting transform relative to tool |

For every paired measurement:

$$
{}^bT_g\,{}^gT_t
=
{}^bT_c\,{}^cT_t
$$

The camera is fixed, so `T_b_c` must remain constant while the FR3 moves.

---

## 5. Do not use the red marker alone for hand-eye calibration

The red marker is useful for Stage 5 target localization, but it is not enough
for the calibration workflow described here.

The red detector gives mainly a target centroid and depth:

```text
(u, v, Z)
```

which produces a 3D point.

Hand-eye calibration needs a **full target pose**:

```text
position + orientation
```

Therefore use a rigid calibration target such as:

- ChArUco board; or
- checkerboard with known geometry.

A ChArUco board is generally convenient because it provides uniquely
identifiable corners and robust pose estimation.

---

## 6. Prepare the calibration board correctly

Use a rigid, flat printed board.

Record exactly:

- board type;
- number of squares/markers;
- square size in metres;
- marker size if ChArUco is used;
- ArUco dictionary;
- printed physical dimensions;
- board coordinate-frame definition.

Print at **100% / actual size** with printer scaling disabled.

After printing, measure several squares with a ruler or caliper. Do not assume
the nominal PDF dimensions survived printer scaling.

Mount the board rigidly to the FR3 end effector or a rigid tool fixture.

The board must not move relative to the selected robot tool frame while samples
are collected.

---

## 7. Implement a calibration-target pose detector

`camera_object_localizer.py` is the Stage 5 red-target localizer. Do not turn it
into the hand-eye calibration detector.

Create a separate node, for example:

```text
calibration_board_detector.py
```

Its responsibility should be:

```text
RGB image
    ↓
detect calibration board
    ↓
ordered 2D corners
    ↓
known 3D board geometry
    ↓
PnP / board pose estimation
    ↓
T_camera_target
```

The detector should:

1. Subscribe to the appropriate color image and `CameraInfo`.
2. Detect ordered board corners/markers.
3. Use the matching camera model.
4. Estimate the board pose `T_c_t`.
5. Draw detected corners and coordinate axes.
6. Reject bad detections.
7. Report reprojection error.
8. Publish or record board translation and orientation.
9. Preserve the image timestamp.

### Important camera-model note

The Stage 5 pinhole equations are useful for simple target-point localization,
but calibration-board pose estimation must use the correct camera calibration
model.

If raw color pixels are used and distortion coefficients are nonzero, use the
matching distortion coefficients in the PnP/calibration calculation.

Depth alignment is not the same thing as image rectification.

---

## 8. Verify the FR3 tool frame

Before recording robot poses, confirm which frame is being used as the tool
frame.

For example:

```bash
ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand_tcp
```

If that frame is not available in the current configuration, inspect the TF
tree and select the actual validated end-effector/TCP frame.

Do not silently substitute another frame; document the chosen frame because it
changes the calibration equations.

---

## 9. Collect paired robot and camera poses

For each calibration sample:

1. Move the FR3 to a safe, collision-checked pose.
2. Keep the calibration board fully visible to the D405.
3. Wait until the arm is completely stationary.
4. Detect and record `T_c_t` from the camera.
5. Look up `T_b_g` from TF at the corresponding image timestamp.
6. Save both poses as one synchronized pair.
7. Save detection quality/reprojection error.
8. Repeat with substantially different robot orientations and positions.

A useful dataset format is:

```yaml
sample_id: 01
stamp: ...

T_b_g:
  translation: [x, y, z]
  quaternion_xyzw: [qx, qy, qz, qw]

T_c_t:
  translation: [x, y, z]
  quaternion_xyzw: [qx, qy, qz, qw]

reprojection_error_px: ...
```

Use metres throughout.

Use ROS quaternion ordering explicitly:

```text
[x, y, z, w]
```

### How many samples?

A practical first dataset is approximately:

```text
15–25 good pose pairs
```

plus several held-out poses for validation.

More samples are not automatically better if all poses are nearly identical.
The robot should provide meaningful rotational diversity about multiple axes.

Avoid collecting only translations with almost identical orientation.

---

## 10. Create a paired-pose recorder

Create a separate node such as:

```text
calibration_pose_recorder.py
```

Its purpose is to synchronize:

```text
camera board pose T_c_t
        +
robot TF pose T_b_g
        ↓
paired calibration sample
```

The recorder should not move the robot automatically during the first
calibration experiment.

A safe initial workflow is:

```text
operator moves robot
        ↓
robot stops
        ↓
board detection valid
        ↓
operator triggers sample save
        ↓
record synchronized pair
```

This keeps motion and data collection logically separate.

---

## 11. Solve the eye-to-hand calibration offline

After collecting the pose pairs, solve for:

```text
T_b_c
```

An OpenCV approach can use `cv2.calibrateHandEye` with the frame substitutions
required for the fixed-camera eye-to-hand case.

Illustrative offline code:

```python
import cv2
import numpy as np

# Lists of corresponding 4x4 transforms:
# T_b_g_samples : base <- gripper/tool
# T_c_t_samples : camera <- calibration target

T_g_b_samples = [
    np.linalg.inv(T_b_g)
    for T_b_g in T_b_g_samples
]

R_b_c, t_b_c = cv2.calibrateHandEye(
    [T[:3, :3] for T in T_g_b_samples],
    [T[:3, 3:4] for T in T_g_b_samples],
    [T[:3, :3] for T in T_c_t_samples],
    [T[:3, 3:4] for T in T_c_t_samples],
    method=cv2.CALIB_HAND_EYE_PARK,
)

T_b_c = np.eye(4)
T_b_c[:3, :3] = R_b_c
T_b_c[:3, 3] = np.asarray(t_b_c).reshape(3)
```

Do not treat solver output as automatically correct.

Check:

```text
rotation finite                      ✅
translation finite                   ✅
RᵀR ≈ I                              ✅
det(R) ≈ +1                          ✅
camera location physically plausible ✅
held-out residuals acceptable        ✅
```

Also estimate the board mounting transform for each sample:

```python
mount_estimates = [
    np.linalg.inv(T_b_g) @ T_b_c @ T_c_t
    for T_b_g, T_c_t in zip(
        T_b_g_samples,
        T_c_t_samples,
    )
]
```

These should be consistent because the board mounting is rigid.

---

## 12. Save the calibration result explicitly

Do not leave the final transform only inside a terminal log.

Save a calibration file, for example:

```text
config/camera_to_fr3_calibration.yaml
```

Example structure:

```yaml
parent_frame: fr3_link0
camera_optical_frame: camera_color_optical_frame
camera_root_frame: <verify from TF>

translation_m:
  x: ...
  y: ...
  z: ...

quaternion_xyzw:
  x: ...
  y: ...
  z: ...
  w: ...

camera_profile:
  width: ...
  height: ...
  fps: ...

board:
  type: charuco
  square_size_m: ...
  marker_size_m: ...
  dictionary: ...

validation:
  number_of_samples: ...
  held_out_samples: ...
  mean_error_mm: ...
  max_error_mm: ...
```

Do not insert guessed transform values.

---

## 13. Connect the calibrated camera into TF2

Before publishing a transform, inspect the existing camera TF tree.

The RealSense driver typically publishes internal camera transforms such as:

```text
camera_link
    ↓
...
    ↓
camera_color_optical_frame
```

Do not give `camera_color_optical_frame` a second TF parent.

Instead, identify the actual RealSense root frame and connect that root to the
FR3 tree.

Let:

- `r` = RealSense camera root frame
- `c` = `camera_color_optical_frame`

If calibration directly produced `T_b_c`, obtain the existing internal camera
transform `T_r_c` from TF and compute:

$$
{}^bT_r
=
{}^bT_c({}^rT_c)^{-1}
$$

Then publish the fixed transform:

```text
fr3_link0
    ↓
camera root
    ↓
RealSense internal TF
    ↓
camera_color_optical_frame
```

Use either:

- `StaticTransformBroadcaster`; or
- `static_transform_publisher` with the measured values.

Never publish a zero/identity placeholder and treat it as calibration.

After publishing, verify:

```bash
ros2 run tf2_ros tf2_echo \
  fr3_link0 \
  camera_color_optical_frame
```

The composed transform should match the calibrated `T_b_c` and remain fixed
while the FR3 moves.

---

## 14. Create `/object_point_base`

Create a separate node:

```text
object_tf_transformer.py
```

Input:

```text
/object_point_camera
```

Output:

```text
/object_point_base
```

Conceptually:

```text
PointStamped in camera frame
        ↓
TF2 lookup / transform
        ↓
PointStamped in fr3_link0
```

Core operation:

```python
import tf2_geometry_msgs
from rclpy.duration import Duration
from tf2_ros import TransformException

try:
    point_base = self.tf_buffer.transform(
        point_camera,
        "fr3_link0",
        timeout=Duration(seconds=0.2),
    )
except TransformException as error:
    self.get_logger().warning(
        f"Transform failed: {error}",
        throttle_duration_sec=1.0,
    )
    return

self.point_base_publisher.publish(point_base)
```

Important:

```text
changing header.frame_id only ❌
actual coordinate transform   ✅
```

The numbers themselves must be transformed.

Reject:

- NaN/Inf points;
- stale target measurements;
- unavailable TF;
- points outside the intended workspace.

---

## 15. Why this enables dynamic vision-guided pickup

Before vision, the project used a fixed pickup location:

```text
saved PRE_GRASP
    ↓
saved GRASP
```

If the object moved, the robot still approached the original fixed position.

After Stage 6:

```text
red target moves
    ↓
new (u, v)
    ↓
new depth
    ↓
new P_camera
    ↓
T_base_camera
    ↓
new P_base
    ↓
new PRE_GRASP
    ↓
new GRASP
```

For example, suppose Stage 6 eventually produces a validated target point:

```text
/object_point_base

x = 0.48 m
y = 0.12 m
z = 0.05 m
```

A simple first pre-grasp could be defined conceptually as:

```text
PRE_GRASP:
    x = object_x
    y = object_y
    z = object_z + safe_vertical_offset
```

For example:

```text
object z = 0.05 m
safe offset = 0.10 m

PRE_GRASP z = 0.15 m
```

Do not use these example numbers without measuring the actual workspace and tool
geometry.

---

## 16. Validate `/object_point_base` without robot motion

Before connecting vision to MoveIt, keep the robot stationary.

Run:

```bash
ros2 topic echo /object_point_base
```

In RViz:

```text
Fixed Frame = fr3_link0
```

Display the transformed target point.

Test multiple known target positions across the intended pickup area.

Record:

| Test position | Reference base XYZ (m) | Estimated base XYZ (m) | Error (mm) | Stationary variation (mm) |
|---|---|---|---|---|
| Center | Record | Record | Calculate | Measure |
| Left | Record | Record | Calculate | Measure |
| Right | Record | Record | Calculate | Measure |
| Near | Record | Record | Calculate | Measure |
| Far | Record | Record | Calculate | Measure |

Use physically measured references where possible.

Do not accept calibration only because the RViz visualization “looks close”.
Measure the error.

The acceptable error depends on:

- target size;
- gripper finger width;
- grasp clearance;
- tool geometry;
- approach direction;
- required task tolerance.

There is no universal error threshold.

---

## 17. Separate surface localization from grasp TCP position

The current red-target localizer estimates the 3D position of the detected red
surface/marker.

That point is not necessarily the gripper TCP grasp point.

For example:

```text
D405 detects top surface
        ↓
P_surface
```

but manipulation needs:

```text
P_TCP_grasp
```

Therefore define a grasp offset based on:

- physical object dimensions;
- marker location on the object;
- gripper geometry;
- selected TCP;
- grasp approach direction.

Conceptually:

```text
P_grasp = P_detected + grasp_offset
```

Do not simply command the gripper TCP to the detected top-surface Z value.

---

## 18. First motion test: dynamic PRE_GRASP only

After `/object_point_base` is validated, do **not** immediately run a complete
autonomous grasp.

The first robot-motion test should be:

```text
fresh target detection
        ↓
/object_point_camera
        ↓
TF2
        ↓
/object_point_base
        ↓
generate PRE_GRASP only
        ↓
MoveIt plan
        ↓
inspect plan
        ↓
supervised execution
        ↓
stop above object
```

Verify that the gripper arrives above the physical target with safe clearance.

Repeat at several object positions before implementing descent.

---

## 19. Then add dynamic GRASP and reuse fixed placement

For the first complete vision-guided pick-and-place experiment, make only the
pickup position dynamic.

Keep the placement/bin poses fixed.

Recommended sequence:

```text
DETECT
    ↓
CAMERA XYZ
    ↓
BASE XYZ
    ↓
DYNAMIC PRE_GRASP
    ↓
DYNAMIC GRASP
    ↓
CLOSE
    ↓
LIFT
    ↓
FIXED PRE_BIN
    ↓
FIXED BIN
    ↓
RELEASE
    ↓
POST_BIN
    ↓
HOME
```

This is much easier to debug than making pickup and placement dynamic at the
same time.

---

## 20. Full vision-guided pick-and-place logic

Once Stage 6 is validated, the high-level architecture becomes:

```text
Physical target
      ↓
RealSense RGB
      ↓
red-target detection
      ↓
(u, v)
      ↓
aligned depth Z
      ↓
camera intrinsics
      ↓
P_camera = (Xc, Yc, Zc)
      ↓
/object_point_camera
      ↓
validated camera-to-FR3 TF
      ↓
P_base = (Xb, Yb, Zb)
      ↓
/object_point_base
      ↓
grasp offset / safety checks
      ↓
PRE_GRASP pose
      ↓
MoveIt planning
      ↓
FR3 approaches object
      ↓
GRASP
      ↓
LIFT
      ↓
fixed BIN
      ↓
RELEASE
```

This is the transition from **fixed pick-and-place** to **vision-guided
pick-and-place**.

---

## 21. Safety and data-validity rules

Before any point is converted into a robot command, verify:

1. Detection is current, not stale.
2. Target contour is valid.
3. Depth is valid and finite.
4. `/object_point_camera` is finite.
5. TF lookup succeeds.
6. `/object_point_base` is inside a predefined workspace.
7. Target height is physically plausible.
8. PRE_GRASP has safe table clearance.
9. The planned trajectory is collision-free.
10. The operator can stop execution immediately.

If any condition fails, withhold motion.

Never reuse the last valid object point indefinitely after the target disappears.

---

## 22. Current Stage 6 implementation checklist

### Already complete

```text
RealSense RGB stream                         ✅
Aligned depth                               ✅
CameraInfo / intrinsics                     ✅
Red-target detection                        ✅
Target center pixel (u, v)                  ✅
Depth at target                             ✅
Camera-frame XYZ                            ✅
/object_point_camera                        ✅
```

### To implement now

```text
Rigid measured ChArUco/checkerboard         ⬜
Calibration-board pose detector             ⬜
T_camera_target output                      ⬜
Robot/tool pose recording                   ⬜
Synchronized pose-pair recorder             ⬜
15–25 varied calibration samples            ⬜
Offline eye-to-hand solver                  ⬜
Held-out calibration validation             ⬜
Saved T_base_camera                         ⬜
Static TF connection                        ⬜
object_tf_transformer.py                    ⬜
/object_point_base                          ⬜
RViz + physical-position validation         ⬜
Dynamic PRE_GRASP test                      ⬜
```

---

## Immediate next lab task

Do **not** move on to automatic grasping yet.

The immediate sequence should be:

```text
1. Prepare and accurately measure a rigid ChArUco/checkerboard.
2. Keep the D405 fixed.
3. Build calibration_board_detector.py.
4. Verify stable board pose T_camera_target.
5. Confirm the FR3 tool/TCP frame.
6. Build calibration_pose_recorder.py.
7. Collect varied synchronized robot-camera pose pairs.
8. Solve T_base_camera offline.
9. Publish the validated transform into TF2.
10. Build object_tf_transformer.py.
11. Verify /object_point_base at multiple known positions.
12. Test dynamic PRE_GRASP only.
13. Only then attempt one supervised vision-guided pick-and-place cycle.
```

The key milestone for Stage 6 is:

> **The same physical target point is measured by the D405, transformed through
> a validated camera-to-FR3 calibration, and appears at the correct measured
> location in `fr3_link0`.**

Once that is repeatable, the project is ready to use vision to update the pickup
location.

---

[Development Process Index](README.md) · [Previous Stage](stage_05_realsense_perception.md) · [Stage 5 Debugging Note](stage_05_red_target_localizer_debugging.md) · [Next Stage](stage_07_vision_guided_sorting.md)
