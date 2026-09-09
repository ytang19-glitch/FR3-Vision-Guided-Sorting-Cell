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

## Immediate next steps — from partial ArUco detection to board pose

The current ChArUco debugging state has improved from:

```text
Detected: 0/17 markers
```

to values such as:

```text
Detected: 7/17 markers
Detected: 8/17 markers
Detected: 9/17 markers
```

This is significant because it confirms that the D405 image stream, ROS 2 image
transport, `cv_bridge`, OpenCV 4.6, and 4x4 ArUco decoding are all functioning.

Do **not** move on to TF2 publication or autonomous FR3 motion yet. The next
milestone is:

```text
stable ArUco detection
        ↓
ChArUco corner detection
        ↓
board 6D pose in the D405 camera frame
```

### A. Improve current marker detection

During detector testing, move the board closer to the D405 so the individual
markers contain more pixels. Keep the whole pattern visible and avoid extreme
tilt, motion blur, glare, and strong overexposure.

A useful practical starting point is to let the ChArUco board occupy roughly
30–60% of the image width. Exact `17/17` detection is not required on every
frame, but detection should be stable enough across useful board poses for the
corner and pose estimator to work reliably.

### B. Confirm the physical board dimensions before metric pose estimation

The source PDF is labeled approximately as:

```text
5 × 7 squares
Checker size: 22 mm
Marker size: 16 mm
Dictionary: ArUco DICT_4X4
```

The physical print has been measured at approximately:

```text
square side ≈ 20 mm
marker side ≈ 15 mm
```

Re-measure carefully before computing metric poses. A good method is to measure
several adjacent squares together and divide by the number of squares, which
reduces ruler error.

The physical dimensions do **not** determine whether `detectMarkers()` can
decode the ArUco pattern. They **do** determine the metric scale of the later
board pose and therefore directly affect the camera-to-FR3 calibration.

### C. Upgrade from ArUco markers to ChArUco corners

For a board with 5 × 7 squares, the number of inner chessboard corners is:

```text
(5 - 1) × (7 - 1) = 24
```

So the detector should evolve from reporting only:

```text
ArUco markers: N/17
```

to also reporting something like:

```text
ArUco markers: 12/17
ChArUco corners: 18/24
```

These ChArUco corners are the more useful observations for calibration because
they can be associated with known metric coordinates on the board.

### D. Estimate the board pose in the camera frame

Once enough ChArUco corners are detected, `calibration_board_detector.py`
should estimate the board's 6D pose relative to the D405 color optical frame.
A useful debug output is:

```text
Markers detected: 14/17
ChArUco corners: 20/24

Board pose in camera frame:
X = ... m
Y = ... m
Z = ... m
Roll  = ...
Pitch = ...
Yaw   = ...
Reprojection error = ... px
```

This transform is the next major Stage 6 milestone:

```text
T_camera_board
```

or equivalently the board pose expressed in the camera frame.

### E. Only after board pose is reliable, involve the FR3

Once board pose detection is stable, rigidly attach the ChArUco board to the
FR3 end effector and collect synchronized camera/robot pose pairs:

```text
Sample 1:
T_camera_board
T_base_tool

Sample 2:
T_camera_board
T_base_tool

...
```

Start with approximately 15–25 useful pose pairs, using meaningfully different
positions and orientations. Do not collect many nearly identical poses.

The final objective is to solve for:

```text
T_base_camera
```

so that object points can be transformed as:

```text
P_base = T_base_camera · P_camera
```

and the vision-guided manipulation chain becomes:

```text
D405
 ↓
object detection
 ↓
P_camera = (Xc, Yc, Zc)
 ↓
T_base_camera
 ↓
P_base = (Xb, Yb, Zb)
 ↓
PRE_GRASP
 ↓
FR3
```

### F. OpenCV 4.6 compatibility reminder

The installed environment reports:

```text
OpenCV version: 4.6.0
```

The newer API:

```python
cv2.aruco.ArucoDetector(dictionary)
```

is not available in the current environment. Use the OpenCV 4.6-compatible
interface:

```python
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()

corners, ids, rejected = cv2.aruco.detectMarkers(
    frame,
    dictionary,
    parameters=parameters,
)
```

### G. Current stop/go rule

Do not proceed to `T_base_camera`, TF2 publication, or robot pickup motion until:

1. ArUco detection is stable across useful board poses.
2. ChArUco corners are being detected consistently.
3. The physical board dimensions are confirmed.
4. The D405 color intrinsics and distortion coefficients match the image stream.
5. `T_camera_board` is stable and passes reprojection checks.

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
Editing square and marker side:
```bash
https://calib.io/pages/camera-calibration-pattern-generator?srsltid=AfmBOoqsVWZyesKWxj5m6iKlV0opKS-32rxhFPMR3SgO4wHWIJIDAhTE
```
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

2. Startup and callbacks
```bash
flowchart TD
    A["main: initialize ROS 2"] --> B["Construct node: __init__"]
    B --> C["spin: dispatch callbacks"]
    C -->|CameraInfo arrives| D["on_camera_info: validate and store"]
    C -->|Color image arrives| E["on_image: process frame"]
    D -.->|Latest camera model| E
    D --> C
    E --> C
    C -->|Shutdown or interruption| F["Destroy node and shut down ROS 2"]
```

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

$$
\mathbf p_b = {}^bR_c\mathbf p_c + {}^bt_c
$$

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

Move the target by hand while the camera remains fixed...
