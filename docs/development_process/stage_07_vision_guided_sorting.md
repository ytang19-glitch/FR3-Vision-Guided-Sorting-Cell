# Stage 7 — Vision-Guided Sorting

## Objective

Stage 7 begins **after Stage 6 camera-to-FR3 calibration has been solved and validated**.

Stage 6 establishes the geometric bridge between the D405 camera and the FR3 base frame:

```text
P_camera
    ↓
T_base_camera
    ↓
P_base
```

Stage 7 uses that calibrated transform to make the robot act on detected objects.

The Stage 7 pipeline is:

```text
D405 RGB + depth
        ↓
object detection / classification
        ↓
pixel (u, v) + depth
        ↓
P_camera
        ↓
T_base_camera        ← calibrated in Stage 6
        ↓
P_base
        ↓
grasp target generation
        ↓
PRE_GRASP
        ↓
MoveIt planning
        ↓
DESCEND
        ↓
GRASP
        ↓
LIFT
        ↓
MOVE TO CLASS-SPECIFIC BIN
        ↓
RELEASE
        ↓
HOME / next object
```

---

## Boundary between Stage 6 and Stage 7

### Stage 6 — Calibration and TF2

Stage 6 contains the calibration work:

1. Detect the measured calibration board (the current Stage 6 guide uses a plain checkerboard).
2. Estimate `camera_T_board`.
3. Rigidly attach the board to the FR3 end effector.
4. Record synchronized `camera_T_board` and `base_T_tool` samples.
5. Collect approximately 15–25 diverse calibration poses.
6. Solve the eye-to-hand calibration.
7. Obtain and validate `base_T_camera`.
8. Connect the calibrated camera transform into TF2.
9. Verify that camera-frame points transform correctly into `fr3_link0`.

The key Stage 6 output is:

```text
T_base_camera
```

Stage 6 is complete only after this transform has been validated.

### Stage 7 — Vision-guided manipulation and sorting

Stage 7 uses the Stage 6 result to locate, pick, classify, and sort real objects.

---

## Step 1 — Detect the object

Use the D405 perception node to obtain the object's camera-frame position:

```text
/object_point_camera
```

with:

```text
P_camera = [Xc, Yc, Zc]
```

The point should be expressed in the D405 camera optical frame.

---

## Step 2 — Transform the object into the FR3 base frame

Use the calibrated Stage 6 transform:

```math
\mathbf p_b = {}^{b}R_c\mathbf p_c + {}^{b}\mathbf t_c
```

Publish the transformed target, for example:

```text
/object_point_base
```

with frame:

```text
fr3_link0
```

This is the important transition from **seeing the object** to giving the robot a target in its own coordinate system.

---

## Step 3 — Validate the transformed target

Before allowing robot motion:

- verify that the target XYZ is finite and fresh;
- reject unavailable or stale TF data;
- verify that the target lies inside the allowed workspace;
- visualize the target in RViz with `fr3_link0` as the fixed frame;
- compare several transformed points with independent physical measurements.

A visually plausible point in RViz is not sufficient by itself. The transformed location should be physically validated before grasp execution.

---

## Step 4 — Generate PRE_GRASP

Convert the detected object point into a safe grasp TCP pose.

```text
object position in fr3_link0
        ↓
object/gripper/TCP offset
        ↓
PRE_GRASP above object
```

The offset should account for object dimensions, finger geometry, the selected TCP, and desired grasp orientation.

Do not command the robot directly to the detected surface point.

---

## Step 5 — Execute one supervised vision-guided grasp

Start with a single object and a supervised cycle:

```text
HOME
 ↓
PRE_GRASP
 ↓
DESCEND
 ↓
CLOSE GRIPPER
 ↓
LIFT
```

Verify alignment and clearance at several pickup positions before attempting continuous autonomous operation.

---

## Step 6 — Add the sorting destination

After reliable vision-guided pickup:

```text
LIFT
 ↓
MOVE TO BIN
 ↓
OPEN GRIPPER
 ↓
HOME
```

For multiple object classes, map perception results to class-specific bin poses.

Example:

```text
red object   → bin A
blue object  → bin B
green object → bin C
```

---

## Step 7 — Add safety and failure handling

Before autonomous repetition, handle at least:

- no object detected;
- invalid depth;
- stale object point;
- unavailable TF;
- target outside the allowed workspace;
- motion-planning failure;
- failed grasp;
- object lost after grasp;
- sorting-bin motion failure.

Stop advancing the sequence after a failed step. Do not automatically return HOME
or open the gripper: the arm may be near an obstacle or holding an object.
Any recovery movement needs a separately checked plan and operator review.
An exception or node shutdown is not proof that an active robot action has stopped;
request cancellation and check the controller/action state.

---

## Step 8 — Evaluate the sorting system

Record quantitative results such as:

- localization error;
- grasp success rate;
- sorting success rate;
- motion-planning time;
- total cycle time.

These measurements turn the project from a demonstration into an engineering experiment that can be compared and improved.

---


## Practical first milestone — one cube, one fixed destination

The first Stage 7 experiment is **variable pickup, fixed placement**. Move the
cube to different verified locations between trials; the robot obtains a new
pickup target from vision each time. Reuse the existing saved placement poses
before adding multiple classes or bins.

This document describes the intended integration. It does not establish that
a Stage 7 motion executable or point-transformer node is already installed,
or that the current calibration has passed validation.

### 1. Complete the calibration gate

Follow [Stage 6](stage_06_calibration_tf2.md) first. For the fixed-camera,
moving-board method, the board must be rigidly attached to the tool and move
with it through varied positions and orientations. A board resting on the
table while only the arm moves does not provide the required pairs.

A stationary-board method is also possible when its full pose relative to
`fr3_link0` is independently established; it requires a different calculation.
Do not mix the two methods or use an old YAML result merely because its
numbers are finite.

Record the camera frame, base frame, calibration date, validation errors and
the exact calibration file used. Recalibrate if the camera or its support moves.
After moving-board calibration, remove the board before cube pickup and update
the planning scene and tool/load configuration as appropriate.

### 2. Run perception and inspect the outputs

Keep the existing robot and camera drivers running. In each new terminal,
source the environment used by the working setup:

```bash
cd /workspace/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Run the localizer in its own terminal:

```bash
ros2 run fr3_vision_sorting camera_object_localizer
```

In another terminal:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/camera_object_localizer/annotated_image`. Confirm the outline follows
the cube, not a separate red destination label. Inspect a measurement:

```bash
ros2 topic echo /object_point_camera --once
```

Check the timestamp and frame as well as XYZ. The point represents the selected
visible surface, not automatically the cube center or desired gripper TCP.

### 3. Transform and verify without motion

Once the validated calibration is connected to TF and the point-transformer
node has been implemented and started, inspect:

```bash
ros2 run tf2_ros tf2_echo fr3_link0 camera_color_optical_frame
```

In another terminal:

```bash
ros2 topic echo /object_point_base --once
```

Use the actual frame names if they differ. Preserve the measurement timestamp
and transform the coordinates; changing only `header.frame_id` is incorrect.
Preserve the RealSense driver's internal TF tree rather than assigning the
optical frame a second parent.

For `p_c = [Xc, Yc, Zc]` and `p_b = [Xb, Yb, Zb]`:

```math
\mathbf p_b = {}^bR_c\mathbf p_c + {}^b\mathbf t_c
```

Both points and the translation are in metres; the rotation is dimensionless.
A 4 × 4 homogeneous transform requires a fourth coordinate of 1.

With the robot stationary, test several independently measured pickup
locations. Record position error and stationary variation. Choose acceptance
limits based on cube size, finger clearance and required placement accuracy;
this guide does not assume a universal millimetre threshold.

### 4. Convert a surface point into a grasp pose

The robot needs a full TCP pose: position **and orientation**.
For a fixed-orientation grasp, define a verified offset in base coordinates:

```math
\mathbf p_{grasp} = \mathbf p_{surface} + \Delta\mathbf p_{TCP}
```

The offset accounts for the detected surface, cube dimensions, desired finger
contact height and the selected TCP. If an offset is defined in an object or
tool frame instead, rotate it into the base frame before adding it.

For a vertical approach, assuming base +Z points upward:

```math
\mathbf p_{pre} = \mathbf p_{grasp} + [0,0,h_{approach}]^T
```

```math
\mathbf p_{lift} = \mathbf p_{grasp} + [0,0,h_{lift}]^T
```

Choose the heights from measured workspace clearance. Keep the validated
orientation fixed during approach and lift. ROS quaternion order is
`[x, y, z, w]`; no particular quaternion is universally a downward grasp.

A center point alone does not estimate cube yaw. Initially constrain object
orientation, or add orientation estimation before handling arbitrary rotations.

### 5. Reuse existing code with a new pickup interface

| Existing part | Stage 7 use |
|---|---|
| `camera_object_localizer.py` | Supply live camera-frame detections |
| Proposed `object_tf_transformer.py` | Transform accepted points into the robot base frame |
| `FixedGraspDemo` | Reuse suitable gripper/action and saved-pose functionality after checking its implementation |
| `config/fixed_grasp/` | Keep HOME and the fixed-pickup baseline for comparison |
| `config/fixed_pick_place/` | Reuse validated PRE_BIN, BIN and POST_BIN |
| Proposed `vision_pick_place_demo.py` | Coordinate target validation, Cartesian pickup planning and saved placement |

Changing detected XYZ does not modify saved joint-state YAML poses.
Implement a pose-target planning interface for the dynamic pickup instead of
calling the old saved GRASP pose and expecting it to follow the cube.
Ensure the planner's end-effector link matches the TCP used to generate poses.

`PointStamped` has no class label, orientation or confidence field. Before
multi-class sorting, introduce a detection message or synchronized data
structure that carries these together with target identity and timestamp.

### 6. Test in stages

1. Generate and display PRE_GRASP and GRASP poses in RViz without executing.
2. Add the table, fixtures and relevant obstacles to the planning scene.
3. Inspect a plan to PRE_GRASP; execute only that approach under supervision.
4. Check alignment at several pickup positions before allowing descent.
5. Test descent and retreat paths with the verified orientation and clearance.
6. Complete one cycle with the sequence below.

| Step | Required check before continuing |
|---|---|
| Acquire target | Fresh, finite, stable detection; correct cube; valid TF; inside workspace |
| Freeze target for this attempt | Save target timestamp and calibration identity; do not silently replace it mid-motion |
| PRE_GRASP | Plan succeeds and execution finishes successfully |
| OPEN | Clearance is available and gripper action succeeds |
| DESCEND | Collision-checked path reaches the intended grasp pose |
| CLOSE | Width/force match the object; grasp result and physical hold are verified |
| LIFT | Object remains held; attached-object collision geometry is represented |
| PRE_BIN → BIN | Saved destination remains clear and reachable from the current state |
| RELEASE | Object is supported or at the verified release height; opening succeeds |
| POST_BIN → HOME | Retreat and return are planned and verified |

If the cube moves after target acquisition, abort the current attempt and
reacquire before descent. Do not use continuous updates as an implicit visual
servo controller. Straight-line endpoint placement alone does not guarantee
a straight-line executed path; verify the complete approach trajectory and
reject incomplete Cartesian paths.

### 7. Handle failures explicitly

| Failure | Response |
|---|---|
| No detection or invalid depth | Wait for a valid target; do not reuse an indefinitely old point |
| TF unavailable or calibration invalid | Block target execution and report the frame/calibration problem |
| Unreachable target or failed plan | Keep the sequence stopped; revise the target or plan |
| Grasp reports failure | Do not proceed to transport; inspect object and finger alignment |
| Object slips or disappears during lift | Stop advancing; inspect before planning recovery |
| Bin motion or release fails | Preserve the known grasp state and require reviewed recovery |

Set explicit action timeouts, inspect terminal action results, and implement
cancellation for an interrupted run. Do not equate destroying a ROS node with
stopping hardware. Gripper width tolerances are not collision tolerances and
should not be enlarged simply to make a failed grasp report success.

### 8. Measure the result before adding complexity

Run repeated trials across the validated pickup region using the same cube,
lighting and placement destination. Log failed attempts as well as successes.

| Measurement | Definition |
|---|---|
| Localization error | Euclidean distance between estimated and independently measured positions |
| Grasp success rate | Stable successful lifts / attempted grasps |
| Placement success rate | Objects released inside the specified destination / attempted cycles |
| Sorting success rate | Objects delivered to the correct class destination / attempted sorting cycles |
| Planning time | Time spent generating the motion plans, reported separately from execution |
| Cycle time | Define a consistent start/end event and report operator waiting separately |
| Failure category | Detection, depth, TF, planning, grasp, transport or release |

Keep calibration identity, object location/class, commanded poses, action
outcomes and timestamps in the trial record. Report typical and worst observed
errors, not just one successful demonstration.

After reliable single-cube pickup and fixed placement, add class-to-bin mapping,
then multiple-object selection, then reviewed recovery and repeated cycles.

---

## Stage 7 success condition

Stage 7 is successful when the system can repeatedly perform:

```text
See and classify object
        ↓
Estimate P_camera
        ↓
Transform to P_base
        ↓
Generate safe PRE_GRASP
        ↓
Plan and pick object
        ↓
Move to correct bin
        ↓
Release object
        ↓
Repeat
```

while the Stage 6 camera-to-base calibration remains fixed and validated.

---

## Overall project progression

```text
Stage 5
Object localization in camera frame
        ↓
Stage 6
Calibration board pose
camera_T_board
        ↓
Collect camera_T_board + base_T_tool
        ↓
Eye-to-hand calibration
        ↓
base_T_camera
        ↓
TF2 validation
        ↓
Stage 7
P_camera → P_base
        ↓
Vision-guided grasping
        ↓
Classification + sorting
        ↓
Failure handling
        ↓
Repeated autonomous cycles
```

> **Stage 6 teaches the camera and robot how their coordinate systems are related. Stage 7 uses that relationship to pick and sort objects.**

---

[Development Process Index](README.md) · [Previous Stage](stage_06_calibration_tf2.md)
