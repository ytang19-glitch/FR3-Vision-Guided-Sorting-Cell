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

1. Detect the ChArUco board.
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
P_base = T_base_camera · P_camera
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

The robot should return to a safe state instead of blindly continuing after a failed perception or motion step.

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
ChArUco board pose
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
