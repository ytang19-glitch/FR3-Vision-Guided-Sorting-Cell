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


## Practical debugging and code map (2026-09-18)

### Repository code versus the local experiment

The repository currently contains the fixed-grasp and fixed-pick-and-place
implementations linked below. The working robot-PC experiment also uses
`vision_pick_place_demo.py`, `camera_object_localizer.py` and
`gripper_control.py`; these files are not present in the repository snapshot
reviewed for this update. The vision function names below describe the local
one-shot version discussed during debugging, not an implementation installed
automatically by cloning this repository.

A separate `object_tf_transformer.py` and `/object_point_base` publisher are
optional architecture choices. The local vision demo transforms the point
internally; do not wait for `/object_point_base` unless a node actually publishes it.

### Which file and function should I read?

Python module paths below are relative to
`ros2_ws/src/fr3_vision_sorting/fr3_vision_sorting/`.

| File | Function / class | Purpose |
|---|---|---|
| `camera_object_localizer.py` (local) | Image/depth processing callbacks; inspect the installed implementation for their names | Detect the cube, obtain valid depth, and publish `PointStamped` on `/object_point_camera`. It does not command the arm. |
| `vision_pick_place_demo.py` (local) | `object_callback()` | Reject unusable measurements, accumulate the detection window and freeze the accepted target. Ignore new detections after locking. |
| Same file | `transform_object_point()` | Look up base-from-camera TF and transform the measured point into `fr3_link0`. |
| Same file | `current_tcp_orientation()` | Read the TCP orientation relative to the base. Keeping the current orientation does not automatically choose a suitable grasp orientation. |
| Same file | `move_to_cartesian_pose()` | In the reviewed local version, build a MoveIt **pose goal** for the TCP. Its name does not guarantee a straight Cartesian path. |
| Same file | `execute_vision_sequence()` | Generate pickup targets, handle preview/START confirmation, run pickup, then the saved placement sequence. |
| Same file | `main()` | Own node initialization, acquisition and execution scheduling. The corrected one-shot design executes the sequence outside the acquisition callback. |
| [`fixed_grasp_demo.py`](../../ros2_ws/src/fr3_vision_sorting/fr3_vision_sorting/fixed_grasp_demo.py) | `FixedGraspDemo.__init__()` | Create MoveIt/grasp action clients and load HOME, PRE_GRASP, GRASP and LIFT YAML files. |
| Same file | `load_joint_state()` | Read YAML, map names to positions and return the seven arm joints in the required order. |
| Same file | `move_to(pose_name)` | Plan and execute a saved **joint target**, such as BIN. |
| Same file | `close_on_object()` | Send the Franka Grasp action and check its reported success. |
| `gripper_control.py` (local dependency) | `GripperController.open_gripper()` | Request gripper opening; inspect that implementation for its action and executor handling. |
| [`fixed_pick_place_demo.py`](../../ros2_ws/src/fr3_vision_sorting/fr3_vision_sorting/fixed_pick_place_demo.py) | `add_bin_poses()`, `run_motion()`, `main()` | Load placement poses and run the fixed sequence with per-step confirmation. |
| [`automatic_pick_place_demo.py`](../../ros2_ws/src/fr3_vision_sorting/fr3_vision_sorting/automatic_pick_place_demo.py) | `add_bin_poses()`, `run_automatic_cycle()` | Run a fixed-position cycle after one confirmation. “Automatic” here does **not** mean camera-guided. |

### Why did we collect PRE_GRASP and GRASP YAML?

The saved files contain joint configurations, not camera measurements and not
calibration transforms. They established a repeatable fixed-position baseline:
first verify arm motion, finger alignment, gripping and placement at known
locations; then replace the pickup location with vision.

| Saved file | Fixed-position experiment | Local vision-guided experiment |
|---|---|---|
| `config/fixed_grasp/home_joint_state.yaml` | Recorded HOME configuration | Reusable return configuration when the route is checked |
| `config/fixed_grasp/pre_grasp_joint_state.yaml` | Arm configuration above the original cube | Baseline/reference; dynamic PRE_GRASP is generated above the newly detected cube |
| `config/fixed_grasp/grasp_joint_state.yaml` | Arm configuration with fingers aligned to the original cube | Baseline/reference; dynamic GRASP replaces this pickup target |
| `config/fixed_grasp/lift_joint_state.yaml` | Lift configuration above the original pickup | Baseline/reference; dynamic LIFT is generated from the new grasp target |
| `config/fixed_pick_place/pre_bin_joint_state.yaml` | Approach configuration near the destination | Still used by `move_to("PRE_BIN")` |
| `config/fixed_pick_place/bin_joint_state.yaml` | Release configuration | Still used by `move_to("BIN")` |
| `config/fixed_pick_place/post_bin_joint_state.yaml` | Retreat configuration | Still used by `move_to("POST_BIN")` |

**Loaded is not the same as executed.** A vision subclass calling
`FixedGraspDemo.__init__()` still loads all four fixed-grasp files. Deleting
PRE_GRASP/GRASP/LIFT YAML can therefore prevent startup even if the vision
sequence never calls those saved pickup targets.

The current base loader reads the last dictionary document in the YAML,
matches joint names and uses only `fr3_joint1` through `fr3_joint7`.
Finger positions, recorded velocities, efforts and timestamps do not become
arm motion commands. Joint angles are in radians. The planner generates a
new trajectory; this is not replay of the original recorded movement.

To tell which target is actually executed, inspect the call:

```python
self.move_to("GRASP")  # Uses the original saved joint configuration.

self.move_to_cartesian_pose(
    x, y, grasp_z, orientation, "GRASP"
)  # Uses a generated TCP pose; "GRASP" is the stage label.
```

Do not change old pickup YAML expecting the camera target to move. Conversely,
moving the cube does not update saved joint configurations.

### Meaning of PRE_GRASP, GRASP and LIFT

- **PRE_GRASP:** a checked approach pose above the pickup, providing clearance
  before the final descent.
- **GRASP:** the TCP pose that places the fingers at the intended contact
  height. Reaching it does not close the fingers; that is a separate action.
- **LIFT:** a clearance pose after successful closure, before transport.

In the local version used during this experiment, the formulas are:

```python
pre_grasp_z = object_z + self.pre_grasp_offset
grasp_z = object_z + self.grasp_offset
lift_z = grasp_z + self.lift_offset
```

Thus the pre-grasp-to-grasp separation is
`pre_grasp_offset - grasp_offset`, not simply `pre_grasp_offset`.
For the previously logged offsets 0.120, 0.025 and 0.150 m, that separation
is 0.095 m. These are experiment values, not universal grasp settings.

The camera point is typically on a visible surface. Select offsets using
the actual TCP, finger geometry and object dimensions. A point supplies no
orientation; use a separately verified orientation policy.

### Debug from perception to execution

Use the last successful log to locate the failing stage rather than changing
several parameters at once.

| Symptom / last log | What it tells you | Next check |
|---|---|---|
| `NO RED TARGET` | No usable color target in the current view | Check occlusion, lighting, selected object and detector output. |
| `INVALID DEPTH` | Color detection did not produce a usable depth sample | Check aligned depth, matching intrinsics, units and valid pixels in the target region. Never replace missing depth with an arbitrary distance. |
| Detection repeatedly resets to 1/10 | The local acceptance window is restarting | Log the reset reason, freshness and XYZ variation. Depth variation may exceed the stability threshold; inspect it before changing the threshold. |
| `Target frozen` | Acquisition is complete | Do not move the cube. New camera detections should not overwrite the accepted target. |
| `two or more unconnected trees` | This listener cannot resolve base-to-camera TF | Follow the [Stage 06 CV Issues recovery](stage_06_CV_ISSUES.md); check both bridge and complete chain. |
| No attribute `tf_buffer` | Python initialization problem | Initialize the buffer and keep its listener alive before using it. |
| `PointStamped ... not loaded or supported` | Missing Python TF conversion registration | Import `tf2_geometry_msgs` when using registered transforms, or use explicit `do_transform_point(point, transform)` after lookup. |
| `PREVIEW ONLY` | Coordinate preview intentionally sends no motion commands | Review targets. Preview is not a motion-plan/collision check. |
| START accepted, then detection restarts | Sequence returned or reset early | Check indentation, unconditional `return`, exceptions and the `finally` reset logic. |
| `No keyboard input` / immediate cancellation | Confirmation input is unavailable or wrong | Use an interactive `ros2 run` terminal for this version and type START + Enter. |
| `Executor is already spinning` | Conflicting/nested executor use | Inspect `main()` and action waits; avoid spinning the same node from a worker while another executor already spins it. |
| Waiting for `/move_action` or gripper server | Required action server is unavailable to this node | Check the existing bringup, action list and environment. Avoid launching duplicate bringups. |
| MoveIt goal rejected / execution failed | Acquisition succeeded; motion did not | Read the MoveIt result, controller status, start state, reachability and collision scene. Do not advance to the next stage. |
| Pickup works, placement orientation is awkward | Placement is a separate set of targets | Inspect LIFT → PRE_BIN and the saved BIN configurations, not only camera calibration. |

Camera occlusion **after** target locking does not itself require continuous
redetection in this one-shot design. Continuing with the frozen target assumes
the cube has not moved. Occlusion before acquisition prevents a valid target.

### Read-only checks and preview

Keep the existing robot/camera drivers and calibrated static publisher running.
In the diagnostic/vision terminal use the environment that restored this setup:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

Run these checks individually:

```bash
ros2 topic echo /object_point_camera --once
ros2 run tf2_ros tf2_echo fr3_link0 camera_color_optical_frame
ros2 action list -t
ros2 control list_controllers
ros2 pkg executables fr3_vision_sorting
```

Stop the continuous TF query before entering the next command in that terminal.
A discovered action name alone does not confirm a functioning controller.

If the local vision executable is installed:

```bash
ros2 run fr3_vision_sorting vision_pick_place_demo \
  --ros-args -p preview_only:=true
```
Practical Tips:
```bash
Measure tcp_to_fingertip accurately; do not guess it.
Keep at least 20 mm fingertip clearance from the table.
Check that grasp_offset places the TCP above the calculated minimum height.
Run with preview_only:=true before real execution.
Use stop_after_pre_grasp:=true for the first physical test.
Set geometry_confirmed:=true only after checking the gripper, table, TCP, and workspace.
Keep the emergency stop accessible during every experiment.
```
Run:
```bash
ros2 run fr3_vision_sorting vision_pick_place_demo --ros-args   -p tcp_to_fingertip:=0.056   -p fingertip_clearance:=0.020   -p grasp_offset:=0.080   -p preview_only:=false   -p geometry_confirmed:=true   -p stop_after_pre_grasp:=false
```

Check the transformed point, all generated target heights and the TCP
quaternion. The one-shot version ends after preview; restart to reacquire.
Setting `preview_only:=false` and confirming START requests the **full**
pick-and-place sequence, not only PRE_GRASP.

### Editing code or saved poses

Edit source files under `/workspace/ros2_ws/src/fr3_vision_sorting/`, not the
generated `build/` or `install/` copies. After changing code, entry points
or installed configuration:

```bash
cd /workspace/ros2_ws
colcon build --packages-select fr3_vision_sorting --symlink-install
source install/setup.bash
```

Check which files the terminal actually imports:

```bash
python3 -c 'import fr3_vision_sorting.vision_pick_place_demo as m; print(m.__file__)'
ros2 pkg prefix --share fr3_vision_sorting
```

The base loader uses the installed package share directory for YAML. Ensure
`setup.py` installs the relevant configuration files. A missing file may
therefore be an installation issue rather than a motion-planning failure.
The inherited logger name `fixed_grasp_demo` alone does not prove an old
vision executable is running.

### Why a saved BIN pose can look strange

A saved joint target fixes the whole arm configuration, including wrist and
elbow posture. It does not automatically preserve the pickup TCP orientation.
Read `move_to("PRE_BIN")`, `move_to("BIN")` and `move_to("POST_BIN")` to
identify which saved configurations control placement.

Compare the TCP poses of LIFT, PRE_BIN, BIN and POST_BIN using the actual FR3
model/forward kinematics, and inspect the planned transitions. If the
destination or desired orientation changed, reteach and validate the affected
poses. Do not adjust one wrist joint blindly.

Also, a prompt saying “descend vertically” does not enforce vertical motion.
The committed `move_to()` uses joint goals; the local
`move_to_cartesian_pose()` uses pose goals. Neither by itself guarantees a
straight line or fixed orientation along the entire path. For controlled
descent/retreat, implement and verify an appropriate Cartesian path, reject
incomplete paths, and check collision clearance.


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
