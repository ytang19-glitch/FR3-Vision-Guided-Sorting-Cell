# FR3 Vision-Guided Pick-and-Place: Setup and Debugging

This package connects a detected object position to MoveIt motion and Franka gripper actions. This guide explains how to diagnose grasp-position errors, keep the tool vertical, and configure offsets without confusing a successful motion command with a successful physical grasp.

**TF provides coordinate transforms and pose feedback. MoveIt plans and executes motion.** A correct TCP pose does not necessarily mean the fingers are correctly aligned with the object: calibration, tool geometry, orientation, and depth must also be correct.

## 1. Read the relevant code

| File or function | Purpose |
| --- | --- |
| [vision_pick_place_demo.py](fr3_vision_sorting/vision_pick_place_demo.py) | Acquires one target, transforms it, computes approach/grasp/lift poses, and runs the sequence. |
| `object_callback()` / `try_start_sequence()` | Check detection validity and freeze the target after the required nearby measurements. |
| `transform_object_point()` | Transform the camera point into `fr3_link0` at its measurement timestamp. |
| `_read_grasp_orientation()` | Read and normalize the configured grasp quaternion. |
| `check_geometry()` | Check table dimensions, target bounds, and the vertical-orientation assumption. |
| `execute_vision_sequence()` | Apply XY correction, calculate heights, handle preview/START, and execute the sequence. |
| `move_to_cartesian_pose()` | Request a MoveIt pose goal for PRE_GRASP. |
| `check_cartesian_start()` | Wait for fresh TCP TF and compare actual pose with the nominal Cartesian start. |
| `move_vertical_cartesian()` | Request a complete collision-checked Cartesian descent or lift. |
| `validate_cartesian_trajectory()` | Check joint names, timing, finite values, and joint steps. |
| [fixed_grasp_demo.py](fr3_vision_sorting/fixed_grasp_demo.py) | Shared saved-pose motion and Franka grasp helpers. |
| [automatic_pick_place_demo.py](fr3_vision_sorting/automatic_pick_place_demo.py) | Provides `add_bin_poses()` for saved destinations. |
| [setup.py](setup.py) | Registers executables and installs available configuration/launch files. |

The vision program imports `gripper_control.py` and depends on saved pose YAML files. At the time of this README update, those files, the perception/calibration modules, and launch files are not all present in this repository snapshot. The commands below assume the complete lab workspace contains these dependencies; a successful package build alone does not prove every executable can run.

## 2. Position, orientation, and tool geometry

Use the same frames throughout:

- Base frame: `fr3_link0`.
- Controlled tool frame: `fr3_hand_tcp`.
- Detection frame: normally `camera_color_optical_frame`, taken from the message header.
- Position and length units: metres.
- Quaternion order: **x, y, z, w**.

A physical fingertip position follows:

```text
p_fingertip_in_base = p_TCP_in_base + R_base_TCP × r_TCP_to_fingertip
```

Therefore, changing orientation changes the fingertip position even when TCP XYZ remains unchanged. For an illustrative 0.11 m lever arm, a 3-degree tilt produces approximately 5.8 mm lateral displacement. This is a geometry example, not a measurement of this tool's TCP offset.

The observed `fr3_hand -> fr3_hand_tcp` translation of 0.103 m is a transform between two frames. It is **not** automatically the TCP-to-lowest-fingertip distance.

### Keep the grasp vertical

The committed defaults are:

```python
"grasp_orientation_qx": 0.0,
"grasp_orientation_qy": 1.0,
"grasp_orientation_qz": 0.0,
"grasp_orientation_qw": 0.0,
```

This quaternion represents a 180-degree rotation about Y. It points TCP +Z along base -Z. It corresponds to a downward approach only when the actual tool axis and horizontal table match those frame assumptions.

PRE_GRASP, GRASP, and LIFT use the same configured orientation. Changing yaw can preserve a downward tool axis while changing the finger closing direction. Keep that direction aligned with the intended opposite faces of the cube.

The PRE_GRASP orientation is a **goal constraint**, not a guarantee that the entire approach maintains a vertical tool. Descent and lift use Cartesian waypoints with matching orientations. Saved bin poses determine their own tool orientations.

## 3. Parameters that affect grasping

Values below describe the committed code or the recorded experiment; they are not universal calibration values.

| Parameter | Meaning | Current default / experiment |
| --- | --- | --- |
| `base_frame` | Reference for target positions and table geometry | `fr3_link0` |
| `tcp_frame` | Tool link controlled by MoveIt and checked through TF | `fr3_hand_tcp` |
| `grasp_orientation_qx/qy/qz/qw` | Desired tool orientation in the base frame | `[0, 1, 0, 0]` |
| `pre_grasp_offset` | PRE_GRASP TCP height above the detected point | 0.120 m |
| `grasp_offset` | GRASP TCP height above the detected point | Default 0.025 m; experiment override 0.080 m |
| `lift_offset` | Lift height above GRASP | 0.150 m |
| `table_top_z` | Table surface height in the base frame | 0.130 m; verify physically |
| `table_center_x/y` | Table rectangle centre in the base frame | 0.0 / 0.0 m; verify frame origin |
| `table_size_x/y` | Table extent along base X/Y | 1.22 / 0.99 m; verify axis alignment |
| `table_thickness` | Thickness of the planning-scene table box | 0.040 m |
| `tcp_to_fingertip` | Downward distance from TCP to lowest finger geometry at the chosen orientation | 0.056 m; approximate experiment value |
| `fingertip_clearance` | Required modelled gap above the table | 0.020 m |
| `stable_detections` | Required consecutive nearby detections | 10 |
| `stable_distance` | Maximum distance between successive accepted detections | 0.010 m |
| `max_detection_age` | Maximum detection age | 1.0 s |
| `cartesian_velocity_scale` / `cartesian_acceleration_scale` | Cartesian trajectory scaling factors | 0.03 / 0.03; dimensionless |
| `max_joint_step` | Maximum allowed adjacent joint-position step in trajectory validation | 0.10 rad |
| `preview_only` | Print coordinates without sending motion commands | True |
| `geometry_confirmed` | Operator acknowledgement that geometry and scene have been checked | False |
| `stop_after_pre_grasp` | Stop before gripper opening and descent | True |

The detector must publish the intended object. `target_color` is only a label in this node; a `PointStamped` message contains no colour classification to filter.

## 4. Understand offsets before changing them

### XY correction

The current code applies this correction **once, after transforming into the base frame**:

```python
x -= 0.012
y -= 0.021
```

These are hard-coded local experimental corrections, not declared ROS parameters. They came from comparing an initial TCP position near `[0.238, 0.409, 0.256]` with a manually aligned position near `[0.226, 0.388, 0.255]`.

Do not infer base-frame X/Y signs from “left” or “right” in a photograph. Do not apply the same correction again in the detector or launch configuration.

Test several cube locations. A roughly constant error may support a fixed correction; an error that changes with location suggests checking camera rotation, depth, intrinsics, or tool geometry. A correction validated at one point does not establish workspace-wide accuracy.

### Height and table clearance

The program calculates:

```text
PRE_GRASP z = detected base-frame z + pre_grasp_offset
GRASP z     = detected base-frame z + grasp_offset
LIFT z      = GRASP z + lift_offset

Minimum GRASP TCP z =
    table_top_z + tcp_to_fingertip + fingertip_clearance
```

For the recorded geometry:

```text
Minimum TCP z = 0.130 + 0.056 + 0.020 = 0.206 m
```

If detected z is 0.134 m and grasp_offset is 0.080 m, requested GRASP z is 0.214 m. The simplified model predicts a fingertip gap of `0.214 - 0.056 - 0.130 = 0.028 m`.

This passes the height check but does not establish that the pads will contact the cube correctly. Check cube height, pad contact area, actual tool dimensions, and detection accuracy. In particular, a detected cube surface almost level with the table deserves investigation.

With pre_grasp_offset = 0.120 and grasp_offset = 0.080, descent is only **0.040 m**, not 0.120 m.

The height formula assumes a horizontal table and the validated vertical tool. It does not replace collision checking for the full fingers, hand, arm, or carried object. The current program does not attach the grasped object to the planning scene.

## 5. Debug in this order

### Step 1 — Verify communication and TF

In each relevant new terminal, use the lab environment:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

These exports affect subsequently started processes, not existing nodes. They helped restore communication in the recorded setup, but an unconnected TF tree is not by itself proof of a UDP problem.

Check both connections:

```bash
ros2 run tf2_ros tf2_echo fr3_link0 camera_color_optical_frame
```

```bash
ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand_tcp
```

Restore the verified calibration publisher if the camera tree is disconnected. Avoid duplicate publishers for the same transform. Moving the cube does not require recalibration; changing the camera-to-base mounting does.

### Step 2 — Preview the target

For the recorded setup, after checking the measured geometry:

```bash
ros2 run fr3_vision_sorting vision_pick_place_demo --ros-args \
  -p tcp_to_fingertip:=0.056 \
  -p fingertip_clearance:=0.020 \
  -p grasp_offset:=0.080 \
  -p preview_only:=true \
  -p stop_after_pre_grasp:=true
```

Inspect the transformed point, corrected XY, target heights, orientation, and minimum height. Preview does not plan a trajectory or validate collisions.

The current detector logic freezes the **last accepted point**, not the average of ten measurements. Pairwise stability does not guarantee low absolute error. Once frozen, further detections are ignored; reacquire if the cube moves.

### Step 3 — Test PRE_GRASP only

After physically checking geometry, modelled obstacles, and the approach path, change the preview command to:

```text
-p preview_only:=false
-p geometry_confirmed:=true
-p stop_after_pre_grasp:=true
```

Run interactively with `ros2 run` and enter `START`. This mode stops **before opening the gripper or descending**.

Keep the validated orientation fixed while diagnosing XY error. Change one quantity at a time.

### Step 4 — Separate tracking error from localization error

Compare the current run's logged PRE_GRASP target with fresh TCP TF:

```text
dx = actual TCP x - commanded TCP x
dy = actual TCP y - commanded TCP y
dz = actual TCP z - commanded TCP z
position error = sqrt(dx² + dy² + dz²)
```

| Observation | Investigate next |
| --- | --- |
| TCP has not reached its commanded pose | Execution result, settling, fresh TF, controller tracking, and goal tolerances |
| TCP matches the command but fingers miss the cube | Camera calibration, depth, TCP definition, tool geometry, and XY correction |
| One finger is lower or touches the table first | Actual orientation, asymmetric tool geometry, and table tilt |
| Alignment changes significantly across the table | Calibration/depth errors before adding more local offsets |
| Correct endpoint but awkward arm posture or path | IK solution, joint limits, singularity proximity, start posture, and planned trajectory |

### Step 5 — Verify opening before descent

The recorded failure reported successful actions while width stayed near 0.03328 m. Therefore, `success=True` alone did not prove physical opening.

Inspect the actual gripper state:

```bash
ros2 action info /franka_gripper/move
ros2 action info /franka_gripper/grasp
ros2 topic echo /franka_gripper/joint_states --once
```

For the observed symmetric finger state, the sum of the two joint positions represented the reported driver width. Custom pads can change the physical contact gap. Verify the local gripper helper checks fresh final width before allowing descent.

Choose grasp width from the object and tool geometry. The current vision sequence requests `width=0.045`, `speed=0.02`, and `force=10.0`; these are experimental settings, not a universal cube grasp.

### Step 6 — Check descent, lift, and placement

The current Cartesian path request uses 5 mm waypoint spacing, collision checking, and requires a complete returned path. Do not execute a partial path merely to avoid a planning failure.

After approach, opening, and descent geometry are verified, `stop_after_pre_grasp:=false` enables the full sequence. Review PRE_BIN, BIN, POST_BIN, and HOME too: correct pickup orientation does not determine the orientation of those saved destinations.

## 6. Accuracy limits versus acceptance limits

The committed implementation currently uses:

| Check | Value | Meaning |
| --- | --- | --- |
| PRE_GRASP position goal region | 2 mm sphere radius | Planner goal acceptance region |
| PRE_GRASP orientation goal tolerances | 0.01 rad on each axis | Planner orientation acceptance |
| Cartesian start position check | 4 mm Euclidean error | Actual start must be close to nominal start |
| Cartesian start orientation check | 1 degree rotation error | Actual orientation must be close to the target |
| TCP TF freshness | Age between 0 and 0.5 s | Reject stale or future-dated TCP data |
| Fresh-TF wait | Up to 3 s | Process callbacks and wait for usable feedback |

These are separate checks. A planning tolerance is not a promise of actual tracking accuracy.

An earlier run failed at **3.23 mm and 0.49 degrees** when the position gate was 3 mm. The repository now uses 4 mm. This change permits a larger start error; it does not improve calibration or control accuracy.

The Cartesian request includes the nominal start waypoint. A small mismatch can produce a correction from the actual pose to that waypoint before descent. Evaluate that correction and its clearance instead of repeatedly increasing the threshold.

## 7. What the saved YAML poses do

A saved joint-state YAML contains joint angles, not an explicit TCP quaternion. Changing one wrist joint can change both position and orientation through the robot's kinematics.

- Fixed-grasp examples use taught pickup poses.
- The vision sequence calculates pickup XYZ from the detected point and uses the configured quaternion.
- Its inherited initialization still depends on the saved-pose configuration.
- PRE_BIN is the approach destination, BIN is the release destination, POST_BIN is the retreat, and HOME is the return pose.

To correct a strange placement orientation, review or re-teach those destination poses and inspect the complete planned path. Changing the vision grasp quaternion alone will not rewrite saved bin poses.

## 8. Common failures and the corresponding fix

| Symptom | Diagnosis or change |
| --- | --- |
| Detection repeats after 10/10 | Freeze once, ignore later callbacks, and avoid automatic restart. |
| START accepted but sequence immediately ends | Check that cancellation/EOF returns are indented inside their branches. |
| `Executor is already spinning` | Avoid nested/concurrent spinning; the current main loop starts motion outside callbacks. |
| PointStamped type not supported by TF | The current code uses explicit `do_transform_point()`. |
| `INVALID DEPTH` | Inspect aligned depth at the detected object; do not substitute an arbitrary depth. |
| Target outside tabletop rectangle | Check transformed coordinates and measured bounds; do not enlarge the table to hide an error. |
| Requested GRASP below modelled minimum | Recheck depth, TCP geometry, table height, and intended contact height. |
| Robot blocks the camera after target acquisition | A frozen target can be used only while the object remains stationary; it is not continuous visual servoing. |
| Stale TCP TF after gripper opening | Process queued TF and wait for fresh feedback, as implemented in V3. |
| Gripper reports success but does not move | Check measured width and physical motion; investigate the gripper separately. |
| Changes appear ineffective | Rebuild, source the intended workspace, and check which installed package is running. |

For Python changes in the complete lab workspace:

```bash
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select fr3_vision_sorting
source install/setup.bash
ros2 pkg prefix fr3_vision_sorting
```

Record each trial's raw and corrected target, commanded pose, actual TCP pose, orientation error, gripper width, and physical outcome. This makes it possible to distinguish calibration errors from motion and grasp failures.

## Further reading

- [Full experiment issue log](../../../docs/development_process/stage_07_Experiment.md)
- [Vision-guided sorting workflow](../../../docs/development_process/stage_07_vision_guided_sorting.md)
- [Execution parameters](../../../docs/development_process/stage_07_vision_pick_place_execution_parameters.md)
- [TF and communication issues](../../../docs/development_process/stage_06_CV_ISSUES.md)
- [Grasping lessons and troubleshooting](../../../docs/development_process/FR3_GRASPING_LESSONS_AND_TROUBLESHOOTING.md)
