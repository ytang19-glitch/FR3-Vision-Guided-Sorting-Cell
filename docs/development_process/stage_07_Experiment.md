# Stage 07 — Vision-Guided Pick-and-Place Experiment and Issue Log

Experiment sessions: 17–18 September 2026. This record summarizes the terminal logs, supplied Python code, photographs, measurements, and operator reports from the experiment.

## 1. Experiment objective and current result

The original note was: “Failure after changing the pose of the cube.”

Moving the cube exposed limitations in localization, workspace limits, grasp geometry, and execution. Moving the cube alone does **not** break the camera-to-base TF or require a new camera calibration. Moving the camera relative to the robot does.

The objective is to detect the red cube, transform its camera coordinates into the robot base, approach with a vertical gripper, descend, grasp, lift, place at saved bin poses, and return home.

**Current evidence does not establish a fully validated physical pick-and-place cycle.**

- TF connectivity was restored, and camera points were transformed into the base.
- The robot reached PRE_GRASP; the operator confirmed the vertical orientation.
- A local XY correction was added, and the operator subsequently reported acceptable alignment.
- One sequence logged completion through HOME, but the operator reported that the gripper never moved. That run was not a successful physical grasp.
- Standalone gripper commands reproduced success without motion. The operator reported recovery after homing and later reported that the updated opening test worked.
- The fresh-TF revision passed the timestamp check on the next reported run.
- The latest run stopped at a Cartesian start error of **3.23 mm and 0.49 degrees**. Increasing the position threshold from 3 mm to 4 mm was discussed conditionally; no subsequent successful run was supplied.

Related documentation:

- [Stage 06 calibration and TF](stage_06_calibration_tf2.md)
- [Stage 06 communication and disconnected TF issues](stage_06_CV_ISSUES.md)
- [Stage 07 implementation and debugging guide](stage_07_vision_guided_sorting.md)
- [General grasping troubleshooting](FR3_GRASPING_LESSONS_AND_TROUBLESHOOTING.md)

## 2. Setup and software boundaries

| Item | Experimental setup |
| --- | --- |
| ROS environment | Ubuntu 24.04, ROS 2 Jazzy, Python 3.12 |
| Robot address | 172.16.0.2 |
| Robot base | fr3_link0 |
| Tool reference | fr3_hand_tcp |
| Camera measurement frame | camera_color_optical_frame |
| Detection topic | /object_point_camera, PointStamped |
| Arm planning | MoveIt /move_action |
| Straight descent/lift | /compute_cartesian_path and /execute_trajectory |
| Gripper opening | /franka_gripper/move |
| Object grasp | /franka_gripper/grasp |
| Application workspace | /workspace/ros2_ws |
| Driver overlay | /opt/franka_ros2_ws/install |

The uploaded local program versions were labelled ONE_SHOT_START_FIX, VERTICAL_CARTESIAN_V2, and VERTICAL_CARTESIAN_V3_TF_REFRESH. They are different revisions, not interchangeable descriptions of one implementation.

At the repository inspection for this document, the local vision_pick_place_demo.py and gripper_control.py revisions were not present in the committed Python directory. This document records the supplied local code and experiment; it does not claim to install or commit those Python revisions.

## 3. Complete issue register

“Observed” means supported by the supplied logs/code or an explicit operator report. A proposed change is not automatically a verified fix.

| ID | Problem or symptom | Evidence / interpretation | Action and status |
| --- | --- | --- | --- |
| 01 | Calibration input file missing | FileNotFoundError for calibration_samples_new.yaml | Use the actual saved input path. Later solve loaded 17 samples. |
| 02 | Repeated calibration poses and limited validation coverage | Initial samples 1–3 were nearly identical; a later validation set mainly changed orientation; another set changed translation | Collect only after robot/board settle and detection is stable, with varied poses. Fit residuals alone are not independent accuracy validation. |
| 03 | Candidate mistaken for validated calibration | Candidate YAML contained validated: false despite low residuals | Keep an independent validation stage before relying on the transform for grasping. |
| 04 | Disconnected robot and camera TF trees | “not part of the same tree” between fr3_link0 and camera frames | Restore the base-to-camera_link bridge and keep its publisher alive. Both bridge/full-chain queries later returned numbers. |
| 05 | Fast DDS shared-memory failures | RTPS_TRANSPORT_SHM open_and_lock_file errors | Matching middleware/UDP environment and restarted processes were followed by restored connectivity. UDP alone was not isolated as the cause. |
| 06 | Confusing camera_link and optical frame transforms | Internal camera rotation differed from calibrated optical-frame pose | Compose the camera internal transform; do not apply the optical calibration unchanged to camera_link. |
| 07 | Static publisher stopped or wrong terminal environment | A publisher had logged “Spinning” but another listener still saw disconnected trees | Keep publisher running; match sourced overlays, ROS environment and transport settings. Exports do not change existing processes. |
| 08 | /tf_static echo produced no messages | Four publishers were discovered, but echo showed only transport errors | Publisher counts do not prove data delivery or a correct TF chain. Check specific transforms. |
| 09 | MoveIt/RViz package unavailable | franka_fr3_moveit_config not found in sourced paths | Check installed package names and source the driver overlay. Exact package availability was environment-dependent. |
| 10 | Controller spawners failed | franka_robot_state_broadcaster spawner exit; arm/joint-state spawners could not acquire lock | Inspect controller-manager startup and duplicate launch sessions. Exact cause not established in the supplied logs. |
| 11 | Duplicate-node and RViz warnings | Duplicate camera/RViz names; plugin factory warning; /recognize_objects unavailable | Separate these warnings from the failing action. They do not by themselves prove a gripper fault. |
| 12 | franka_msgs missing | ImportError; package prefix/interface lookup failed; apt package unavailable | Source-build made Grasp interface available. Later prefix/interface commands succeeded. |
| 13 | Building the entire driver failed | FrankaConfig.cmake requested libfranka 0.19.0, not found for franka_gripper | franka_msgs built, but that did not mean the complete driver build succeeded. Maintain a compatible driver/libfranka environment. |
| 14 | TF buffer absent in Python | VisionPickPlaceDemo had no attribute tf_buffer | Initialize Buffer and TransformListener in the node. Later logs progressed beyond this error. |
| 15 | PointStamped transform type unsupported | “Type ... PointStamped ... is not loaded or supported” | Use explicit do_transform_point with lookup_transform; later camera-to-base transformations succeeded. |
| 16 | Old code still running | Search of the build copy found no do_transform_point | Edit the source file, build the package, source the overlay, and inspect the version banner. |
| 17 | Pasted logs corrupted Python | orientation_constra03311442] caused SyntaxError | Remove terminal text; restore orientation_constraint.weight = 1.0; syntax-check before launching. |
| 18 | INVALID DEPTH despite a color target | Camera annotation alternated between target detection and invalid depth | Color detection does not guarantee a valid depth sample. Reject invalid data; investigate alignment, depth validity, ROI and occlusion rather than fabricate Z. |
| 19 | Detection repeatedly reset before 10/10 | Depth varied, e.g. around 0.928–0.946 m | Pairwise stability threshold caused resets. Ten accepted samples are not proof of accurate calibration. |
| 20 | Detection continued to 11/10 and beyond | Callbacks continued while START was pending | Freeze immediately at the required count and ignore later detections during the cycle. |
| 21 | START cancelled immediately or did not trigger motion | EOF/cancel paths and unconditional returns were present | Run interactive input with ros2 run; place return inside the appropriate except/if block. Later START was accepted. |
| 22 | Detection restarted after a failed/cancelled cycle | finally reset detection and timer | One-shot revision stopped automatic retries and required an explicit new run. |
| 23 | Executor already spinning | Error just after planning PRE_GRASP | Execute the blocking sequence outside callbacks; avoid concurrent/nested spinning of the same node. Later PRE_GRASP succeeded. |
| 24 | Robot blocked camera view | NO RED TARGET after arm approached | Acquire while target is visible, then freeze for a static cube. If the cube moves after freezing, abort/reacquire; do not assume tracking continues. |
| 25 | Preview mistaken for failure to move | PREVIEW ONLY and no motion sent | preview_only=true is intentional coordinate-only output, not collision or trajectory validation. |
| 26 | Geometry confirmation blocked execution | Execution blocked ... set geometry_confirmed | This is an explicit setup acknowledgement, not an automatic measurement or proof of safety. |
| 27 | PRE_GRASP-only mode mistaken for full grasp | Reached PRE_GRASP. Stopping before opening/descent | stop_after_pre_grasp=true stops before gripper opening. Full mode must be deliberately selected. |
| 28 | Awkward gripper orientation / one part touched table | Operator report of contact and tilted poses | Use the same configured vertical quaternion for approach/descent/lift; verify real tool axes and finger geometry. Operator later confirmed orientation. |
| 29 | Table geometry initially missing or wrong | Missing measured parameters, then target outside rectangle | Measure height, footprint, origin and direction. Correcting size alone does not establish table center. |
| 30 | Hand-to-TCP distance confused with fingertip offset | fr3_hand to fr3_hand_tcp Z approximately 0.103 m | This is not TCP-to-lowest-fingertip distance. Use measured tool geometry. |
| 31 | Guessed 0.12 m fingertip offset blocked grasp | Conservative upper bound made minimum TCP height too high | Replaced with approximate 0.056 m from a measured pose/gap; uncertainty remains. |
| 32 | Requested grasp below minimum height | GRASP z=0.1607, minimum=0.1960 | Guard correctly refused motion. Do not falsify geometry to pass it; review grasp height and depth. |
| 33 | Vision Z implausibly close to tabletop | Object Z around 0.134–0.142 m versus table 0.130 m | Verify that depth corresponds to cube surface and that table/base calibration is correct. Larger offsets do not fix faulty depth. |
| 34 | Cube not centered between fingers | Robot reached commanded XYZ, but operator saw lateral offset | Measured a local correction from manual alignment; do not infer axes or millimeters from a perspective photograph. |
| 35 | Target varied between repeated runs | Corrected target differed from manually aligned reference by millimeters | Check fresh unobstructed detection, frozen sample policy and calibration. Do not keep accumulating offsets. |
| 36 | Gripper did not move despite success | Opening, grasp and release all reported about 0.03328 m | CLI reproduced it. Thus not solely a vision-Python fault. Recovery reported after homing; exact underlying fault was not proven. |
| 37 | Completion log despite no physical grasp | Sequence went through PRE_BIN/BIN/HOME while fingers did not move | Server success alone is insufficient physical task validation. Opening verification added; grasp verification remains limited. |
| 38 | TCP TF too old after opening | TCP TF age is 1.094s | Added a bounded fresh-TF wait before Cartesian motion. Next reported run passed freshness and reached start-pose validation. |
| 39 | Cartesian start outside position tolerance | Start error 3.23 mm, 0.49 degrees | 3 mm position guard rejected; 1 degree orientation guard passed. Conditional 4 mm proposal remains unverified. |
| 40 | Placement looked awkward | PRE_BIN/BIN/POST_BIN were saved joint configurations | Vision changes pickup position, not saved bin orientation. Re-teach/plan compatible placement poses; joint waypoints do not guarantee straight placement. |

## 4. Calibration and TF evidence

The solved 17-sample candidate gave:

| Quantity | Value |
| --- | --- |
| Camera optical origin in base [m] | [0.441892919, -0.349669241, 1.283480356] |
| Optical orientation in base [xyzw] | [0.991330877, -0.011721972, 0.025890043, -0.128278574] |
| Mount translation RMS / maximum | 1.277135 / 2.400874 mm |
| Mount rotation RMS / maximum | 0.174579 / 0.355993 degrees |
| Candidate status | validated: false |

These are consistency residuals on calibration data, not a guarantee of cube localization accuracy across the table.

Collection lesson: wait for the moving board to stop, require stable valid camera-to-board poses, and pair them with time-consistent robot poses. Do not count repeated stationary samples as independent pose diversity.

The working topology was:

```text
fr3_link0 -> camera_link -> camera_color_frame -> camera_color_optical_frame
```

The session used these environment settings in newly started processes:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

Read-only verification:

```bash
ros2 run tf2_ros tf2_echo fr3_link0 camera_link
ros2 run tf2_ros tf2_echo fr3_link0 camera_color_optical_frame
```

Run one query at a time. An initial “frame does not exist” followed by valid data can be startup discovery; persistent disconnected-tree errors need investigation. Static transforms showing time 0.0 are different from stale dynamic TCP transforms.

See the linked Stage 06 issue document for the session-specific static-publisher command. Reuse its calibration only if the camera mounting has not changed, and avoid publishing the same child frame from multiple authorities.

## 5. Tool orientation, table geometry and grasp height

### 5.1 Vertical orientation

The configured goal quaternion was [0, 1, 0, 0] in xyzw order: a 180-degree rotation about Y. For this configuration, TCP +Z points along base -Z. This corresponds to downward approach only when the physical finger axis and horizontal-table assumption are correct.

The same quaternion was used for PRE_GRASP, GRASP and LIFT. Current TCP orientation was no longer copied into these targets. Actual measured orientation after a move was close to the goal, with small nonzero tracking error.

Goal orientation constraints on a normal pose plan constrain the endpoint; they do not alone guarantee constant orientation along the whole approach. The Cartesian descent/lift explicitly use common-orientation waypoints.

### 5.2 Session geometry estimates

| Parameter | Value used | Qualification |
| --- | --- | --- |
| table_top_z | 0.130 m | Operator's approximate table height above base; must refer to fr3_link0 origin |
| table_size_x | 1.22 m | Assumes long edge follows base X |
| table_size_y | 0.99 m | Assumes short edge follows base Y |
| table_center_x/y | 0 / 0 m | Assumes base origin is under actual table center |
| tcp_to_fingertip | 0.056 m | Approximate estimate, not a calibrated manufacturer value |
| fingertip_clearance | 0.020 m | Chosen modeled minimum gap |

Earlier bounds X=[0.10,0.90], Y=[-0.40,0.20] rejected a target near Y=+0.416 m. Under the later centered-table assumption, bounds became X=[-0.61,+0.61], Y=[-0.495,+0.495].

A rectangular tabletop collision box must reflect real installation openings and geometry. A solid box through the robot mounting region can create false collisions. The full finger geometry, pads and cable clearance also matter.

### 5.3 Fingertip offset measurement

A near-vertical pose gave TCP Z=0.216 m. With estimated table Z=0.130 m and reported lowest-tip gap about 0.030 m:

```text
tcp_to_fingertip = TCP_Z - table_top_z - measured_gap
                = 0.216 - 0.130 - 0.030
                = 0.056 m approximately
```

The measured orientation was slightly tilted, and both height/gap had uncertainty. Recheck in the intended vertical orientation. Do not deliberately touch the table to obtain this measurement.

The approximately 0.103 m fr3_hand-to-fr3_hand_tcp translation is a different offset; it cannot be substituted for tcp_to_fingertip.

### 5.4 Why the height guard rejected motion

For vertical fingers:

```text
lowest_tip_Z = TCP_Z - tcp_to_fingertip
minimum_TCP_Z = table_top_z + tcp_to_fingertip + fingertip_clearance
```

With a 10 mm gap, minimum TCP Z was 0.196 m. Requested Z=0.1607 m predicted a tip at 0.1047 m, below the assumed tabletop.

With the later 20 mm gap, minimum TCP Z became 0.206 m. A target TCP Z=0.216 m predicts about 30 mm clearance under the model, but still needs sufficient contact overlap with the cube.

The actual code continued to calculate:

```text
grasp_Z = detected_object_Z + grasp_offset
pregrasp_Z = detected_object_Z + pre_grasp_offset
lift_Z = grasp_Z + lift_offset
```

The session changed grasp_offset from 0.025 to 0.080 m via CLI; pre_grasp_offset remained 0.120 m. Thus descent distance became only 0.040 m. A fixed grasp Z=0.216 m was discussed but was **not implemented** in the uploaded revision. Do not confuse the minimum-height check with an automatic target-height adjustment.

## 6. Lateral alignment and local compensation

The first PRE_GRASP command was approximately [0.237,0.410,0.256] m, and measured TCP was [0.238,0.409,0.256] m. This showed close tracking of the requested position; it did not establish that the vision target matched the physical grasp center.

With the cube reported stationary, manual alignment gave [0.226,0.388,0.255] m. The local horizontal correction used was:

```python
x, y, z = object_position
x -= 0.012
y -= 0.021
```

Apply this once, before geometry checks and before deriving all pickup waypoints. It is a local experimental correction, not a replacement for calibration or tool-center identification.

A subsequent target [0.217,0.382,0.261] m differed from the manually aligned reference by [-9,-6,+6] mm. The operator later reported acceptable location, but no workspace-wide validation was provided.

In the uploaded V2 file, the message “Object in fr3_link0” was printed **after** XY compensation. It therefore did not show raw transformed coordinates. V3 clarified the label to “Object after XY correction”.

Photographs showed projected finger/cube overlap, but perspective images alone did not establish exact offsets or base-axis directions. Compare the center between contact pads with the cube center, not a single finger edge.

## 7. Gripper failure, diagnosis and recovery

### 7.1 Misleading success

The sequence sent:

- Move: width=0.080 m, speed=0.050 m/s.
- Grasp: width=0.045 m, speed=0.020 m/s, force=10 N, epsilon inner/outer=0.010 m.
- Release Move: width=0.080 m.

All feedback stayed near 0.03328 m, yet success was reported and the arm continued to bin/home. The operator observed no finger movement.

A separate CLI Move at 0.080 m and 0.020 m/s reproduced the same unchanged feedback and success=true. This excluded the vision sequence as the sole cause; it did not uniquely identify a hardware or driver defect.

### 7.2 What the checks established

| Check | Observation |
| --- | --- |
| Move / Grasp action servers | One each, named /franka_gripper |
| Driver process | /opt/franka_ros2_ws/install/franka_gripper/lib/franka_gripper/franka_gripper_node |
| robot_ip | 172.16.0.2 |
| use_sim_time | false; this alone does not prove real hardware execution |
| Joint-state publisher | /franka_gripper |
| Finger positions | 0.01664026 m each; sum about 0.03328052 m |
| Duplicate-node warning | Camera/RViz duplicates were visible; not proof of duplicate gripper servers |

### 7.3 Standalone tests used

These commands cause physical finger motion. Stop the pick-and-place program first; use an empty gripper with clear opening/closing space.

```bash
# Opening test
ros2 action send_goal /franka_gripper/move \
  franka_msgs/action/Move \
  "{width: 0.08, speed: 0.02}" --feedback

# Empty-gripper closing test, not an object-holding Grasp
ros2 action send_goal /franka_gripper/move \
  franka_msgs/action/Move \
  "{width: 0.04, speed: 0.02}" --feedback

# Homing: physical motion; clear the entire finger travel first
ros2 action send_goal /franka_gripper/homing \
  franka_msgs/action/Homing "{}" --feedback
```

Homing was followed by an operator report of recovery. Record this as “recovered after homing”, not as proof of the exact fault. Do not run homing automatically before every pickup or while holding an object.

### 7.4 Python opening verification added

The original GripperController.move() returned result.success directly. The replacement preserved open_gripper()/close_gripper() and added:

1. Action terminal status must be SUCCEEDED and result.success must be true.
2. Subscribe to /franka_gripper/joint_states and sum the two named finger positions.
3. After action completion, require three consecutive new samples within 2 mm of the requested width.
4. Reject old, duplicate, nonfinite or incomplete samples; use a bounded 3-second verification window.
5. Return False when verification fails so the vision sequence does not descend.
6. Default Move speed becomes 0.020 m/s, matching the successful manual test; this speed difference was not proven to be the original cause.

The reported joint width and the physical gap between custom contact pads may differ. Validate their mapping. Fresh width confirmation is still not independent proof that an object was grasped.

The inherited close_on_object() still relied on Grasp success and retained width=0.045 m. Additional object-holding verification is outstanding. The helper also retains blocking action waits; its 3-second verification timeout is not an action-response timeout or emergency-stop mechanism.

## 8. Fresh TF after opening and Cartesian start rejection

### 8.1 Stale TF

Observed:

```text
Cartesian start rejected: TCP TF age is 1.094s
Straight descent was not fully planned/executed
```

The gripper helper spins its own node while opening. The vision node can have queued TF messages when control returns. V2 performed only one spin_once() before reading TCP TF, which could leave stale data in its buffer. This is a plausible code-level explanation; stopped or delayed publishers can produce similar symptoms.

V3 waits up to 3 seconds while processing callbacks, accepting a TCP transform aged 0–0.5 seconds. It then retains the position/orientation checks. If freshness never recovers, it refuses motion rather than accepting an old pose.

The next reported run reached geometric start validation, so freshness passed on that run.

### 8.2 Position threshold

Latest observed error:

```text
Cartesian start rejected: Start error: 3.23 mm, 0.49 degrees
Straight descent was not fully planned/executed
```

| Check | Measured | V3 threshold | Outcome |
| --- | --- | --- | --- |
| TCP position error | 3.23 mm | 3.00 mm | Rejected |
| TCP orientation error | 0.49 degrees | 1.00 degree | Passed |

A conditional change to 4 mm was suggested after physical clearance review. It was not included in the originally delivered V3 file and has no reported follow-up result. Do not repeatedly widen the threshold to force execution.

The Cartesian request contains a nominal start waypoint and a goal waypoint. Allowing a larger start mismatch can introduce an initial lateral correction before descent. It does not improve accuracy. Check settling, actual pose, TF timing and path clearance first.

Both V2/V3 require complete Cartesian fraction, supported speed-scaling fields, valid trajectory timing, and bounded joint steps. They must not silently execute a partial Cartesian path or fall back to an unchecked descent.

## 9. Why the saved YAML poses still matter

| File / function | Role in this experiment |
| --- | --- |
| camera_object_localizer.py | Detect target and publish camera-frame PointStamped; local code, not committed at this inspection |
| vision_pick_place_demo.py | Freeze target, transform, apply correction, validate geometry, plan pickup and sequence bin motions |
| object_callback() / try_start_sequence() | Fresh valid detections, pairwise stability, immediate one-shot freeze |
| transform_object_point() | TF at the detection timestamp plus explicit do_transform_point |
| check_geometry() | Table rectangle and simplified vertical-tool assumptions |
| check_cartesian_start() | Fresh actual TCP pose and start-error checks |
| move_to_cartesian_pose() | Normal MoveGroup pose goal for PRE_GRASP; not a straight-line guarantee |
| move_vertical_cartesian() | Collision-aware straight descent/lift request and trajectory validation |
| gripper_control.py | Move action for opening/release; later fresh-width verification |
| fixed_grasp_demo.py | Inherited MoveIt/grasp helpers and saved pose loading |
| automatic_pick_place_demo.py / add_bin_poses() | Load PRE_BIN, BIN, POST_BIN saved configurations |
| fixed_grasp/*.yaml | Older taught arm poses; inherited initialization may still require them even when vision computes pickup poses |
| fixed_pick_place/*.yaml | Saved bin approach, placement and retreat joint configurations |
| setup.py / package.xml | Executable installation, data files and dependencies |

The supplied saved pose files are JointState records, not Cartesian orientation files. Their seven arm joint positions determine the end-effector pose through forward kinematics. Changing “orientation” means teaching/planning a new joint configuration or using a Cartesian goal; adding a quaternion field to JointState YAML does not change the motion.

The inherited loader uses arm joint positions, not the recorded velocity/effort arrays or finger positions to execute gripper actions. Vision-based pickup does not automatically update the saved bin poses.

## 10. Read-only debugging and reproducibility

Use the same sourced environment as the running system:

```bash
ros2 pkg executables fr3_vision_sorting
ros2 pkg prefix franka_msgs
ros2 interface show franka_msgs/action/Grasp

ros2 action info /franka_gripper/move
ros2 action info /franka_gripper/grasp
ros2 param dump /franka_gripper
ros2 topic info /franka_gripper/joint_states --verbose
ros2 topic echo /franka_gripper/joint_states --once
ros2 node list
```

For TCP pose:

```bash
ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand_tcp
```

For source edits, use /workspace/ros2_ws/src/fr3_vision_sorting/fr3_vision_sorting/ rather than editing a generated build copy:

```bash
cd /workspace/ros2_ws
python3 -m py_compile src/fr3_vision_sorting/fr3_vision_sorting/vision_pick_place_demo.py
python3 -m py_compile src/fr3_vision_sorting/fr3_vision_sorting/gripper_control.py
colcon build --symlink-install --packages-select fr3_vision_sorting
source install/setup.bash
```

CLI parameter changes alone do not require rebuilding. CLI overrides take precedence over Python defaults. Check the startup banner to distinguish V2 from V3_TF_REFRESH.

Coordinate-only preview of the experimental settings:

```bash
ros2 run fr3_vision_sorting vision_pick_place_demo --ros-args \
  -p tcp_to_fingertip:=0.056 \
  -p fingertip_clearance:=0.020 \
  -p grasp_offset:=0.080 \
  -p preview_only:=true \
  -p stop_after_pre_grasp:=true
```

Preview sends no arm/gripper motion and does not validate a motion plan. These measurements and local corrections are specific to this setup, not universal FR3 settings.

## 11. Remaining validation work

- Resolve the latest start-position mismatch without blindly weakening guards.
- Confirm actual open width before descent and actual object retention before transport.
- Measure cube size, physical pad gap, TCP offset and table height more accurately.
- Investigate vision Z near tabletop height; confirm valid aligned depth on the intended cube.
- Validate localization at independent workspace positions; the local XY compensation has not been validated across the table.
- Ensure custom fingers, table mounting openings and carried object are represented appropriately in collision/payload configuration.
- Revisit saved bin poses and placement orientation; successful arrival at a joint pose does not establish appropriate physical placement.
- Record a complete physical pickup, retention during transport, release and return, alongside logs. A “Vision pick-and-place completed” message alone is insufficient.
