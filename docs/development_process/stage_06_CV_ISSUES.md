# Stage 06 — CV Issues: Disconnected Camera and Robot TF

## Observed issue (2026-09-18)

The detector reached 10/10 measurements and froze the target, but the pick-and-place node stopped before motion:

```text
Target frozen. This node is ignoring further detections.
Could not transform object point: Could not find a connection between
'fr3_link0' and 'camera_color_optical_frame' because they are not part
of the same tree. Tf has two or more unconnected trees.
Cycle ended. No automatic restart.
```

Detection succeeded, but the node could not obtain the camera-to-base transform. Moving only the cube does not disconnect TF and does not require recalibration. The cube position must be measured again after moving it.

## Confirmed recovery

After setting the following environment variables before launching the static publisher and TF queries, and restarting the static bridge, the operator confirmed that **both TF queries succeeded**:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

This records the successful recovery procedure for this setup. It does not independently prove that UDP configuration was the sole cause: the publisher was also restarted. The disconnected-tree message alone does not distinguish a missing publisher from a communication/configuration problem.

The first variable selects the Fast DDS ROS middleware implementation. The second requests UDPv4 built-in transport for Fast DDS. These settings affect subsequently started processes in that shell; they do not change already-running nodes or persist automatically in a new terminal.

## 1. Prepare both terminals

Run this block in **each** new terminal used for the static publisher and TF query:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

Use the same ROS domain and compatible network/container environment as the existing robot and camera nodes. Do not start a second full robot bringup to fix this TF issue.

## 2. Terminal 1: restore the static bridge

Stop the previous standalone publisher for this same transform if it is still running, then start one publisher:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.441874157 \
  --y -0.349579122 \
  --z 1.283422420 \
  --qx 0.438665 \
  --qy -0.424338 \
  --qz -0.540823 \
  --qw -0.578810 \
  --frame-id fr3_link0 \
  --child-frame-id camera_link
```

**Keep this terminal running.**

These numbers are the previously used calibration-derived pose of `camera_link` in `fr3_link0`. They apply only to the unchanged camera/base installation; they are not generic FR3 calibration values. Recalibrate if the camera moved relative to the robot base.

The camera driver supplies the internal camera transforms. Do not publish a second parent for `camera_color_optical_frame` or replace its internal transform with these numbers.

## 3. Terminal 2: verify the bridge and complete chain

First check the static bridge:

```bash
ros2 run tf2_ros tf2_echo fr3_link0 camera_link
```

After transform numbers appear, stop only this query with Ctrl+C and check the complete chain:

```bash
ros2 run tf2_ros tf2_echo fr3_link0 camera_color_optical_frame
```

Expected chain for this setup:

```text
fr3_link0 → camera_link → camera_color_frame → camera_color_optical_frame
```

The complete transform should approximately reproduce the previously solved optical-frame calibration:

```text
Translation [m]: [0.442, -0.350, 1.283]
Quaternion [x, y, z, w]: [0.991, -0.012, 0.026, -0.128]
```

Quaternion signs may all be reversed without changing the represented rotation.

| Result | Next check |
|---|---|
| Both queries succeed | The querying listener can resolve the complete camera-to-base chain. Restart the vision node with the same environment. |
| Bridge succeeds, optical-frame query fails | Check the camera driver's internal TF publication and exact frame names. |
| Bridge fails | Check publisher lifetime/output, ROS domain, middleware settings and network/container visibility. |

Successful TF lookup verifies connectivity, not calibration accuracy or collision clearance.

## 4. Restart target acquisition

The vision node ends a failed cycle without automatically restarting. In its terminal, apply the same setup and exports from step 1, then run a coordinate preview:

```bash
ros2 run fr3_vision_sorting vision_pick_place_demo \
  --ros-args -p preview_only:=true
```

Check the new `Object in fr3_link0` coordinates and PRE_GRASP/GRASP/LIFT targets. This preview does not plan or execute motion.

Keep the cube stationary after `Target frozen`. If it moves, restart acquisition rather than using the frozen coordinates. Keep the static bridge running throughout the session.

## Lessons learned

- A working camera detector does not guarantee that camera-to-robot TF is available.
- Apply the environment settings before starting the publisher, query and vision processes.
- New terminals do not inherit exports made in a separate terminal.
- Restarting the relevant processes matters; exporting variables alone does not reconfigure them.
- This session recovered with the UDPv4 settings and a restarted static publisher; both TF checks were confirmed successful.
- Moving the cube requires a fresh detection; moving the camera relative to the base requires recalibration.

Related guide: [Stage 06 — Calibration and TF2](stage_06_calibration_tf2.md).
