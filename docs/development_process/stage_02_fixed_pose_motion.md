# Stage 2 — Repeatable HOME ↔ PRE_GRASP Motion

## Objective

Prove that the FR3 can safely and repeatedly reach two predefined poses without camera data:

```text
HOME → PRE_GRASP → HOME
```

Definitions:

- `HOME`: safe starting and resting pose;
- `PRE_GRASP`: safe preparation pose above the object;
- `GRASP`: lower pose where the gripper reaches the object.

For the first experiment, PRE_GRASP should remain approximately `20–30 cm` above the object.

Do not descend or close the gripper yet.

---

## Part A — Create the ROS 2 Package

If the package does not already exist:

```bash
mkdir -p /workspace/ros2_ws/src
cd /workspace/ros2_ws/src
```

Create it:

```bash
ros2 pkg create fr3_vision_sorting \
  --build-type ament_python \
  --dependencies \
  rclpy \
  geometry_msgs \
  sensor_msgs \
  visualization_msgs \
  moveit_msgs \
  shape_msgs \
  tf2_ros \
  tf2_geometry_msgs \
  cv_bridge
```

Create the configuration directory:

```bash
mkdir -p \
  /workspace/ros2_ws/src/fr3_vision_sorting/config
```

The Python node belongs in:

```text
/workspace/ros2_ws/src/fr3_vision_sorting/
└── fr3_vision_sorting/
    └── fixed_pose_demo.py
```

---

## Part B — Select and Record HOME

Start MoveIt in Terminal 1. Use fake hardware for initial development:

```bash
docker exec -it fr3_vision_sorting bash

source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true
```

In RViz:

1. choose a safe resting pose;
2. keep the arm away from the table and obstacles;
3. select `Plan`;
4. inspect the trajectory;
5. select `Execute`.

In Terminal 2:

```bash
docker exec -it fr3_vision_sorting bash

source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash
```

Record HOME:

```bash
ros2 topic echo /joint_states --once \
  > /workspace/ros2_ws/src/fr3_vision_sorting/config/home_joint_state.yaml
```

Check it:

```bash
cat \
  /workspace/ros2_ws/src/fr3_vision_sorting/config/home_joint_state.yaml
```

---

## Part C — Select and Record PRE_GRASP

In RViz:

1. position the gripper above the test object;
2. point the gripper downward;
3. remain approximately `20–30 cm` above the object;
4. select `Plan`;
5. inspect the trajectory;
6. select `Execute`.

Record PRE_GRASP:

```bash
ros2 topic echo /joint_states --once \
  > /workspace/ros2_ws/src/fr3_vision_sorting/config/pre_grasp_joint_state.yaml
```

Verify both files:

```bash
ls -lh \
  /workspace/ros2_ws/src/fr3_vision_sorting/config
```

Expected:

```text
home_joint_state.yaml
pre_grasp_joint_state.yaml
```

If the active topic is not `/joint_states`, find the correct topic with:

```bash
ros2 topic list | grep joint_states
```

---

## Part D — Build the Package

```bash
cd /workspace/ros2_ws

source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash

colcon build \
  --symlink-install \
  --packages-select fr3_vision_sorting

source install/setup.bash
```

Verify the package:

```bash
ros2 pkg prefix fr3_vision_sorting
```

Verify its executables:

```bash
ros2 pkg executables fr3_vision_sorting
```

Expected:

```text
fr3_vision_sorting fixed_pose_demo
```

---

## Part E — Understand the Fixed-Pose Program

The two YAML files define where the robot should move:

```text
YAML = target joint positions
```

The Python node defines how the experiment runs:

```text
fixed_pose_demo.py
    ↓
Read HOME YAML
    ↓
Read PRE_GRASP YAML
    ↓
Send a MoveGroup action request
    ↓
Official MoveIt plans the trajectory
    ↓
FR3 controller executes the trajectory
```

The node uses the official MoveIt action server:

```text
/move_action
```

This avoids loading another independent OMPL instance.

---

## Part F — Run the Fixed-Pose Demo

### Terminal 1 — Official MoveIt

For real hardware:

```bash
docker exec -it fr3_vision_sorting bash

source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=172.16.0.2 \
  use_fake_hardware:=false
```

### Terminal 2 — Verify MoveIt and Controllers

```bash
docker exec -it fr3_vision_sorting bash

source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash

ros2 action info /move_action
ros2 control list_controllers
```

Expected:

```text
/move_action action server available
fr3_arm_controller active
joint_state_broadcaster active
```

### Terminal 2 — Run the Student Node

```bash
ros2 run fr3_vision_sorting fixed_pose_demo
```

The node should display:

```text
Real robot sequence: HOME → PRE_GRASP → HOME
Type EXECUTE exactly to continue:
```

Before entering `EXECUTE`, confirm:

- workspace clear;
- emergency stop accessible;
- enable device in hand;
- velocity and acceleration scaling at `0.03`;
- HOME and PRE_GRASP already tested manually.

Then enter:

```text
EXECUTE
```

---

---

[Development Process Index](README.md) · [Previous Stage](stage_01_official_fr3.md) · [Next Stage](stage_03_fixed_grasping.md)
