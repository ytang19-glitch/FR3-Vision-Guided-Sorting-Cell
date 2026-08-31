# FR3 ROS 2 Basic Command Cheat Sheet

This document contains the most frequently used ROS 2 commands for the FR3 fixed-position grasping and pick-and-place project.

---

# 1. Enter the Docker Container

```bash
docker exec -it fr3_vision_sorting bash
```

---

# 2. Source the Environment

Run this in every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash
```

---

# 3. Go to the Project

```bash
cd /workspace/ros2_ws/src/fr3_vision_sorting
```

Check the project files:

```bash
ls
```

Typical structure:

```text
fr3_vision_sorting/
├── config/
├── fr3_vision_sorting/
├── launch/
├── package.xml
├── resource/
├── setup.cfg
└── setup.py
```

---

# 4. Go to the Config Folder

The recorded robot poses are stored here:

```bash
cd /workspace/ros2_ws/src/fr3_vision_sorting/config
```

Check existing pose files:

```bash
ls
```

For example:

```text
home_joint_state.yaml
pre_grasp_joint_state.yaml
grasp_joint_state.yaml
lift_joint_state.yaml
bin_joint_state.yaml
release_joint_state.yaml
```

---

# 5. Check the Current Robot Joint State

Show one complete joint-state message:

```bash
ros2 topic echo /joint_states --once
```

Show only joint names:

```bash
ros2 topic echo /joint_states --once --field name
```

Show only joint positions:

```bash
ros2 topic echo /joint_states --once --field position
```

---

# 6. Create and Record a New Pose YAML

You do NOT need to use `touch` first.

The `>` operator automatically creates the YAML file.

First enter the config directory:

```bash
cd /workspace/ros2_ws/src/fr3_vision_sorting/config
```

Then record the current robot pose:

```bash
ros2 topic echo /joint_states --once > new_pose.yaml
```

For example, create a BIN pose:

```bash
ros2 topic echo /joint_states --once > bin_joint_state.yaml
```

Create a RELEASE pose:

```bash
ros2 topic echo /joint_states --once > release_joint_state.yaml
```

---

# 7. Record the Main Robot Poses

## HOME

Move the robot to HOME and run:

```bash
cd /workspace/ros2_ws/src/fr3_vision_sorting/config

ros2 topic echo /joint_states --once \
  > home_joint_state.yaml
```

## PRE_GRASP

Move the robot to PRE_GRASP and run:

```bash
ros2 topic echo /joint_states --once \
  > pre_grasp_joint_state.yaml
```

## GRASP

Keep the gripper OPEN.

Move slowly from PRE_GRASP down to the desired grasp position.

Then record:

```bash
ros2 topic echo /joint_states --once \
  > grasp_joint_state.yaml
```

The sequence is:

```text
PRE_GRASP
    ↓
OPEN GRIPPER
    ↓
DESCEND
    ↓
GRASP POSE
    ↓
CLOSE GRIPPER
```

## LIFT

Move to the safe LIFT pose and record:

```bash
ros2 topic echo /joint_states --once \
  > lift_joint_state.yaml
```

## BIN

Move to the desired BIN pose:

```bash
ros2 topic echo /joint_states --once \
  > bin_joint_state.yaml
```

## RELEASE

Move to the desired release pose:

```bash
ros2 topic echo /joint_states --once \
  > release_joint_state.yaml
```

---

# 8. Check the Saved YAML Files

List all files:

```bash
cd /workspace/ros2_ws/src/fr3_vision_sorting/config

ls -lh
```

Read HOME:

```bash
cat home_joint_state.yaml
```

Read PRE_GRASP:

```bash
cat pre_grasp_joint_state.yaml
```

Read GRASP:

```bash
cat grasp_joint_state.yaml
```

Read LIFT:

```bash
cat lift_joint_state.yaml
```

Read BIN:

```bash
cat bin_joint_state.yaml
```

You can also print line numbers:

```bash
cat -n grasp_joint_state.yaml
```

---

# 9. Compare Two Recorded Poses

Compare PRE_GRASP and GRASP:

```bash
diff pre_grasp_joint_state.yaml grasp_joint_state.yaml
```

Compare HOME and LIFT:

```bash
diff home_joint_state.yaml lift_joint_state.yaml
```

This is useful for checking how much the joint configuration changed between two poses.

---

# 10. Find Joint-State Topics

```bash
ros2 topic list | grep joint
```

Check whether `/joint_states` is publishing:

```bash
ros2 topic hz /joint_states
```

If necessary, check the FR3-specific topic:

```bash
ros2 topic hz /fr3/franka/joint_states
```

---

# 11. Open the Gripper

Open the gripper to 80 mm:

```bash
ros2 action send_goal \
  /franka_gripper/move \
  franka_msgs/action/Move \
  "{width: 0.08, speed: 0.02}" \
  --feedback
```

---

# 12. Grasp an Object

Current known working parameters:

```text
width   = 0.039 m
speed   = 0.02 m/s
force   = 10 N
epsilon = ±0.005 m
```

Command:

```bash
ros2 action send_goal \
  /franka_gripper/grasp \
  franka_msgs/action/Grasp \
  "{width: 0.039, speed: 0.02, force: 10.0, epsilon: {inner: 0.005, outer: 0.005}}" \
  --feedback
```

Example successful result:

```text
current_width: 0.040586

success: true
error: ''

Goal finished with status: SUCCEEDED
```

---

# 13. Test Different Grasp Parameters

Example:

```bash
ros2 action send_goal \
  /franka_gripper/grasp \
  franka_msgs/action/Grasp \
  "{width: 0.045, speed: 0.02, force: 10.0, epsilon: {inner: 0.005, outer: 0.005}}" \
  --feedback
```

Always use conservative force and speed values when testing on the real robot.

---

# 14. Check Gripper Actions

List available gripper actions:

```bash
ros2 action list | grep gripper
```

Inspect the grasp action:

```bash
ros2 action info /franka_gripper/grasp
```

Inspect the move action:

```bash
ros2 action info /franka_gripper/move
```

---

# 15. Check the Action Message Format

If you forget how to write a command, use:

```bash
ros2 interface show franka_msgs/action/Grasp
```

For gripper movement:

```bash
ros2 interface show franka_msgs/action/Move
```

This is one of the most useful ROS 2 debugging commands.

---

# 16. Check ROS 2 Nodes

```bash
ros2 node list
```

---

# 17. Check ROS 2 Topics

```bash
ros2 topic list
```

Useful filters:

```bash
ros2 topic list | grep joint
```

```bash
ros2 topic list | grep camera
```

```bash
ros2 topic list | grep tf
```

---

# 18. Check Controllers

```bash
ros2 control list_controllers
```

The controllers required by the robot should show:

```text
active
```

---

# 19. Build the ROS 2 Package

After changing Python code, package configuration, or adding installed config files:

```bash
cd /workspace/ros2_ws

source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash

colcon build --symlink-install \
  --packages-select fr3_vision_sorting

source install/setup.bash
```

---

# 20. Check Package Executables

```bash
ros2 pkg executables fr3_vision_sorting
```

For example:

```text
fr3_vision_sorting fixed_grasp_demo
```

---

# 21. Run the Fixed Grasp Demo

```bash
ros2 run fr3_vision_sorting fixed_grasp_demo
```

Current sequence:

```text
HOME
 ↓
PRE_GRASP
 ↓
OPEN
 ↓
GRASP
 ↓
CLOSE
 ↓
LIFT
 ↓
HOME
```

---

# 22. Check the Grasp Parameters in the Python File

```bash
grep -n -A5 "demo.close_on_object" \
  /workspace/ros2_ws/src/fr3_vision_sorting/fr3_vision_sorting/fixed_grasp_demo.py
```

Expected:

```python
width=0.039,
speed=0.02,
force=10.0,
```

---

# 23. TF Commands

Check TF topics:

```bash
ros2 topic list | grep tf
```

Inspect TF:

```bash
ros2 topic echo /tf --once
```

Check the transform from the FR3 base to the end effector:

```bash
ros2 run tf2_ros tf2_echo fr3_link0 fr3_hand_tcp
```

---

# 24. RealSense Commands

Check whether the RealSense is detected:

```bash
lsusb | grep -i realsense
```

Find camera topics:

```bash
ros2 topic list | grep camera
```

Check RGB frequency:

```bash
ros2 topic hz /camera/camera/color/image_raw
```

---

# 25. Most Important Command Patterns

## Inspect ROS data

```bash
ros2 topic echo <topic>
```

Example:

```bash
ros2 topic echo /joint_states --once
```

## Send a robot command

```bash
ros2 action send_goal ...
```

## Save ROS data into a YAML file

```bash
ros2 topic echo /joint_states --once > pose.yaml
```

## Read a YAML file

```bash
cat pose.yaml
```

## Search inside a file

```bash
grep "position" pose.yaml
```

## Compare two poses

```bash
diff pose1.yaml pose2.yaml
```

## Check message/action structure

```bash
ros2 interface show <interface>
```

---

# Quick Pose Recording Workflow

This is the workflow to remember:

```bash
# 1. Go to config
cd /workspace/ros2_ws/src/fr3_vision_sorting/config

# 2. Check existing poses
ls

# 3. Check current robot position
ros2 topic echo /joint_states --once --field position

# 4. Record the new pose
ros2 topic echo /joint_states --once > new_pose_joint_state.yaml

# 5. Verify it
cat new_pose_joint_state.yaml

# 6. Check all recorded poses
ls -lh
```

Example:

```bash
cd /workspace/ros2_ws/src/fr3_vision_sorting/config

ros2 topic echo /joint_states --once \
  > bin_joint_state.yaml

cat bin_joint_state.yaml
```

---

# Quick Gripper Workflow

```bash
# OPEN
ros2 action send_goal \
  /franka_gripper/move \
  franka_msgs/action/Move \
  "{width: 0.08, speed: 0.02}" \
  --feedback
```

```bash
# GRASP
ros2 action send_goal \
  /franka_gripper/grasp \
  franka_msgs/action/Grasp \
  "{width: 0.039, speed: 0.02, force: 10.0, epsilon: {inner: 0.005, outer: 0.005}}" \
  --feedback
```

---

# Command Mental Model

Remember these six basic Linux/ROS 2 patterns:

```text
ros2 topic echo     → Read robot data

ros2 action         → Command an action

> file.yaml         → Save output / create file

cat file.yaml       → Read a file

grep                → Search

diff                → Compare
```

These commands form the basic CLI workflow for recording, inspecting, testing, and debugging FR3 robot poses.