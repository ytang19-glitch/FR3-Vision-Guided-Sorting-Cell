# Stage 1 — Official FR3 Model

## Objective

Verify that:

- the official FR3 description is installed;
- the robot appears correctly in RViz;
- joint states are published;
- the TF tree contains `fr3_link0` and `fr3_hand_tcp`;
- MoveIt can plan and execute using fake hardware.

Use `franka_description`. Do not copy or modify the official URDF.

## 1. Open the Project

Run on the Ubuntu host:

```bash
cd ~/fr3_vision_sorting
```

Allow Docker applications to open graphical windows:

```bash
echo "$DISPLAY"
xhost +local:docker
```

Start the container:

```bash
docker compose up -d
```

Check that it is running:

```bash
docker ps
```

Enter the container:

```bash
docker exec -it fr3_vision_sorting bash
```

## 2. Load ROS 2 and Franka

Inside Docker:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash
```

Check the packages:

```bash
ros2 pkg prefix franka_description
ros2 pkg prefix franka_fr3_moveit_config
```

## 3. Start Fake-Hardware MoveIt

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true
```

RViz should display the official FR3 model.

Use the RViz MotionPlanning panel to:

1. move the orange goal-state robot;
2. select `Plan`;
3. inspect the trajectory;
4. select `Execute`;
5. confirm that the displayed robot reaches the goal.

## 4. Verify Controllers

Open another terminal:

```bash
docker exec -it fr3_vision_sorting bash

source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash
```

Run:

```bash
ros2 control list_controllers
```

Expected:

```text
fr3_arm_controller       active
joint_state_broadcaster  active
```

## 5. Verify Joint States

Find the available topics:

```bash
ros2 topic list | grep joint_states
```

Check the publication rate:

```bash
ros2 topic hz /joint_states
```

Inspect one message:

```bash
ros2 topic echo /joint_states --once
```

A joint-state message contains:

- `name`: joint names;
- `position`: joint angles in radians;
- `velocity`: joint velocities;
- `effort`: measured or estimated effort.

## 6. Verify TF

```bash
ros2 run tf2_ros tf2_echo \
  fr3_link0 \
  fr3_hand_tcp
```

A continuously updated translation and rotation indicates that the transform is available.

## Stage 1 Success Criteria

```text
FR3 visible in RViz
Controllers active
Joint states publishing
TF available
Fake-hardware planning successful
Fake-hardware execution successful
```

---

# Before Moving the Real FR3

Real-hardware operation requires supervision and access to the emergency stop and enable device.

## 1. Check Franka Desk

```text
Control box connected
        ↓
Robot unlocked
        ↓
FCI enabled
        ↓
Robot ready
```

Confirm:

- no active robot errors;
- brakes released;
- FCI enabled;
- emergency stop accessible;
- enable device in hand;
- workspace clear of people and obstacles.

## 2. Check the Network

Run on the Ubuntu host:

```bash
ping -c 4 172.16.0.2
```

Expected:

```text
0% packet loss
```

## 3. Start Real-Hardware MoveIt

Stop the fake-hardware launch before starting real hardware.

Inside Docker:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=172.16.0.2 \
  use_fake_hardware:=false
```

## 4. Verify the Real Controller

```bash
ros2 control list_controllers
```

Expected:

```text
fr3_arm_controller       active
joint_state_broadcaster  active
```

Do not execute motion if either controller is inactive.

Set the first real-hardware test to:

```text
Velocity scaling:     0.03
Acceleration scaling: 0.03
```

---

---

[Development Process Index](README.md) · [Next Stage](stage_02_fixed_pose_motion.md)
