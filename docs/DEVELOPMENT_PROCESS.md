# FR3 Vision-Guided Sorting Cell — Development Process

This guide teaches students how to build the FR3 sorting experiment incrementally, from the official robot model to autonomous vision-guided sorting.

## Development Roadmap

```text
Stage 1: Official FR3 model and fake hardware
        ↓
Stage 2: HOME ↔ PRE_GRASP fixed-pose motion
        ↓
Stage 3: Fixed-position grasping
        ↓
Stage 4: Fixed-position pick and place
        ↓
Stage 5: RealSense RGB-D perception
        ↓
Stage 6: Camera calibration and TF2
        ↓
Stage 7: Vision-guided sorting
```

Follow the stages in order. Do not begin vision-guided motion until fixed-coordinate motion is repeatable and safe.

---

## Project Structure

The project should have the following structure:

```text
fr3_vision_sorting/
├── Dockerfile
├── compose.yaml
├── start_container.sh
├── README.md
├── docs/
│   └── DEVELOPMENT_PROCESS.md
└── ros2_ws/
    └── src/
        └── fr3_vision_sorting/
            ├── package.xml
            ├── setup.py
            ├── setup.cfg
            ├── resource/
            │   └── fr3_vision_sorting
            ├── config/
            │   ├── home_joint_state.yaml
            │   └── pre_grasp_joint_state.yaml
            └── fr3_vision_sorting/
                ├── __init__.py
                └── fixed_pose_demo.py
```

The official Franka packages are installed separately:

```text
/opt/franka_ros2_ws
```

Do not modify the official Franka workspace. Store student code in:

```text
/workspace/ros2_ws
```

---

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

# Stage 2 — Fixed-Position Grasping

Only begin after Stage 2 passes.

Add one function at a time:

```text
HOME
  ↓
PRE_GRASP
  ↓
OPEN GRIPPER
  ↓
DESCEND
  ↓
CLOSE GRIPPER
  ↓
LIFT
  ↓
HOME
```

Validate each new action separately.

Do not add RealSense coordinates yet.

---

# Stage 4 — Fixed-Position Pick and Place

Add a known bin pose:

```text
HOME
  → PRE_GRASP
  → OPEN
  → DESCEND
  → CLOSE
  → LIFT
  → MOVE_TO_BIN
  → RELEASE
  → HOME
```

Repeat this sequence at least ten times before adding vision.

---

# Stage 5 — RealSense Perception

After fixed-position pick and place works:

```text
RGB image
    ↓
OpenCV or YOLO detection
    ↓
Object class
    ↓
Center pixel (u,v)
    ↓
Aligned depth
    ↓
Camera-frame XYZ
```

At this stage, only calculate and visualize the object position. Do not immediately command the real FR3.

---

# Stage 6 — Calibration and TF2

For a fixed overhead RealSense, estimate:

```text
camera frame → fr3_link0
```

Then transform the detected point:

```text
Camera XYZ
    ↓
TF2 transformation
    ↓
FR3 base XYZ
```

Display the transformed position as an RViz marker. The marker must appear on the physical object before it is used as a motion target.

---

# Stage 7 — Vision-Guided Sorting

Final sequence:

```text
RealSense RGB-D
    ↓
Detect and classify object
    ↓
Calculate camera-frame XYZ
    ↓
Transform into fr3_link0
    ↓
Generate PRE_GRASP
    ↓
MoveIt planning
    ↓
Grasp object
    ↓
Place object in class-specific bin
```

Record:

- localization error;
- grasp success rate;
- sorting success rate;
- planning time;
- total cycle time.

---

# Safety Principle

Always develop in this order:

```text
Simple
  ↓
Repeatable
  ↓
Fake hardware
  ↓
Low-speed real hardware
  ↓
Fixed-coordinate grasping
  ↓
Vision
  ↓
Autonomous operation
```

Never test a new motion for the first time at full speed on the real FR3.
