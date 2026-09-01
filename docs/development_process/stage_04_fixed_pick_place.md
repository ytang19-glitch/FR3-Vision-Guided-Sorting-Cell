# Stage 4 — Fixed-Position Pick and Place

> **Attention:** Stage 4 extends the successful Stage 3 grasping sequence. It does **not** duplicate the Stage 3 pose files.

Stage 4 reuses the validated Stage 3 grasp poses and adds only the three bin-related poses required for placement.

## Reusing Stage 3 configuration

The following poses are shared directly with Stage 3 and remain in `config/fixed_grasp/`:

| Pose | Configuration file | Purpose |
|---|---|---|
| `HOME` | `config/fixed_grasp/home_joint_state.yaml` | Safe starting and ending pose |
| `PRE_GRASP` | `config/fixed_grasp/pre_grasp_joint_state.yaml` | Safe pose above the object |
| `GRASP` | `config/fixed_grasp/grasp_joint_state.yaml` | Object pickup pose |
| `LIFT` | `config/fixed_grasp/lift_joint_state.yaml` | Safely lift the object |

Stage 4 also reuses the tested gripper parameters:

```python
GRASP_WIDTH = 0.039  # metres
GRASP_SPEED = 0.02   # metres per second
GRASP_FORCE = 10.0   # newtons
```

Stage 4 extends the successful fixed grasp from Stage 3 by carrying the object
to a known container and releasing it.

Do not use RealSense coordinates in this stage. The object and container remain
at fixed, manually measured positions.

## Objective

Complete the following sequence on the FR3:

```text
HOME → PRE_GRASP → OPEN → GRASP → CLOSE → LIFT
     → PRE_BIN → BIN → RELEASE → POST_BIN → HOME
```

Three new bin-related poses are introduced in Stage 4:

| Pose | Meaning |
|---|---|
| `PRE_BIN` | Safe position approximately 10–15 cm above the container |
| `BIN` | Safe release position inside or immediately above the container |
| `POST_BIN` | Vertical retreat position after releasing the object |

This approach prevents the arm from moving horizontally while the gripper is
inside or close to the container.

## Prerequisites

Complete Stage 3 before beginning this stage:

- `HOME → PRE_GRASP → GRASP → LIFT → HOME` works reliably;
- `/franka_gripper/grasp` returns `success: true`;
- the object remains stable during `LIFT`;
- the arm controller is active;
- the robot has no Franka Desk errors;
- Stage 3 grasp parameters have been validated for the test object.

The parameters validated for the current object were:

```text
Target width:  0.039 m
Speed:         0.020 m/s
Force:         10.0 N
Inner epsilon: 0.005 m
Outer epsilon: 0.005 m
```

These values are object-specific. Measure and validate them again when the
object, finger geometry, grasp location, or orientation changes.

## Required pose-file layout

The pose files should be organized by stage responsibility:

```text
ros2_ws/src/fr3_vision_sorting/config/
├── fixed_grasp/
│   ├── home_joint_state.yaml
│   ├── pre_grasp_joint_state.yaml
│   ├── grasp_joint_state.yaml
│   └── lift_joint_state.yaml
└── fixed_pick_place/
    ├── pre_bin_joint_state.yaml
    ├── bin_joint_state.yaml
    └── post_bin_joint_state.yaml
```

The Stage 3 files stay in `fixed_grasp/`. Stage 4 references those files rather
than copying them into `fixed_pick_place/`.

The completed ROS 2 package should therefore contain:

```text
ros2_ws/src/fr3_vision_sorting/
├── config/
│   ├── fixed_grasp/
│   │   ├── home_joint_state.yaml
│   │   ├── pre_grasp_joint_state.yaml
│   │   ├── grasp_joint_state.yaml
│   │   └── lift_joint_state.yaml
│   └── fixed_pick_place/
│       ├── pre_bin_joint_state.yaml
│       ├── bin_joint_state.yaml
│       └── post_bin_joint_state.yaml
├── fr3_vision_sorting/
│   ├── __init__.py
│   ├── fixed_pose_demo.py
│   ├── gripper_control.py
│   ├── fixed_grasp_demo.py
│   └── fixed_pick_place_demo.py
├── package.xml
└── setup.py
```

## 1. Start the real FR3 and MoveIt

On the Ubuntu host, allow Docker GUI applications if necessary:

```bash
xhost +local:docker
```

Enter the container:

```bash
docker exec -it fr3_vision_sorting bash
```

Source the workspaces:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash
```

Launch the official FR3 MoveIt configuration:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=172.16.0.2 \
  use_fake_hardware:=false
```

Keep this terminal running.

In another container terminal, verify the controller:

```bash
ros2 control list_controllers
```

The arm controller must report:

```text
fr3_arm_controller ... active
```

## 2. Confirm the joint-state topic and configuration folders

```bash
ros2 topic list | grep joint_states
ros2 topic hz /joint_states
```

Use the topic that is actively publishing. The commands below assume that it
is `/joint_states`.

Create both configuration directories if they do not already exist:

```bash
mkdir -p \
  /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_grasp \
  /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_pick_place
```

### Record or refresh `HOME`

`HOME` belongs to the shared Stage 3 `fixed_grasp` configuration because it is
used as the safe starting and ending state by both Stage 3 and Stage 4.

Move the robot to the validated HOME pose in RViz, wait until it stops, then:

```bash
ros2 topic echo /joint_states --once \
  > /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_grasp/home_joint_state.yaml
```

Check it:

```bash
cat \
  /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_grasp/home_joint_state.yaml
```

If the existing Stage 3 HOME pose is already validated, do not overwrite it
unless you intentionally want to redefine HOME.

## 3. Move existing Stage 4 bin files into the correct folder

If your current directory looks like this:

```text
config/
├── bin_joint_state.yaml
├── post_bin_joint_state.yaml
├── pre_bin_joint_state.yaml
├── fixed_grasp/
└── fixed_pick_place/
```

the three bin pose files are one level too high. Move them into
`config/fixed_pick_place/`:

```bash
cd /workspace/ros2_ws/src/fr3_vision_sorting/config

mv -v pre_bin_joint_state.yaml fixed_pick_place/
mv -v bin_joint_state.yaml fixed_pick_place/
mv -v post_bin_joint_state.yaml fixed_pick_place/
```

Do not move or duplicate `home_joint_state.yaml`. The shared HOME pose must
remain here:

```text
config/fixed_grasp/home_joint_state.yaml
```

Both `fixed_grasp_demo.py` and `fixed_pick_place_demo.py` reuse this HOME
file. If it does not exist yet, record it as described in the previous section.

After moving them, verify the layout:

```bash
find /workspace/ros2_ws/src/fr3_vision_sorting/config \
  -maxdepth 2 -type f -name '*.yaml' -print
```

Expected organization:

```text
config/fixed_grasp/home_joint_state.yaml
config/fixed_grasp/pre_grasp_joint_state.yaml
config/fixed_grasp/grasp_joint_state.yaml
config/fixed_grasp/lift_joint_state.yaml
config/fixed_pick_place/pre_bin_joint_state.yaml
config/fixed_pick_place/bin_joint_state.yaml
config/fixed_pick_place/post_bin_joint_state.yaml
```

## 4. Record `PRE_BIN`

Place the container at its fixed experiment location. In RViz, move the empty
gripper to a pose:

- centered over the container opening;
- approximately 10–15 cm above the container;
- clear of the table, camera stand, and container walls;
- reachable without approaching a joint limit;
- with the same end-effector orientation intended for release.

After the robot stops completely, record the pose directly into the Stage 4
folder:

```bash
ros2 topic echo /joint_states --once \
  > /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_pick_place/pre_bin_joint_state.yaml
```

## 5. Record `BIN`

From `PRE_BIN`, descend vertically and slowly to the release pose.

The `BIN` pose must satisfy all of the following:

- the object is inside or immediately above the container opening;
- the object can fall only a short, safe distance;
- both fingers can open without touching the container walls;
- the gripper remains clear of the container bottom;
- the arm can retreat vertically after release.

Record the pose:

```bash
ros2 topic echo /joint_states --once \
  > /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_pick_place/bin_joint_state.yaml
```

## 6. Record `POST_BIN`

Move vertically upward from `BIN` by approximately 10–15 cm. Do not begin with
a horizontal retreat.

Record the pose:

```bash
ros2 topic echo /joint_states --once \
  > /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_pick_place/post_bin_joint_state.yaml
```

If `POST_BIN` is intentionally identical to `PRE_BIN`, it may initially be
copied:

```bash
cp \
  /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_pick_place/pre_bin_joint_state.yaml \
  /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_pick_place/post_bin_joint_state.yaml
```

## 7. Validate the recorded files

```bash
ls -lh \
  /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_grasp

ls -lh \
  /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_pick_place
```

Check one Stage 4 file:

```bash
grep -A12 '^name:' \
  /workspace/ros2_ws/src/fr3_vision_sorting/config/fixed_pick_place/pre_bin_joint_state.yaml
```

The message may contain seven FR3 arm joints and two gripper joints. The Python
node selects and reorders only `fr3_joint1` through `fr3_joint7`.

`ros2 topic echo` may write a YAML separator named `---`. The pose loader in
`fixed_grasp_demo.py` must use `yaml.safe_load_all()` so this separator does not
cause a parsing error.

## 8. Add `fixed_pick_place_demo.py`

Place the Stage 4 node at:

```text
/workspace/ros2_ws/src/fr3_vision_sorting/
└── fr3_vision_sorting/
    └── fixed_pick_place_demo.py
```

The node reuses the tested functions from:

```python
from fr3_vision_sorting.fixed_grasp_demo import (
    FixedGraspDemo,
    require_confirmation,
)
from fr3_vision_sorting.gripper_control import GripperController
```

The current object parameters are defined near the top of the node:

```python
GRASP_WIDTH = 0.039
GRASP_SPEED = 0.02
GRASP_FORCE = 10.0
```

The Stage 4 node should load:

```text
HOME / PRE_GRASP / GRASP / LIFT
    from config/fixed_grasp/

PRE_BIN / BIN / POST_BIN
    from config/fixed_pick_place/
```

## 9. Update `setup.py`

Because the YAML files are now stored in subdirectories, install both folders
explicitly instead of relying only on `glob("config/*.yaml")`:

```python
(
    os.path.join("share", package_name, "config", "fixed_grasp"),
    glob("config/fixed_grasp/*.yaml"),
),
(
    os.path.join("share", package_name, "config", "fixed_pick_place"),
    glob("config/fixed_pick_place/*.yaml"),
),
```

Add the Stage 4 executable while retaining the earlier stages:

```python
entry_points={
    "console_scripts": [
        "fixed_pose_demo = fr3_vision_sorting.fixed_pose_demo:main",
        "gripper_control = fr3_vision_sorting.gripper_control:main",
        "fixed_grasp_demo = fr3_vision_sorting.fixed_grasp_demo:main",
        "fixed_pick_place_demo = fr3_vision_sorting.fixed_pick_place_demo:main",
    ],
},
```

## 10. Build the package

```bash
cd /workspace/ros2_ws

source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash

colcon build --symlink-install \
  --packages-select fr3_vision_sorting

source install/setup.bash
```

A successful build ends with:

```text
Finished <<< fr3_vision_sorting
Summary: 1 package finished
```

Confirm the executable:

```bash
ros2 pkg executables fr3_vision_sorting \
  | grep fixed_pick_place_demo
```

Expected output:

```text
fr3_vision_sorting fixed_pick_place_demo
```

## 11. Validate the bin path without an object

Before the complete pick-and-place test, use RViz at low speed to plan and
execute each transition with an empty gripper:

```text
LIFT → PRE_BIN
PRE_BIN → BIN
BIN → POST_BIN
POST_BIN → HOME
```

Check that:

- the path does not contact the table;
- the arm does not contact the camera stand;
- the gripper does not contact the container;
- `BIN → POST_BIN` is a vertical retreat;
- `POST_BIN → HOME` begins only after clearing the container;
- the opened fingers have sufficient clearance at `BIN`.

Do not test a new path for the first time while holding an object.

## 12. Run the complete Stage 4 demo

Keep the official MoveIt launch terminal running. In another container
terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash
source /workspace/ros2_ws/install/setup.bash

ros2 run fr3_vision_sorting fixed_pick_place_demo
```

Before each step, the program displays the next requested action and requires:

```text
EXECUTE
```

Any other input cancels the sequence.

The first complete test should pause for inspection at these points:

1. `GRASP`: confirm finger alignment and table clearance.
2. `LIFT`: confirm that the object is stable.
3. `PRE_BIN`: confirm that the object is centered over the container.
4. `BIN`: confirm that both fingers can open safely.
5. `POST_BIN`: confirm that the gripper is clear before returning HOME.

## 13. Safe recovery after a failure

The program stops immediately if an arm or gripper action fails. Determine the
current robot pose before commanding another movement.

If the robot stops while holding the object:

1. Do not release it while unsupported.
2. Keep the enabling device and emergency stop accessible.
3. Move vertically to a known clearance pose at low speed when safe.
4. Place the object on a supported surface before opening the gripper.
5. Return HOME only after clearing the table and container.

Open the gripper only when the object is supported:

```bash
ros2 action send_goal \
  /franka_gripper/move \
  franka_msgs/action/Move \
  "{width: 0.08, speed: 0.02}"
```

## Planning Scene limitation

This first fixed-position node does not automatically add the table, camera
stand, container, or object to the MoveIt Planning Scene. MoveIt can therefore
avoid robot self-collision but cannot avoid unmodelled equipment.

Before increasing speed or running repeated autonomous cycles, add collision
geometry for:

- the table;
- the camera stand;
- the destination container;
- other fixed equipment;
- the grasped object.

Attach the object to `fr3_hand_tcp` after a successful grasp and detach it
after release.

## Stage 4 experiment log

| Cycle | Pick | Lift | Move to bin | Release | Return HOME | Error/notes |
|---:|:---:|:---:|:---:|:---:|:---:|---|
| 1 |  |  |  |  |  |  |
| 2 |  |  |  |  |  |  |
| 3 |  |  |  |  |  |  |
| 4 |  |  |  |  |  |  |
| 5 |  |  |  |  |  |  |
| 6 |  |  |  |  |  |  |
| 7 |  |  |  |  |  |  |
| 8 |  |  |  |  |  |  |
| 9 |  |  |  |  |  |  |
| 10 |  |  |  |  |  |  |

## Stage 4 success criteria

Stage 4 is complete when:

```text
10/10 fixed-position cycles succeed
Grasp action returns success = true
Object does not slip during transport
Object is released inside the container
No table, bin, camera-stand, or self-collision occurs
No controller or Franka Desk error occurs
Robot safely returns to HOME after every cycle
```
### Solution of wrong file

```bash
cd /workspace/ros2_ws/src/fr3_vision_sorting

mv config/pre_bin_joint_state.yaml \
   config/fixed_pick_place/

mv config/bin_joint_state.yaml \
   config/fixed_pick_place/

mv config/post_bin_joint_state.yaml \
   config/fixed_pick_place/
```

Do not add RealSense-based dynamic coordinates until the fixed-position
sequence is repeatable and safe.

---

[Development Process Index](README.md) · [Previous Stage](stage_03_fixed_grasping.md) · [Next Stage](stage_05_realsense_perception.md)
