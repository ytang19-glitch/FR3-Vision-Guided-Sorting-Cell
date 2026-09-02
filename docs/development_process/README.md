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

## Stage Guides

| Stage | Guide | Main result |
|---:|---|---|
| — | [FR3 ROS 2 Basic Command Cheat Sheet](FR3%20ROS%202%20Basic%20Command%20Cheat%20Sheet.md) | Quick reference for joint states, pose YAML files, gripper actions, build/run, TF and debugging commands |
| — | [FR3 Grasping Lessons and Troubleshooting](FR3_GRASPING_LESSONS_AND_TROUBLESHOOTING.md) | Vertical grasp alignment, `inner`/`outer` epsilon, known grasp results, failure modes and lessons learned |
| 1 | [Official FR3 model](stage_01_official_fr3.md) | Verified URDF, TF, controllers and fake hardware |
| 2 | [Repeatable fixed-pose motion](stage_02_fixed_pose_motion.md) | `HOME → PRE_GRASP → HOME` |
| 3 | [Fixed-position grasping](stage_03_fixed_grasping.md) | Grasp and lift one known object |
| 4 | [Fixed-position pick and place](stage_04_fixed_pick_place.md) | Move one object to a known bin |
| 5 | [RealSense perception](stage_05_realsense_perception.md) | Detect object and recover camera-frame XYZ |
| 6 | [Calibration and TF2](stage_06_calibration_tf2.md) | Transform camera XYZ into `fr3_link0` |
| 7 | [Vision-guided sorting](stage_07_vision_guided_sorting.md) | Autonomous class-based sorting |

Read the [Safety Principle](SAFETY.md) before operating real hardware.

## Project Structure

The manipulation stages reuse earlier tested components instead of duplicating
their code:

```text
fr3_vision_sorting/
├── Dockerfile
├── compose.yaml
├── start_container.sh
├── README.md
├── docs/
│   └── development_process/
│       ├── README.md
│       ├── stage_01_official_fr3.md
│       ├── stage_02_fixed_pose_motion.md
│       ├── stage_03_fixed_grasping.md
│       └── stage_04_fixed_pick_place.md
└── ros2_ws/
    └── src/
        └── fr3_vision_sorting/
            ├── package.xml
            ├── setup.py
            ├── setup.cfg
            ├── resource/
            │   └── fr3_vision_sorting
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
            └── fr3_vision_sorting/
                ├── __init__.py
                ├── fixed_pose_demo.py
                ├── gripper_control.py
                ├── fixed_grasp_demo.py
                ├── fixed_pick_place_demo.py
                └── automatic_pick_place_demo.py
```

## Code reuse between Stage 3 and Stage 4

Stage 3 defines the reusable `FixedGraspDemo` class in
`fixed_grasp_demo.py`. It provides:

- loading and reordering saved FR3 joint states;
- connection to MoveIt's `/move_action` server;
- movement to `HOME`, `PRE_GRASP`, `GRASP`, and `LIFT`;
- connection to Franka's `/franka_gripper/grasp` action;
- the validated grasp width, speed, force, and epsilon handling.

Stage 4 imports this tested implementation:

```python
from fr3_vision_sorting.fixed_grasp_demo import (
    FixedGraspDemo,
    require_confirmation,
)
from fr3_vision_sorting.gripper_control import GripperController
```

It then creates the shared Stage 3 controller:

```python
demo = FixedGraspDemo()
```

This automatically loads the four poses stored in `config/fixed_grasp/`:

```text
HOME
PRE_GRASP
GRASP
LIFT
```

Stage 4 adds only the destination poses from `config/fixed_pick_place/`:

```python
demo.poses["PRE_BIN"] = demo.load_joint_state(
    config_dir / "pre_bin_joint_state.yaml"
)
demo.poses["BIN"] = demo.load_joint_state(
    config_dir / "bin_joint_state.yaml"
)
demo.poses["POST_BIN"] = demo.load_joint_state(
    config_dir / "post_bin_joint_state.yaml"
)
```

The full reuse relationship is:

```text
fixed_grasp_demo.py
    ├── arm motion and grasp functions
    └── HOME / PRE_GRASP / GRASP / LIFT
                    ↓ imported by
fixed_pick_place_demo.py
    └── adds PRE_BIN / BIN / POST_BIN and release
                    ↓ reused by
automatic_pick_place_demo.py
    └── runs one supervised cycle without per-step prompts
```

This organization keeps one authoritative implementation of grasping. A fix to
`FixedGraspDemo.close_on_object()`, such as correcting the grasp epsilon, is
automatically used by both Stage 3 and Stage 4.

Do not copy the shared Stage 3 YAML files into `fixed_pick_place/`. Keeping
one copy prevents the two experiments from using inconsistent poses.

> **Real-hardware safety:** “Automatic” means the sequence runs without a
> confirmation between every step. A trained operator must still remain beside
> the FR3 with the enabling device and emergency stop accessible.

The official Franka packages are installed separately:

```text
/opt/franka_ros2_ws
```

Do not modify the official Franka workspace. Store student code in:

```text
/workspace/ros2_ws
```

---
