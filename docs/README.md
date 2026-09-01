# Documentation

This directory contains the development notes, command references, experiment lessons, troubleshooting records, and stage-by-stage guides for the FR3 Vision-Guided Sorting Cell.

## Documentation Index

| Document | Purpose |
|---|---|
| [Development Process](development_process/README.md) | Stage-by-stage roadmap from the official FR3 setup to vision-guided sorting |
| [FR3 ROS 2 Basic Command Cheat Sheet](development_process/FR3%20ROS%202%20Basic%20Command%20Cheat%20Sheet.md) | Common ROS 2, YAML pose-recording, gripper, build, TF, and debugging commands |
| [FR3 Grasping Lessons and Troubleshooting](development_process/FR3_GRASPING_LESSONS_AND_TROUBLESHOOTING.md) | Issues encountered during the grasping experiment, solutions, vertical grasp alignment, and `inner`/`outer` epsilon behavior |
| [Safety Principle](development_process/SAFETY.md) | Safety guidance for real FR3 experiments |

## Key Grasping Experiment Lesson

For the current fixed-position top-down grasping experiment, the **vertical orientation of the end effector is one of the most important physical conditions for repeatable grasping**.

The intended sequence is:

```text
PRE_GRASP
    ↓
OPEN GRIPPER
    ↓
DESCEND VERTICALLY
    ↓
GRASP POSE
    ↓
CLOSE / GRASP
    ↓
LIFT VERTICALLY
```

A vertical grasp is important because it keeps the two fingers approximately symmetric around the object. If the hand is tilted or laterally offset, one finger can contact first and push or rotate the object before the second finger establishes contact. This can produce an unstable grasp even when the gripper action reports success.

Vertical approach and lift also reduce unnecessary horizontal motion close to the table, helping avoid sweeping the object away or contacting the table with the fingers.

The saved `grasp_joint_state.yaml` represents the **arm pose at the grasp location**. The gripper should remain open while moving from `PRE_GRASP` to `GRASP`, and the Franka `Grasp` action should be sent only after the object is correctly positioned between the fingers.

## Issues Encountered and Solutions

| Issue | Likely cause | Solution / lesson |
|---|---|---|
| Grasp reliability changed even with similar gripper parameters | End effector not sufficiently vertical or object not centered | Correct the physical GRASP pose before changing force or epsilon |
| Uncertainty about whether the gripper should already be closed at GRASP | GRASP pose and gripper state were being treated as the same thing | Keep the gripper open during `PRE_GRASP → GRASP`; close only after reaching the grasp pose |
| Object appeared held but action could report failure | Final measured width outside the accepted epsilon range | Compare `current_width` against `width - inner` and `width + outer` |
| Commanded width and measured width were different | Real contact geometry and compliance change the final stopped width | Treat `width` as the expected target and use a reasonable epsilon tolerance |
| `inner=0.08` made the lower tolerance extremely permissive | 80 mm tolerance is much larger than the 39 mm target width | Use tighter tolerances when repeatability and meaningful success detection matter |
| Pose YAML workflow was confusing | Unsure how to create/save pose files | Use `ros2 topic echo /joint_states --once > pose.yaml`; `>` creates the file automatically |
| Joint-state topic uncertainty | Robot setups can expose different joint-state topics | Use `ros2 topic list | grep joint` and verify the correct topic before recording |
| `fr3_vision_sorting fixed_grasp_demo` returned command not found | ROS package name was executed like a shell program | Run `ros2 run fr3_vision_sorting fixed_grasp_demo` |
| Code/config changes did not appear immediately | Workspace was not rebuilt or sourced | Run `colcon build --symlink-install --packages-select fr3_vision_sorting` and source `install/setup.bash` |

For the full explanation, calculations, successful grasp result, and debugging order, read [FR3 Grasping Lessons and Troubleshooting](development_process/FR3_GRASPING_LESSONS_AND_TROUBLESHOOTING.md).
