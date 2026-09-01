# FR3 Grasping Lessons and Troubleshooting

This note records the most important lessons learned while developing the fixed-position FR3 grasping workflow.

The main lesson is that a successful grasp is not determined by the gripper command alone. The **robot pose**, **gripper opening**, **object placement**, and **grasp tolerance parameters** all work together.

---

## 1. Why the vertical GRASP pose is critical

For a top-down grasp, the end effector should approach the object approximately vertically and the object should be centered between the two fingers before the grasp command is sent.

A good sequence is:

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

The GRASP pose is the arm pose at which the fingers are still open but are already positioned around the object.

### Why vertical alignment matters

If the hand is tilted or laterally offset, one finger may touch the object before the other. This can push the object away, rotate it, produce an uneven contact, or make the object slip during lift.

Vertical alignment also helps keep the motion predictable near the table. The approach direction is mostly along Z rather than sideways across the surface, which reduces the chance of sweeping the object away.

A good GRASP pose should satisfy:

- the object is centered between the fingers;
- both fingers have similar clearance before closing;
- the fingers are not touching the table;
- the hand has enough clearance from the table and surrounding objects;
- the arm is not close to a joint limit or singular configuration;
- the planned lift can move upward without collision.

The gripper should normally be **open before the descent begins**. Do not descend to the grasp location with the gripper already closed around empty space.

---

## 2. GRASP pose versus gripper state

The saved `grasp_joint_state.yaml` represents the **arm configuration**, not the fact that the fingers are already closed.

The intended logic is:

```text
Move arm to PRE_GRASP
        ↓
Open gripper
        ↓
Move arm to GRASP pose
        ↓
Execute Franka Grasp action
        ↓
Lift object
```

Therefore, when recording the GRASP pose, keep the gripper open and position the object between the two fingers.

Example:

```bash
cd /workspace/ros2_ws/src/fr3_vision_sorting/config

ros2 topic echo /joint_states --once \
  > grasp_joint_state.yaml
```

Verify the saved pose:

```bash
cat grasp_joint_state.yaml
```

---

## 3. Franka Grasp parameters

The current code uses:

```python
def close_on_object(
    self,
    width: float = 0.039,
    speed: float = 0.02,
    force: float = 10.0,
) -> bool:
    """Close on an object using Franka's contact-aware Grasp action."""
    goal = Grasp.Goal()
    goal.width = width
    goal.speed = speed
    goal.force = force
    goal.epsilon = GraspEpsilon(inner=0.08, outer=0.008)
```

The four important quantities are:

| Parameter | Meaning |
|---|---|
| `width` | Expected grasped object width in metres |
| `speed` | Finger closing speed |
| `force` | Requested grasping force |
| `epsilon` | Width tolerance used to decide whether the grasp is considered successful |

The Franka grasp success condition is approximately:

```text
(width - epsilon.inner) < measured_width < (width + epsilon.outer)
```

For example, with:

```python
width = 0.039
inner = 0.005
outer = 0.005
```

the accepted final width is approximately:

```text
0.034 m < measured width < 0.044 m
```

This is why `inner` and `outer` can significantly affect whether the action reports success.

---

## 4. Meaning of `inner`

`inner` is the allowed deviation when the measured grasp width is **smaller** than the commanded width.

Example:

```python
width = 0.039
inner = 0.005
```

The lower accepted width is:

```text
0.039 - 0.005 = 0.034 m
```

A larger `inner` tolerance allows the fingers to close further than expected and still report the grasp as successful.

This can help when the real object width is uncertain, but an excessively large value makes the success test much less informative.

---

## 5. Meaning of `outer`

`outer` is the allowed deviation when the measured grasp width is **larger** than the commanded width.

Example:

```python
width = 0.039
outer = 0.005
```

The upper accepted width is:

```text
0.039 + 0.005 = 0.044 m
```

This was important in the successful test because the measured width was:

```text
current_width = 0.040586 m
```

Compared with the target:

```text
0.040586 - 0.039 = 0.001586 m
```

The real grasp finished about **1.59 mm wider** than the commanded target. Therefore an `outer` tolerance greater than about `0.001586 m` can accept that measured width.

Both:

```python
outer=0.005
```

and:

```python
outer=0.008
```

are wide enough for that particular observed grasp result.

---

## 6. Important note about `inner=0.08`

The current line:

```python
goal.epsilon = GraspEpsilon(inner=0.08, outer=0.008)
```

uses an inner tolerance of **80 mm**.

For a target width of `39 mm`, the mathematical lower bound becomes:

```text
39 mm - 80 mm = -41 mm
```

Since a physical gripper width cannot be negative, this effectively makes the lower-side tolerance extremely permissive.

That may make the action easier to report as successful, but it also means the success result is less strict and may hide a poor grasp.

For a repeatable grasp, a tighter value such as:

```python
goal.epsilon = GraspEpsilon(
    inner=0.005,
    outer=0.005,
)
```

is usually easier to interpret because it corresponds to approximately ±5 mm around the requested width.

Do not tune epsilon only to force `success: true`. The object should also be physically stable and survive the lift.

---

## 7. Why pose and epsilon must be considered together

A large epsilon cannot compensate for a bad physical grasp pose.

For example:

```text
Bad hand alignment
      ↓
One finger contacts first
      ↓
Object shifts sideways
      ↓
Gripper stops at an unexpected width
      ↓
Large epsilon may still return success
      ↓
Object can slip during LIFT
```

The better approach is:

```text
Correct vertical pose
      +
Object centered between fingers
      +
Appropriate target width
      +
Reasonable epsilon
      +
Low safe speed and force
      ↓
Repeatable physical grasp
```

A `SUCCEEDED` action status is useful, but the real completion test is whether the object remains secure during the lift and transfer.

---

## 8. Known working grasp result

A successful test produced:

```text
current_width: 0.040586 m
success: true
status: SUCCEEDED
```

The tested command was based around:

```text
width: 0.039 m
speed: 0.02 m/s
force: 10 N
```

The observed final width was approximately `40.59 mm`.

This demonstrates that the commanded target width does not have to exactly equal the measured final width. The epsilon range is used to decide whether the measured result is acceptable.

---

## 9. Problems encountered during development

### Issue 1 — GRASP pose was not sufficiently vertical or centered

**Symptom:** grasp reliability changed even though the gripper parameters were similar.

**Cause:** the object was not equally positioned between both fingers, or the end effector orientation was not suitable for a clean top-down grasp.

**Lesson:** first fix the physical grasp pose before increasing force or tolerance.

---

### Issue 2 — Uncertainty about whether the gripper should be open at GRASP

**Resolution:** the gripper should be open during the PRE_GRASP → GRASP descent. The Franka Grasp action is sent only after the hand reaches the correct grasp pose.

```text
PRE_GRASP → OPEN → DESCEND → GRASP POSE → CLOSE
```

---

### Issue 3 — Grasp action failed even when the object appeared to be held

**Cause:** the final finger width can be outside the allowed epsilon interval even if physical contact occurs.

**Lesson:** compare the measured `current_width` with:

```text
width - inner
width + outer
```

before changing parameters randomly.

---

### Issue 4 — Target width and measured width were not identical

The target was:

```text
0.039 m
```

while the successful measured result was:

```text
0.040586 m
```

This is normal. The target is the expected grasp width, while the actual stopped width depends on object geometry, contact, compliance, and finger placement.

---

### Issue 5 — Pose YAML recording workflow was unclear

The reliable workflow is:

```bash
cd /workspace/ros2_ws/src/fr3_vision_sorting/config

ros2 topic echo /joint_states --once \
  > grasp_joint_state.yaml

cat grasp_joint_state.yaml
```

The `>` operator creates the YAML file automatically if it does not already exist.

---

### Issue 6 — Joint-state topic can differ between robot setups

Check available topics:

```bash
ros2 topic list | grep joint
```

Then verify that the selected topic is publishing:

```bash
ros2 topic hz /joint_states
```

or, when required:

```bash
ros2 topic hz /fr3/franka/joint_states
```

Always confirm that the recorded message contains the FR3 arm joints before saving a pose.

---

### Issue 7 — ROS executable command syntax

A ROS executable must normally be started with:

```bash
ros2 run fr3_vision_sorting fixed_grasp_demo
```

Running only:

```bash
fr3_vision_sorting fixed_grasp_demo
```

results in a shell `command not found` error because `fr3_vision_sorting` is a ROS package name, not a standalone shell executable.

---

### Issue 8 — Rebuild and source after package changes

After changing package code or installed configuration:

```bash
cd /workspace/ros2_ws

source /opt/ros/jazzy/setup.bash
source /opt/franka_ros2_ws/install/setup.bash

colcon build --symlink-install \
  --packages-select fr3_vision_sorting

source install/setup.bash
```

If a new executable does not appear, check:

```bash
ros2 pkg executables fr3_vision_sorting
```

---

## 10. Recommended debugging order

When a grasp fails, debug in this order:

```text
1. Check object placement
        ↓
2. Check vertical hand orientation
        ↓
3. Check object is centered between fingers
        ↓
4. Check GRASP height above the table
        ↓
5. Check gripper is open before descent
        ↓
6. Check target width
        ↓
7. Check measured current_width
        ↓
8. Check inner / outer epsilon
        ↓
9. Check force and speed
        ↓
10. Test LIFT stability
```

Do not begin by dramatically increasing force or epsilon.

---

## 11. Practical success criteria

A grasp should be considered stable only if all of the following are true:

```text
Grasp action reports success
        +
Object is centered and securely contacted
        +
Object does not rotate excessively
        +
Object does not slip during LIFT
        +
No table collision
        +
No controller / Franka error
        ↓
Stable grasp
```

For a fixed-position baseline, repeat the same grasp multiple times with the object returned to the same location. A repeatable physical grasp is more important than obtaining a single `success: true` result.
