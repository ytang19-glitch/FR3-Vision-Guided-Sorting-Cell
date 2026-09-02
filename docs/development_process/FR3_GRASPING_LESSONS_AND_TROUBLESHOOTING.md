# FR3 Grasping Lessons and Troubleshooting

This note records the most important lessons learned while developing the fixed-position FR3 grasping workflow.

The main lesson is that a successful grasp is not determined by the gripper command alone. The **robot pose**, **gripper opening**, **object placement**, **grasp tolerance parameters**, and **action result handling** all work together.

The current fixed-grasp baseline is now working with the following validated grasp parameters:

```python
width = 0.045
speed = 0.02
force = 10.0
inner_epsilon = 0.010
outer_epsilon = 0.010
```

The recommended sequence is:

```text
HOME
 ↓
PRE_GRASP
 ↓
OPEN GRIPPER
 ↓
GRASP_POSE
 ↓
CLOSE_ON_OBJECT
 ↓
CHECK result.success
 ↓
LIFT
 ↓
HOME
```

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
EXECUTE FRANKA GRASP ACTION
    ↓
LIFT VERTICALLY
```

The saved GRASP pose is the **arm configuration** where the fingers are still open but already positioned around the object. It is not the gripper-closing action itself.

### Why vertical alignment matters

If the hand is tilted or laterally offset, one finger may touch the object before the other. This can:

- push the object sideways;
- rotate the object;
- create asymmetric contact;
- make the object slip;
- cause the gripper to stop at an unexpected width;
- make Franka return `success = false` even though contact occurred.

Ideal alignment:

```text
   gripper
   |     |
   | [■] |
   | [■] |
      ↑
    cube
```

Off-center alignment:

```text
   gripper
   |     |
   |   [■]
   |   [■]
        ↑
      cube
```

In the second case, one finger can touch first and move the cube before the second finger establishes proper contact.

A good GRASP pose should satisfy:

- the object is approximately centered between the fingers;
- both fingers have similar clearance before closing;
- the gripper is approximately vertical for a top-down grasp;
- the fingers are not touching the table;
- the hand has enough clearance from surrounding objects;
- the planned lift can move upward without collision.

---

## 2. GRASP pose versus gripper state

The saved `grasp_joint_state.yaml` represents the **arm pose**, not the fact that the fingers are closed.

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
Check grasp result
        ↓
Lift object only if grasp is confirmed
```

A clearer conceptual name for the saved pose is therefore:

```text
GRASP_POSE
```

rather than treating `GRASP` as the closing action itself.

The gripper should normally be **fully open before the descent begins**.

For the current setup, an opening near:

```text
0.080 m
```

provides useful lateral clearance before closing on the cube.

---

## 3. Current validated Franka Grasp parameters

The fixed-grasp baseline currently uses:

```python
def close_on_object(
    self,
    width: float = 0.045,
    speed: float = 0.02,
    force: float = 10.0,
) -> bool:
    goal = Grasp.Goal()
    goal.width = width
    goal.speed = speed
    goal.force = force
    goal.epsilon = GraspEpsilon(
        inner=0.010,
        outer=0.010,
    )
```

The important quantities are:

| Parameter | Meaning |
|---|---|
| `width` | Expected final grasp width in metres |
| `speed` | Finger closing speed |
| `force` | Requested grasping force |
| `inner` | Allowed deviation below the target width |
| `outer` | Allowed deviation above the target width |

The approximate success interval is:

```text
width - inner < measured_width < width + outer
```

For the current parameters:

```text
width = 0.045 m
inner = 0.010 m
outer = 0.010 m
```

which gives approximately:

```text
0.035 m < measured_width < 0.055 m
```

This range is intentionally more tolerant than the earlier ±5 mm test while the fixed grasp pose is still being tuned.

---

## 4. Why the CLI command could succeed while `fixed_grasp_demo` failed

A key troubleshooting lesson was that this command could succeed:

```bash
ros2 action send_goal \
  /franka_gripper/grasp \
  franka_msgs/action/Grasp \
  "{width: 0.045, speed: 0.02, force: 10.0, epsilon: {inner: 0.005, outer: 0.005}}" \
  --feedback
```

while the automatic program sometimes printed:

```text
[ERROR] [fixed_grasp_demo]: The gripper did not confirm a grasp.
```

This does **not automatically mean the Python action client is wrong**.

The CLI test and the automated demo may start from slightly different physical conditions:

```text
CLI test
  ↓
Cube manually centered
  ↓
Gripper open
  ↓
Grasp
  ↓
Success
```

versus:

```text
Automatic demo
  ↓
Move through HOME / PRE_GRASP / GRASP_POSE
  ↓
Small XY or orientation error
  ↓
One finger contacts first
  ↓
Cube moves slightly
  ↓
Unexpected final width
  ↓
Franka may return success = false
```

Therefore, when CLI succeeds but the demo fails, compare the **physical initial condition**, not only the numerical grasp parameters.

---

## 5. Why a slightly off-center cube can cause failure

If the cube is slightly away from the gripper center, the first contact can become asymmetric.

Example:

```text
Ideal

   |       |
   |  [■]  |
   |  [■]  |
      center
```

versus:

```text
Off center

   |       |
   |    [■]|
   |    [■]|
          ↑
       shifted cube
```

Possible consequences:

1. one finger touches the cube first;
2. the cube is pushed sideways;
3. the cube rotates;
4. the second finger contacts at a different point;
5. the final gripper width differs from the expected value;
6. the grasp action may return `success = false`.

Increasing force is not the first solution to this problem. More force can simply push or rotate a poorly aligned object harder.

The better order is:

```text
Fix pose geometry
      ↓
Improve centering
      ↓
Use enough opening before descent
      ↓
Use reasonable epsilon
      ↓
Then tune force if necessary
```

---

## 6. The role of epsilon

The current working code uses:

```python
goal.epsilon = GraspEpsilon(
    inner=0.010,
    outer=0.010,
)
```

With:

```text
width = 0.045 m
```

this gives an approximate accepted region of:

```text
35 mm -------- 45 mm -------- 55 mm
                 ↑
              target
```

This is useful when the real object contact width varies slightly because of:

- small XY placement error;
- finger contact location;
- small cube rotation;
- mechanical compliance;
- repeatability limits in the saved pose.

However:

> `epsilon` changes the success criterion; it does not physically center the cube or improve contact geometry.

Do not keep increasing epsilon only to force `success = true`.

A physically bad grasp can still drop the cube during lift even if the software reports success.

---

## 7. Why `inner=0.08` was not a good final setting

An earlier test used:

```python
goal.epsilon = GraspEpsilon(
    inner=0.08,
    outer=0.008,
)
```

For a target width near 39 mm, an `inner` tolerance of 80 mm makes the lower-side acceptance extremely permissive and difficult to interpret physically.

The lesson is:

```text
Large epsilon
    ≠
Better grasp
```

A smaller, interpretable tolerance should be preferred once the grasp pose is stable.

The current baseline uses:

```python
inner=0.010
outer=0.010
```

while the physical alignment is being improved.

---

## 8. Add feedback logging to diagnose grasp failures

The original program only reported:

```text
The gripper did not confirm a grasp.
```

That message alone is not enough to diagnose the failure.

The improved action client should attach a feedback callback:

```python
send_future = self.grasp_client.send_goal_async(
    goal,
    feedback_callback=self.grasp_feedback_callback,
)
```

Example callback:

```python
def grasp_feedback_callback(self, feedback_msg) -> None:
    current_width = feedback_msg.feedback.current_width
    self.get_logger().info(
        f"Gripper width: {current_width:.5f} m"
    )
```

A successful run may look like:

```text
Gripper width: 0.07999 m
Gripper width: 0.07052 m
Gripper width: 0.06184 m
Gripper width: 0.05312 m
Gripper width: 0.04721 m
Gripper width: 0.04634 m
```

This allows the final physical finger position to be compared against the expected epsilon interval.

---

## 9. Always print the Franka action result

The program should explicitly inspect:

```python
wrapped_result = result_future.result()
result = wrapped_result.result
```

and log:

```python
self.get_logger().info(
    f"Grasp result: success={result.success}, "
    f"error='{result.error}'"
)
```

This distinguishes between several failure modes:

```text
Goal rejected
      ≠
No action result
      ≠
Action completed but grasp success=False
      ≠
Physical object later slipping during lift
```

The program should only continue to LIFT when:

```python
result.success is True
```

---

## 10. Recommended robust fixed-grasp sequence

The fixed-position baseline should use this sequence:

```text
HOME
 ↓
PRE_GRASP
 ↓
OPEN GRIPPER TO ~0.080 m
 ↓
MOVE TO GRASP_POSE
 ↓
KEEP GRIPPER APPROXIMATELY VERTICAL
 ↓
EXECUTE GRASP ACTION
   width = 0.045 m
   speed = 0.02 m/s
   force = 10 N
   inner = 0.010 m
   outer = 0.010 m
 ↓
CHECK result.success
 ↙                 ↘
TRUE               FALSE
 ↓                   ↓
LIFT             DO NOT LIFT
```

This is safer and easier to debug than lifting regardless of grasp confirmation.

---

## 11. What to do when the cube is slightly away from the gripper center

For the current fixed-position stage:

1. Open the gripper fully before descending.
2. Keep the grasp orientation vertical.
3. Make sure the GRASP pose gives both fingers enough side overlap with the cube.
4. Avoid grasping only the top edge or corner of the cube.
5. Use a reasonable epsilon rather than an extremely strict one.
6. If the grasp fails, inspect `current_width` before changing parameters.
7. Re-center or adjust the saved GRASP pose if one finger repeatedly contacts first.

A useful physical target is:

```text
        |         |
        |   [■]   |
        |   [■]   |
        |   [■]   |
            ↑
       cube inside
       finger region
```

rather than:

```text
        |         |
        |       [■]
        |       [■]
                 ↑
             edge contact
```

The second configuration has much lower grasp tolerance.

---

## 12. Recommended debugging order

When the program reports:

```text
The gripper did not confirm a grasp.
```

debug in this order:

```text
1. Confirm gripper was fully open before descent
        ↓
2. Check cube XY placement
        ↓
3. Check vertical gripper orientation
        ↓
4. Check grasp Z height / finger overlap
        ↓
5. Check whether one finger contacts first
        ↓
6. Read feedback current_width
        ↓
7. Compare current_width with width ± epsilon
        ↓
8. Read result.success and result.error
        ↓
9. Only then tune epsilon / width
        ↓
10. Tune force only if physical holding force is actually insufficient
```

Do not begin by dramatically increasing force.

---

## 13. Rebuild and source after code changes

After changing the Python node:

```bash
cd /workspace/ros2_ws

colcon build \
  --packages-select fr3_vision_sorting \
  --symlink-install

source install/setup.bash
```

Then run:

```bash
ros2 run fr3_vision_sorting fixed_grasp_demo
```

This prevents testing an older installed copy of the Python executable by mistake.

---

## 14. From fixed grasping to vision-guided grasping

The fixed-position grasp is only the baseline.

A vision-guided system should eventually remove the requirement that the cube be manually placed at exactly the same XY location every time.

The planned pipeline is:

```text
RealSense detects cube
        ↓
Estimate cube center / pose
        ↓
Transform camera coordinates to robot frame
        ↓
Correct robot XY position
        ↓
Move above cube
        ↓
Open gripper
        ↓
Vertical descent
        ↓
Grasp
        ↓
Check result.success
        ↓
Lift and sort
```

This is the long-term solution for a cube that is slightly away from the previously recorded fixed grasp center.

---

## 15. Suggested repeatability experiment

Before moving fully to vision guidance, measure the physical tolerance of the fixed grasp.

For example, intentionally shift the cube by:

```text
0 mm
2 mm
5 mm
10 mm
15 mm
```

in X and Y and repeat each grasp several times.

Record:

- XY offset;
- `current_width`;
- `result.success`;
- whether the cube survives LIFT;
- whether the cube rotates or slips.

This produces a useful engineering metric:

```text
XY placement error
        ↓
Grasp success rate
```

The goal for the fixed-position baseline should be repeatability, not a single successful grasp. A practical milestone is approximately 9 successful grasps out of 10 under the same placement conditions before moving on to vision-guided correction.

---

## 16. Final lessons learned

The most important lessons from this stage are:

```text
Correct grasp pose
      +
Vertical approach
      +
Fully open gripper before descent
      +
Object reasonably centered
      +
Appropriate target width
      +
Reasonable epsilon
      +
Action feedback logging
      +
Check result.success before lift
      ↓
Repeatable FR3 grasp
```

The current validated grasp baseline is:

```text
Open width:       ~0.080 m
Grasp width:       0.045 m
Speed:             0.020 m/s
Force:            10.0 N
Inner epsilon:     0.010 m
Outer epsilon:     0.010 m
Approach:          approximately vertical
Lift condition:    only after result.success == True
```

The biggest lesson is that **a grasp is a geometry-and-contact problem first, and a parameter-tuning problem second**.
