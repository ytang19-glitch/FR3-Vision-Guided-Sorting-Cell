# Vision Pick-and-Place Execution Parameters

This document explains why the following ROS 2 command is physically and logically consistent for the FR3 vision-based pick-and-place experiment:

```bash
ros2 run fr3_vision_sorting vision_pick_place_demo --ros-args \
  -p tcp_to_fingertip:=0.056 \
  -p fingertip_clearance:=0.020 \
  -p grasp_offset:=0.080 \
  -p preview_only:=false \
  -p geometry_confirmed:=true \
  -p stop_after_pre_grasp:=false
```

All distance parameters are expressed in metres.

## Parameter meanings

| Parameter | Value | Meaning |
|---|---:|---|
| `tcp_to_fingertip` | `0.056 m` | Measured distance from the robot TCP to the fingertip. |
| `fingertip_clearance` | `0.020 m` | Required minimum distance between the fingertip and the table. |
| `grasp_offset` | `0.080 m` | Vertical offset from the detected object reference position to the grasp TCP position. |
| `preview_only` | `false` | Allows the node to plan and execute robot and gripper motion. |
| `geometry_confirmed` | `true` | Confirms that the table, gripper geometry, TCP offset, tool orientation, and scene have been checked. |
| `stop_after_pre_grasp` | `false` | Allows the robot to continue from pre-grasp through grasp, lift, placement, release, and retreat. |

## Physical clearance calculation

The measured table height is:

```text
table_top_z = 0.130 m
```

The minimum safe TCP height is calculated as:

```text
minimum_grasp_z = table_top_z
                  + fingertip_clearance
                  + tcp_to_fingertip

minimum_grasp_z = 0.130 + 0.020 + 0.056
                = 0.206 m
```

Therefore, the grasp TCP should remain above `0.206 m`.

## Grasp-position calculation

For the current detected object reference position:

```text
object_z = 0.134 m
```

The node calculates the grasp TCP height as:

```text
grasp_z = object_z + grasp_offset
        = 0.134 + 0.080
        = 0.214 m
```

The grasp position is safe because:

```text
grasp_z = 0.214 m > minimum_grasp_z = 0.206 m
```

The corresponding fingertip height is approximately:

```text
fingertip_z = grasp_z - tcp_to_fingertip
            = 0.214 - 0.056
            = 0.158 m
```

The actual fingertip clearance above the table is therefore:

```text
actual_clearance = fingertip_z - table_top_z
                 = 0.158 - 0.130
                 = 0.028 m
                 = 28 mm
```

The requested clearance is `20 mm`, so the current configuration provides approximately:

```text
additional_margin = 28 mm - 20 mm = 8 mm
```

## Expected execution sequence

With `preview_only=false` and `stop_after_pre_grasp=false`, the node is expected to execute:

1. Move to the pre-grasp pose.
2. Descend to the grasp pose.
3. Close the gripper.
4. Lift the object.
5. Move to the placement pose.
6. Open the gripper to release the object.
7. Retreat from the placement area.

For the current recorded target, the key vertical positions are approximately:

```text
PRE_GRASP z = 0.254 m
GRASP     z = 0.214 m
LIFT      z = 0.364 m
```

## Important distinction

`grasp_offset` and `fingertip_clearance` are different quantities:

- `grasp_offset` determines where the TCP is placed relative to the detected object.
- `fingertip_clearance` limits how close the fingertips may approach the table.
- `tcp_to_fingertip` connects the TCP coordinate to the physical fingertip location.

The grasp offset must therefore be checked against the fingertip-clearance constraint rather than selected independently.

## Safety check before execution

Before running the command on the physical FR3, verify:

- the table height is still `0.130 m`;
- the TCP-to-fingertip distance is still `0.056 m`;
- the gripper orientation matches the assumed coordinate frame;
- the object and target are inside the calibrated workspace;
- no obstacle lies inside the planned robot or gripper path;
- the emergency stop is accessible.

Because `preview_only=false` and `geometry_confirmed=true`, this command enables real robot execution rather than only displaying the planned coordinates.
