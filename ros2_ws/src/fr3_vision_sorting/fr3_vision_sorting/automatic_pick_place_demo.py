#!/usr/bin/env python3
"""Automatic fixed-position pick-and-place for the real Franka FR3.

The operator confirms once before starting. After confirmation, the program
runs one complete cycle automatically:

HOME -> PRE_GRASP -> OPEN -> GRASP -> CLOSE -> LIFT
     -> PRE_BIN -> BIN -> RELEASE -> POST_BIN -> HOME

A trained operator must remain beside the robot with the enabling device and
emergency stop accessible.
"""

import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory

from fr3_vision_sorting.fixed_grasp_demo import FixedGraspDemo
from fr3_vision_sorting.gripper_control import GripperController


# Grasp parameters validated for the current cube.
GRASP_WIDTH = 0.045
GRASP_SPEED = 0.02
GRASP_FORCE = 10.0

# Pause after every successful action.
ACTION_PAUSE_SECONDS = 2.0


def add_bin_poses(demo: FixedGraspDemo) -> None:
    """Load Stage 4 bin poses."""
    config_dir = (
        Path(get_package_share_directory("fr3_vision_sorting"))
        / "config"
        / "fixed_pick_place"
    )

    pose_files = {
        "PRE_BIN": "pre_bin_joint_state.yaml",
        "BIN": "bin_joint_state.yaml",
        "POST_BIN": "post_bin_joint_state.yaml",
    }

    for pose_name, filename in pose_files.items():
        demo.poses[pose_name] = demo.load_joint_state(
            config_dir / filename
        )

    demo.get_logger().info(
        f"Loaded bin poses from {config_dir}"
    )


def pause_after_action() -> None:
    """Pause briefly so the robot and object can settle."""
    time.sleep(ACTION_PAUSE_SECONDS)


def move_or_stop(
    demo: FixedGraspDemo,
    pose_name: str,
) -> bool:
    """Execute one pose and stop the sequence if it fails."""
    demo.get_logger().info(f"Automatic step: {pose_name}")

    if not demo.move_to(pose_name):
        demo.get_logger().error(
            f"Automatic sequence stopped at {pose_name}."
        )
        return False

    pause_after_action()
    return True


def open_or_stop(
    demo: FixedGraspDemo,
    gripper: GripperController,
    action_name: str,
) -> bool:
    """Open the gripper and stop if the action fails."""
    demo.get_logger().info(f"Automatic step: {action_name}")

    if not gripper.open_gripper():
        demo.get_logger().error(
            f"Automatic sequence stopped during {action_name}."
        )
        return False

    pause_after_action()
    return True


def confirm_start() -> bool:
    """Require one explicit confirmation before automatic motion."""
    print("\nAutomatic Stage 4 cycle")
    print("-----------------------")
    print(
        "HOME -> PRE_GRASP -> OPEN -> GRASP -> CLOSE -> LIFT\n"
        "     -> PRE_BIN -> BIN -> RELEASE -> POST_BIN -> HOME"
    )
    print("\nBefore starting, confirm:")
    print("- The cube is at the recorded pickup position.")
    print("- The container is at the recorded bin position.")
    print("- The workspace is clear.")
    print("- Franka Desk and FCI are ready.")
    print("- The emergency stop and enabling device are accessible.")
    print("- An operator will remain beside the robot.")
    print("- Only one cycle will run.")

    answer = input(
        "\nType START AUTOMATIC CYCLE to begin: "
    ).strip()

    return answer == "START AUTOMATIC CYCLE"


def run_automatic_cycle(
    demo: FixedGraspDemo,
    gripper: GripperController,
) -> bool:
    """Run exactly one automatic pick-and-place cycle."""

    if not move_or_stop(demo, "HOME"):
        return False

    if not move_or_stop(demo, "PRE_GRASP"):
        return False

    if not open_or_stop(demo, gripper, "OPEN"):
        return False

    if not move_or_stop(demo, "GRASP"):
        return False

    demo.get_logger().info("Automatic step: CLOSE")

    if not demo.close_on_object(
        width=GRASP_WIDTH,
        speed=GRASP_SPEED,
        force=GRASP_FORCE,
    ):
        demo.get_logger().error(
            "Grasp was not confirmed. Automatic sequence stopped."
        )
        return False

    pause_after_action()

    if not move_or_stop(demo, "LIFT"):
        return False

    if not move_or_stop(demo, "PRE_BIN"):
        return False

    if not move_or_stop(demo, "BIN"):
        return False

    if not open_or_stop(demo, gripper, "RELEASE"):
        return False

    if not move_or_stop(demo, "POST_BIN"):
        return False

    if not move_or_stop(demo, "HOME"):
        return False

    demo.get_logger().info(
        "Automatic pick-and-place cycle completed successfully."
    )
    return True


def main(args=None) -> None:
    rclpy.init(args=args)

    demo = None
    gripper = None

    try:
        demo = FixedGraspDemo()
        add_bin_poses(demo)

        gripper = GripperController()

        if not confirm_start():
            demo.get_logger().warning(
                "Automatic cycle cancelled by the operator."
            )
            return

        demo.get_logger().warning(
            "Automatic motion is starting. Remain beside the emergency stop."
        )

        run_automatic_cycle(demo, gripper)

    except KeyboardInterrupt:
        if demo is not None:
            demo.get_logger().warning(
                "Automatic cycle interrupted by the operator."
            )

    except (FileNotFoundError, ValueError) as error:
        if demo is not None:
            demo.get_logger().error(f"Configuration error: {error}")
        else:
            print(f"Configuration error: {error}")

    except Exception as error:
        if demo is not None:
            demo.get_logger().error(
                f"Unexpected error; motion stopped: {error}"
            )
        else:
            print(f"Unexpected error: {error}")

    finally:
        if gripper is not None:
            gripper.destroy_node()

        if demo is not None:
            demo.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
