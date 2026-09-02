#!/usr/bin/env python3
"""Stage 4: fixed-position pick and place for the Franka FR3.

Shared Stage 3 poses:
    config/fixed_grasp/

New Stage 4 poses:
    config/fixed_pick_place/

Sequence:
    HOME -> PRE_GRASP -> OPEN -> GRASP -> CLOSE -> LIFT
         -> PRE_BIN -> BIN -> RELEASE -> POST_BIN -> HOME
"""

from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory

from fr3_vision_sorting.fixed_grasp_demo import (
    FixedGraspDemo,
    require_confirmation,
)
from fr3_vision_sorting.gripper_control import GripperController


# Parameters successfully tested during Stage 3.
GRASP_WIDTH = 0.039  # metres
GRASP_SPEED = 0.02  # metres per second
GRASP_FORCE = 10.0  # newtons


def add_bin_poses(demo: FixedGraspDemo) -> None:
    """Load Stage 4 bin poses from config/fixed_pick_place/."""
    config_dir = (
        Path(get_package_share_directory("fr3_vision_sorting"))
        / "config"
        / "fixed_pick_place"
    )

    bin_pose_files = {
        "PRE_BIN": "pre_bin_joint_state.yaml",
        "BIN": "bin_joint_state.yaml",
        "POST_BIN": "post_bin_joint_state.yaml",
    }

    for pose_name, filename in bin_pose_files.items():
        path = config_dir / filename
        demo.poses[pose_name] = demo.load_joint_state(path)

    demo.get_logger().info(
        f"Loaded Stage 4 bin poses from: {config_dir}"
    )


def run_motion(
    demo: FixedGraspDemo,
    pose_name: str,
    prompt: str,
) -> bool:
    """Ask for confirmation and execute one saved arm pose."""
    require_confirmation(prompt)
    return demo.move_to(pose_name)


def main(args=None) -> None:
    rclpy.init(args=args)

    demo = None
    gripper = None

    try:
        # FixedGraspDemo automatically loads the shared Stage 3 poses:
        # HOME, PRE_GRASP, GRASP and LIFT.
        demo = FixedGraspDemo()

        # Add the Stage 4 destination-container poses:
        # PRE_BIN, BIN and POST_BIN.
        add_bin_poses(demo)

        gripper = GripperController()

        print("\nStage 4 fixed-position pick-and-place")
        print("-------------------------------------")
        print(
            "HOME -> PRE_GRASP -> OPEN -> GRASP -> CLOSE -> LIFT\n"
            "     -> PRE_BIN -> BIN -> RELEASE -> POST_BIN -> HOME"
        )
        print("\nArm velocity and acceleration scaling: 0.03")
        print(
            f"Grasp parameters: width={GRASP_WIDTH:.3f} m, "
            f"speed={GRASP_SPEED:.3f} m/s, "
            f"force={GRASP_FORCE:.1f} N"
        )

        if not run_motion(
            demo,
            "HOME",
            "Clear the workspace and prepare the enable device.\n"
            "Move to HOME?",
        ):
            return

        if not run_motion(
            demo,
            "PRE_GRASP",
            "Move to PRE_GRASP above the object?",
        ):
            return

        require_confirmation(
            "Confirm that the fingers have enough clearance.\n"
            "Open the gripper?"
        )
        if not gripper.open_gripper():
            demo.get_logger().error("Failed to open the gripper.")
            return

        if not run_motion(
            demo,
            "GRASP",
            "Confirm the object is correctly positioned.\n"
            "Descend to the saved GRASP pose?",
        ):
            return

        require_confirmation(
            "Verify finger alignment, object width and table clearance.\n"
            "Close the gripper on the object?"
        )
        if not demo.close_on_object(
            width=GRASP_WIDTH,
            speed=GRASP_SPEED,
            force=GRASP_FORCE,
        ):
            return

        if not run_motion(
            demo,
            "LIFT",
            "The grasp was confirmed.\n"
            "Lift the object to the saved LIFT pose?",
        ):
            return

        if not run_motion(
            demo,
            "PRE_BIN",
            "Confirm that the object is stable.\n"
            "Move to PRE_BIN above the container?",
        ):
            return

        if not run_motion(
            demo,
            "BIN",
            "Verify alignment with the container.\n"
            "Descend to the BIN release pose?",
        ):
            return

        require_confirmation(
            "Confirm that the object is supported by or positioned safely "
            "above the container.\nRelease the object?"
        )
        if not gripper.open_gripper():
            demo.get_logger().error("Failed to release the object.")
            return

        if not run_motion(
            demo,
            "POST_BIN",
            "The object was released.\n"
            "Retreat vertically to POST_BIN?",
        ):
            return

        if not run_motion(
            demo,
            "HOME",
            "Confirm that the gripper is clear of the container.\n"
            "Return to HOME?",
        ):
            return

        demo.get_logger().info(
            "Stage 4 fixed-position pick and place completed successfully."
        )

    except (KeyboardInterrupt, FileNotFoundError, ValueError) as error:
        if demo is not None:
            demo.get_logger().warning(f"Stopped safely: {error}")
        else:
            print(f"Stopped safely: {error}")

    except Exception as error:
        if demo is not None:
            demo.get_logger().error(f"Unexpected error: {error}")
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
