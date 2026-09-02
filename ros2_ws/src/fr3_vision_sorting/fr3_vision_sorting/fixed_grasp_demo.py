#!/usr/bin/env python3
"""Stage 3: fixed-position grasping with an FR3 and Franka Hand.

Sequence:
    HOME
    -> PRE_GRASP
    -> OPEN
    -> GRASP_POSE
    -> CLOSE_ON_OBJECT
    -> LIFT
    -> HOME

Arm motion is planned and executed through MoveIt's /move_action server.
The gripper is opened using GripperController and closed using Franka's
contact-aware Grasp action.
"""

from pathlib import Path
from typing import Dict, List

import rclpy
import yaml

from ament_index_python.packages import get_package_share_directory
from franka_msgs.action import Grasp
from franka_msgs.msg import GraspEpsilon
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
)
from rclpy.action import ActionClient
from rclpy.node import Node

from fr3_vision_sorting.gripper_control import GripperController


ARM_JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]


class FixedGraspDemo(Node):
    """Execute fixed arm poses and a contact-aware Franka grasp."""

    def __init__(self) -> None:
        super().__init__("fixed_grasp_demo")

        # --------------------------------------------------------------
        # Action clients
        # --------------------------------------------------------------

        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            "/move_action",
        )

        self.grasp_client = ActionClient(
            self,
            Grasp,
            "/franka_gripper/grasp",
        )

        # --------------------------------------------------------------
        # Wait for servers
        # --------------------------------------------------------------

        self.get_logger().info("Waiting for MoveIt /move_action...")

        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                "MoveIt /move_action server is not available."
            )

        self.get_logger().info("MoveIt action server is ready.")

        self.get_logger().info(
            "Waiting for /franka_gripper/grasp..."
        )

        if not self.grasp_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(
                "Franka grasp action server is not available."
            )

        self.get_logger().info(
            "Franka grasp action server is ready."
        )

        # --------------------------------------------------------------
        # Load saved poses
        # --------------------------------------------------------------

        config_dir = (
            Path(
                get_package_share_directory(
                    "fr3_vision_sorting"
                )
            )
            / "config"
            / "fixed_grasp"
        )

        self.poses = {
            "HOME": self.load_joint_state(
                config_dir / "home_joint_state.yaml"
            ),
            "PRE_GRASP": self.load_joint_state(
                config_dir / "pre_grasp_joint_state.yaml"
            ),
            "GRASP": self.load_joint_state(
                config_dir / "grasp_joint_state.yaml"
            ),
            "LIFT": self.load_joint_state(
                config_dir / "lift_joint_state.yaml"
            ),
        }

    # ==================================================================
    # Joint-state loading
    # ==================================================================

    def load_joint_state(self, path: Path) -> List[float]:
        """Load and reorder the seven FR3 joints from a YAML file."""

        if not path.is_file():
            raise FileNotFoundError(
                f"Pose file not found: {path}"
            )

        with path.open("r", encoding="utf-8") as stream:
            documents = [
                document
                for document in yaml.safe_load_all(stream)
                if isinstance(document, dict)
            ]

        if not documents:
            raise ValueError(
                f"No valid JointState found in {path}"
            )

        data = documents[-1]

        names = data.get("name", [])
        positions = data.get("position", [])

        if len(names) != len(positions):
            raise ValueError(
                f"name/position length mismatch in {path}"
            )

        joint_map: Dict[str, float] = dict(
            zip(names, positions)
        )

        missing = [
            name
            for name in ARM_JOINTS
            if name not in joint_map
        ]

        if missing:
            raise ValueError(
                f"Missing joints in {path}: {missing}"
            )

        return [
            float(joint_map[name])
            for name in ARM_JOINTS
        ]

    # ==================================================================
    # Arm motion
    # ==================================================================

    def move_to(self, pose_name: str) -> bool:
        """Plan and execute a saved joint-space pose."""

        if pose_name not in self.poses:
            self.get_logger().error(
                f"Unknown pose: {pose_name}"
            )
            return False

        positions = self.poses[pose_name]

        constraints = Constraints(
            name=pose_name.lower()
        )

        for joint_name, position in zip(
            ARM_JOINTS,
            positions,
        ):
            constraint = JointConstraint()

            constraint.joint_name = joint_name
            constraint.position = position

            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01

            constraint.weight = 1.0

            constraints.joint_constraints.append(
                constraint
            )

        goal = MoveGroup.Goal()

        goal.request.group_name = "fr3_arm"

        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0

        goal.request.max_velocity_scaling_factor = 0.03
        goal.request.max_acceleration_scaling_factor = 0.03

        goal.request.goal_constraints = [
            constraints
        ]

        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.get_logger().info(
            f"Planning and executing {pose_name}..."
        )

        send_future = (
            self.move_group_client.send_goal_async(goal)
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(
                f"No goal handle returned for {pose_name}."
            )
            return False

        if not goal_handle.accepted:
            self.get_logger().error(
                f"{pose_name} goal was rejected."
            )
            return False

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error(
                f"No result returned for {pose_name}."
            )
            return False

        error_code = (
            wrapped_result.result.error_code.val
        )

        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"{pose_name} failed. "
                f"MoveIt error code: {error_code}"
            )
            return False

        self.get_logger().info(
            f"Reached {pose_name}."
        )

        return True

    # ==================================================================
    # Gripper feedback
    # ==================================================================

    def grasp_feedback_callback(
        self,
        feedback_msg,
    ) -> None:
        """Print current finger opening during grasp."""

        current_width = (
            feedback_msg.feedback.current_width
        )

        self.get_logger().info(
            f"Gripper width: "
            f"{current_width:.5f} m"
        )

    # ==================================================================
    # Contact-aware grasp
    # ==================================================================

    def close_on_object(
        self,
        width: float = 0.045,
        speed: float = 0.02,
        force: float = 10.0,
        inner_epsilon: float = 0.010,
        outer_epsilon: float = 0.010,
    ) -> bool:
        """Close fingers using Franka's Grasp action."""

        goal = Grasp.Goal()

        goal.width = width
        goal.speed = speed
        goal.force = force

        goal.epsilon = GraspEpsilon(
            inner=inner_epsilon,
            outer=outer_epsilon,
        )

        self.get_logger().info(
            "Sending grasp goal:"
        )

        self.get_logger().info(
            f"  target width = {width:.3f} m"
        )

        self.get_logger().info(
            f"  speed        = {speed:.3f} m/s"
        )

        self.get_logger().info(
            f"  force        = {force:.1f} N"
        )

        self.get_logger().info(
            f"  inner epsilon= {inner_epsilon:.3f} m"
        )

        self.get_logger().info(
            f"  outer epsilon= {outer_epsilon:.3f} m"
        )

        # --------------------------------------------------------------
        # Send Grasp goal
        # --------------------------------------------------------------

        send_future = self.grasp_client.send_goal_async(
            goal,
            feedback_callback=self.grasp_feedback_callback,
        )

        rclpy.spin_until_future_complete(
            self,
            send_future,
        )

        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(
                "No grasp goal handle returned."
            )
            return False

        if not goal_handle.accepted:
            self.get_logger().error(
                "Grasp goal was rejected."
            )
            return False

        self.get_logger().info(
            "Grasp goal accepted."
        )

        # --------------------------------------------------------------
        # Wait for result
        # --------------------------------------------------------------

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        wrapped_result = result_future.result()

        if wrapped_result is None:
            self.get_logger().error(
                "No grasp result received."
            )
            return False

        result = wrapped_result.result

        # --------------------------------------------------------------
        # Print Franka result
        # --------------------------------------------------------------

        self.get_logger().info(
            f"Grasp result: "
            f"success={result.success}, "
            f"error='{result.error}'"
        )

        if not result.success:
            self.get_logger().error(
                "The gripper did not confirm a grasp."
            )

            self.get_logger().error(
                "Possible causes: cube off-center, "
                "incorrect grasp height, object moved, "
                "or final finger width outside epsilon."
            )

            return False

        self.get_logger().info(
            "Object grasp confirmed."
        )

        return True


# ======================================================================
# User confirmation
# ======================================================================

def require_confirmation(message: str) -> None:
    """Require explicit confirmation before real robot motion."""

    answer = input(
        f"\n{message}\n"
        "Type EXECUTE to continue: "
    ).strip()

    if answer != "EXECUTE":
        raise KeyboardInterrupt(
            "Operation cancelled by user."
        )


# ======================================================================
# Main
# ======================================================================

def main(args=None) -> None:

    rclpy.init(args=args)

    demo = None
    gripper = None

    try:

        demo = FixedGraspDemo()
        gripper = GripperController()

        print("\nStage 3 sequence:")

        print(
            "HOME -> PRE_GRASP -> OPEN -> "
            "GRASP_POSE -> CLOSE -> LIFT -> HOME"
        )

        print(
            "Velocity and acceleration scaling: 0.03"
        )

        # --------------------------------------------------------------
        # HOME
        # --------------------------------------------------------------

        require_confirmation(
            "Clear the workspace. Move to HOME?"
        )

        if not demo.move_to("HOME"):
            return

        # --------------------------------------------------------------
        # PRE-GRASP
        # --------------------------------------------------------------

        require_confirmation(
            "Move to PRE_GRASP?"
        )

        if not demo.move_to("PRE_GRASP"):
            return

        # --------------------------------------------------------------
        # OPEN
        # --------------------------------------------------------------

        require_confirmation(
            "Open the gripper fully?"
        )

        if not gripper.open_gripper():
            demo.get_logger().error(
                "Failed to open gripper."
            )
            return

        # --------------------------------------------------------------
        # DESCEND
        # --------------------------------------------------------------

        require_confirmation(
            "Descend vertically to the saved GRASP pose?"
        )

        if not demo.move_to("GRASP"):
            return

        # --------------------------------------------------------------
        # CONTACT-AWARE GRASP
        # --------------------------------------------------------------

        require_confirmation(
            "Close on the object? "
            "Check cube position and finger alignment."
        )

        grasp_success = demo.close_on_object(
            width=0.045,
            speed=0.02,
            force=10.0,
            inner_epsilon=0.010,
            outer_epsilon=0.010,
        )

        if not grasp_success:
            demo.get_logger().error(
                "Grasp failed. "
                "The robot will NOT lift the object."
            )
            return

        # --------------------------------------------------------------
        # LIFT
        # --------------------------------------------------------------

        require_confirmation(
            "Grasp confirmed. Lift the object?"
        )

        if not demo.move_to("LIFT"):
            return

        # --------------------------------------------------------------
        # HOME
        # --------------------------------------------------------------

        require_confirmation(
            "Return to HOME while holding the object?"
        )

        if not demo.move_to("HOME"):
            return

        demo.get_logger().info(
            "Stage 3 sequence completed successfully."
        )

    except (
        KeyboardInterrupt,
        FileNotFoundError,
        ValueError,
        RuntimeError,
    ) as error:

        if demo is not None:
            demo.get_logger().warning(
                f"Stopped safely: {error}"
            )
        else:
            print(
                f"Stopped: {error}"
            )

    finally:

        if gripper is not None:
            gripper.destroy_node()

        if demo is not None:
            demo.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
