#!/usr/bin/env python3
"""Vertical Cartesian revision V3 (fresh TF wait); NOT hardware-tested.

Requires existing FixedGraspDemo, GripperController and saved YAML dependencies.
Horizontal table only. Configure measured table center/height/size and the
actual downward offset from fr3_hand_tcp to the lowest finger geometry.
The default yaw approximately preserves the user's previous TCP X-axis heading;
roll/pitch are levelled mathematically, not validated on the physical gripper.
Preview is default. Motion additionally requires geometry_confirmed:=true;
stop_after_pre_grasp:=true is default. No automatic retry.
The held object is not attached to the planning scene by this program;
transport still requires appropriate payload/collision-scene configuration.
Inherited action methods may block; Ctrl+C is not guaranteed action cancellation.

One-shot, manually confirmed vision pick-and-place for the FR3.

Uses the existing FixedGraspDemo, GripperController and saved bin poses.
The detector must already publish the intended object on /object_point_camera.
The target_color parameter is a label, NOT a filter for PointStamped messages.

Default: preview_only:=true prints targets without arm/gripper commands.
After checking the setup, preview_only:=false additionally requires START.
Keep the cube, camera and robot stationary while the frozen target is reviewed.
These offsets and saved poses are inherited from the supplied program; they
are not a validation of calibration, reachability or collision clearance.

Important control-flow fixes:
  * START no longer hits an unconditional return.
  * The TCP orientation comes from base -> TCP, not base -> camera.
  * Only one thread spins ROS callbacks; motion runs outside callbacks.
  * Detection freezes at the requested count. There is no automatic retry.
"""

import copy
import math
import time
from typing import Optional

import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PointStamped, Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_geometry_msgs import do_transform_point

from fr3_vision_sorting.automatic_pick_place_demo import add_bin_poses
from fr3_vision_sorting.fixed_grasp_demo import FixedGraspDemo
from fr3_vision_sorting.gripper_control import GripperController


class VisionPickPlaceDemo(FixedGraspDemo):
    """Collect one stable target and attempt at most one confirmed cycle."""

    def __init__(self):
        super().__init__()

        # This listener uses the same executor as this node. Do not add a
        # background spin thread while the motion helpers also spin the node.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self.execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )

        defaults = {
            "target_color": "R",
            "object_topic": "/object_point_camera",
            "base_frame": "fr3_link0",
            "tcp_frame": "fr3_hand_tcp",
            "pre_grasp_offset": 0.12,
            "grasp_offset": 0.025,
            "lift_offset": 0.15,
            "stable_detections": 10,
            "stable_distance": 0.01,
            "max_detection_age": 1.0,
            "preview_only": True,
            # Replace these with the validated vertical tool orientation.
            "grasp_orientation_qx": 0.0,
            "grasp_orientation_qy": 1.0,
            "grasp_orientation_qz": 0.0,
            "grasp_orientation_qw": 0.0,
            # Horizontal tabletop only: base +Z must be its upward normal.
            # NaN deliberately requires measurements, not guessed geometry.
            "table_top_z": 0.13,
            "table_center_x": 0.0,
            "table_center_y": 0.0,
            "geometry_confirmed": False,
            "stop_after_pre_grasp": True,
            "cartesian_velocity_scale": 0.03,
            "cartesian_acceleration_scale": 0.03,
            "max_joint_step": 0.10,
            "table_thickness": 0.04,
            "table_size_x": 1.22,
            "table_size_y": 0.99,
            "tcp_to_fingertip": 0.056,
            "fingertip_clearance": 0.02,
        }
        for name, default in defaults.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, default)

        self.target_color = str(self.get_parameter("target_color").value)
        self.object_topic = str(self.get_parameter("object_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tcp_frame = str(self.get_parameter("tcp_frame").value)
        self.pre_grasp_offset = float(self.get_parameter("pre_grasp_offset").value)
        self.grasp_offset = float(self.get_parameter("grasp_offset").value)
        self.lift_offset = float(self.get_parameter("lift_offset").value)
        self.required_detections = int(self.get_parameter("stable_detections").value)
        self.stable_distance = float(self.get_parameter("stable_distance").value)
        self.max_detection_age = float(self.get_parameter("max_detection_age").value)
        self.preview_only = bool(self.get_parameter("preview_only").value)
        self.grasp_orientation = self._read_grasp_orientation()
        for key in ("table_center_x", "table_center_y", "cartesian_velocity_scale",
                    "cartesian_acceleration_scale", "max_joint_step"):
            setattr(self, key, float(self.get_parameter(key).value))
        self.geometry_confirmed = bool(self.get_parameter("geometry_confirmed").value)
        self.stop_after_pre_grasp = bool(self.get_parameter("stop_after_pre_grasp").value)
        for scale in (self.cartesian_velocity_scale, self.cartesian_acceleration_scale):
            if not math.isfinite(scale) or not 0.0 < scale <= 0.1:
                raise ValueError("Cartesian scaling must be in (0, 0.1]")
        if not math.isfinite(self.max_joint_step) or self.max_joint_step <= 0:
            raise ValueError("max_joint_step must be positive")
        self.table_top_z = float(self.get_parameter("table_top_z").value)
        self.table_thickness = float(self.get_parameter("table_thickness").value)
        self.table_size_x = float(self.get_parameter("table_size_x").value)
        self.table_size_y = float(self.get_parameter("table_size_y").value)
        self.tcp_to_fingertip = float(
            self.get_parameter("tcp_to_fingertip").value
        )
        self.fingertip_clearance = float(
            self.get_parameter("fingertip_clearance").value
        )

        numeric = (
            self.pre_grasp_offset, self.grasp_offset, self.lift_offset,
            self.stable_distance, self.max_detection_age,
            self.table_thickness, self.fingertip_clearance,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("Offsets and detection limits must be finite")
        if self.required_detections < 1 or self.stable_distance <= 0.0:
            raise ValueError("stable_detections and stable_distance must be positive")
        if self.max_detection_age <= 0.0:
            raise ValueError("max_detection_age must be positive")
        if self.pre_grasp_offset <= self.grasp_offset or self.lift_offset <= 0.0:
            raise ValueError("PRE_GRASP must be above GRASP; lift_offset must be positive")
        if not all(value > 0.0 for value in (
            self.table_thickness, self.fingertip_clearance,
        )):
            raise ValueError("Table and fingertip safety dimensions must be positive")

        # Retain your existing HOME, PRE_BIN, BIN and POST_BIN destinations.
        add_bin_poses(self)

        self.latest_point: Optional[PointStamped] = None
        self.last_xyz: Optional[tuple[float, float, float]] = None
        self.stable_count = 0
        self.running = False
        self.last_stamp_ns = None

        self.object_sub = self.create_subscription(
            PointStamped, self.object_topic, self.object_callback, 10,
        )
        self.sequence_timer = self.create_timer(0.2, self.try_start_sequence)
        self.get_logger().info(
            f"Vision update: VERTICAL_CARTESIAN_V3_TF_REFRESH. preview_only={self.preview_only}. "
            f"Waiting for {self.required_detections} nearby {self.target_color} "
            f"detections on {self.object_topic}"
        )

    def _read_grasp_orientation(self):
        """Return the configured, normalized vertical grasp orientation."""
        values = tuple(
            float(self.get_parameter(name).value)
            for name in (
                "grasp_orientation_qx", "grasp_orientation_qy",
                "grasp_orientation_qz", "grasp_orientation_qw",
            )
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Grasp orientation must be finite")
        norm = math.sqrt(sum(value * value for value in values))
        if norm < 1e-9:
            raise ValueError("Grasp orientation quaternion has zero length")
        quaternion = Pose().orientation
        quaternion.x, quaternion.y, quaternion.z, quaternion.w = (
            value / norm for value in values
        )
        return quaternion

    def reset_detection_window(self):
        """Reset acquisition only; never called to retry a completed cycle."""
        self.latest_point = None
        self.last_xyz = None
        self.last_stamp_ns = None
        self.stable_count = 0

    def object_callback(self, msg: PointStamped):
        """Accept fresh detections only until a target has been frozen."""
        if self.running:
            return  # No Detection 11/10 while confirming or moving.

        xyz = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
        if not msg.header.frame_id or not all(math.isfinite(v) for v in xyz) or xyz[2] <= 0.0:
            self.reset_detection_window()
            self.get_logger().warning("Ignoring invalid point, depth or frame_id")
            return

        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if stamp_ns <= 0 or age < 0.0 or age > self.max_detection_age:
            self.reset_detection_window()
            self.get_logger().warning(f"Ignoring stale/invalid detection: age={age:.3f}s")
            return
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return  # Repeated messages do not count as new measurements.

        same_frame = (
            self.latest_point is not None
            and self.latest_point.header.frame_id == msg.header.frame_id
        )
        small_gap = (
            self.last_stamp_ns is not None
            and (stamp_ns - self.last_stamp_ns) / 1e9 <= self.max_detection_age
        )
        if self.last_xyz is None or not same_frame or not small_gap:
            self.stable_count = 1
        else:
            distance = math.dist(xyz, self.last_xyz)
            self.stable_count = self.stable_count + 1 if distance <= self.stable_distance else 1

        self.latest_point = copy.deepcopy(msg)
        self.last_xyz = xyz
        self.last_stamp_ns = stamp_ns
        self.get_logger().info(
            f"Detection {self.stable_count}/{self.required_detections}: "
            f"camera point = [{xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}]"
        )
        # Freeze immediately at 10/10, not at a later timer tick.
        self.try_start_sequence()

    def try_start_sequence(self):
        """Mark the target ready; main() starts the cycle outside callbacks."""
        if self.running or self.latest_point is None:
            return
        if self.stable_count < self.required_detections:
            return

        self.running = True
        self.sequence_timer.cancel()
        self.get_logger().info(
            "Target frozen. This node is ignoring further detections. "
            "Keep the cube, camera and robot stationary while reviewing it."
        )

    def transform_object_point(self) -> Optional[tuple[float, float, float]]:
        """Use the camera transform at the frozen measurement timestamp."""
        if self.latest_point is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.latest_point.header.frame_id,
                Time.from_msg(self.latest_point.header.stamp),
            )
            # Explicit helper avoids generic PointStamped registration errors.
            transformed = do_transform_point(self.latest_point, transform)
            point = transformed.point
            xyz = (float(point.x), float(point.y), float(point.z))
            if not all(math.isfinite(value) for value in xyz):
                raise ValueError("Transformed point is not finite")
            return xyz
        except Exception as error:
            self.get_logger().error(f"Could not transform object point: {error}")
            return None

    def current_tcp_orientation(self):
        """Get the robot TCP orientation, NOT the camera orientation."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tcp_frame,  # Important: this must NOT be the camera frame.
                Time(),
            )
            quaternion = copy.deepcopy(transform.transform.rotation)
            values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            if not all(math.isfinite(value) for value in values):
                raise ValueError("TCP quaternion is not finite")
            norm = math.sqrt(sum(value * value for value in values))
            if norm < 1e-9:
                raise ValueError("TCP quaternion has zero length")
            quaternion.x /= norm
            quaternion.y /= norm
            quaternion.z /= norm
            quaternion.w /= norm
            return quaternion
        except Exception as error:
            self.get_logger().error(f"Could not read TCP orientation: {error}")
            return None

    def check_geometry(self, x, y):
        """Validate measured values; this is not physical calibration."""
        keys = ("table_top_z", "table_center_x", "table_center_y",
                "table_size_x", "table_size_y", "tcp_to_fingertip")
        missing = [key for key in keys if not math.isfinite(getattr(self, key))]
        if missing:
            self.get_logger().error("Set measured parameters: " + ", ".join(missing))
            return False
        if self.table_size_x <= 0 or self.table_size_y <= 0 or self.tcp_to_fingertip < 0:
            self.get_logger().error("Table sizes must be positive; fingertip offset nonnegative")
            return False
        if (abs(x - self.table_center_x) > self.table_size_x / 2 or
                abs(y - self.table_center_y) > self.table_size_y / 2):
            self.get_logger().error("Target is outside the measured tabletop rectangle")
            return False
        q = self.grasp_orientation
        # TCP +Z must be base -Z for this simplified fingertip-height model.
        axis_z = 1.0 - 2.0 * (q.x*q.x + q.y*q.y)
        if axis_z > -math.cos(math.radians(0.1)):
            self.get_logger().error("Configured orientation is not vertical in base frame")
            return False
        return True

    def check_cartesian_start(self, x, y, z, orientation):
        """Wait for fresh TF, then verify the actual Cartesian start pose.

        GripperController spins its own node during opening. This node must
        process queued TF messages afterward before deciding whether TF is stale.
        The bounded wait does not relax position/orientation checks.
        """
        try:
            deadline = time.monotonic() + 3.0
            tf = None
            last_error = "No fresh TCP transform"
            self.get_logger().info("Waiting for fresh TCP TF before Cartesian motion")
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                try:
                    candidate = self.tf_buffer.lookup_transform(
                        self.base_frame, self.tcp_frame, Time()
                    )
                    age = (
                        self.get_clock().now().nanoseconds
                        - Time.from_msg(candidate.header.stamp).nanoseconds
                    ) / 1e9
                    if 0.0 <= age <= 0.5:
                        tf = candidate
                        break
                    last_error = f"TCP TF age is {age:.3f}s"
                except tf2_ros.TransformException as error:
                    last_error = str(error)
            if tf is None:
                raise ValueError(f"Fresh TF unavailable within 3s: {last_error}")
            p, q = tf.transform.translation, tf.transform.rotation
            actual = (q.x, q.y, q.z, q.w)
            desired = (orientation.x, orientation.y, orientation.z, orientation.w)
            norm = math.sqrt(sum(v*v for v in actual))
            if not math.isfinite(norm) or norm < 1e-9:
                raise ValueError("Invalid actual orientation")
            dot = abs(sum(a*b for a, b in zip(actual, desired)) / norm)
            angle = 2 * math.acos(min(1.0, dot))
            distance = math.dist((x, y, z), (p.x, p.y, p.z))
            if not math.isfinite(distance) or distance > 0.004 or angle > math.radians(1):
                raise ValueError(f"Start error: {distance*1000:.2f} mm, "
                                 f"{math.degrees(angle):.2f} degrees")
            return True
        except Exception as error:
            self.get_logger().error(f"Cartesian start rejected: {error}")
            return False

    def validate_cartesian_trajectory(self, solution):
        """Reject empty, untimed or discontinuous joint paths before execution."""
        trajectory = solution.joint_trajectory
        points = trajectory.points
        expected = {f"fr3_joint{i}" for i in range(1, 8)}
        if set(trajectory.joint_names) != expected or len(points) < 2:
            self.get_logger().error("Invalid Cartesian joint names or empty path")
            return False
        previous_time = -1.0
        previous_positions = None
        for point in points:
            t = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            if (t < 0 or t <= previous_time or len(point.positions) != 7 or
                    not all(math.isfinite(v) for v in point.positions)):
                self.get_logger().error("Invalid Cartesian positions/timing; refusing execution")
                return False
            if previous_positions is not None and any(
                abs(a-b) > self.max_joint_step
                for a, b in zip(point.positions, previous_positions)
            ):
                self.get_logger().error("Cartesian joint jump detected; refusing execution")
                return False
            for values in (point.velocities, point.accelerations):
                if values and (len(values) != 7 or not all(math.isfinite(v) for v in values)):
                    self.get_logger().error("Invalid trajectory derivatives")
                    return False
            previous_time, previous_positions = t, point.positions
        return True

    def add_table_collision(self) -> bool:
        """Add the tabletop so Cartesian planning checks the full descent."""
        if not self.apply_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("MoveIt planning-scene service unavailable")
            return False

        table = CollisionObject()
        table.id = "vision_table"
        table.header.frame_id = self.base_frame
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            self.table_size_x,
            self.table_size_y,
            self.table_thickness,
        ]
        table.primitives.append(primitive)
        table_pose = Pose(
            position=Point(
                x=self.table_center_x,
                y=self.table_center_y,
                z=self.table_top_z - self.table_thickness / 2.0,
            )
        )
        table_pose.orientation.w = 1.0
        table.primitive_poses.append(table_pose)
        table.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(table)
        future = self.apply_scene_client.call_async(
            ApplyPlanningScene.Request(scene=scene)
        )
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or not response.success:
            self.get_logger().error("MoveIt rejected the tabletop collision object")
            return False
        return True

    def move_vertical_cartesian(self, start_z, grasp_z, x, y, orientation) -> bool:
        """Use the same checked orientation for descent AND lift.

        Requires a horizontal table, current TCP near the nominal start,
        supported speed fields, a complete timed path and small joint steps.
        A collision check is only as accurate as the model/planning scene.
        """
        if not self.check_cartesian_start(x, y, start_z, orientation):
            return False
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("MoveIt Cartesian-path service unavailable")
            return False
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveIt trajectory action unavailable")
            return False

        start = Pose(position=Point(x=float(x), y=float(y), z=float(start_z)))
        start.orientation = copy.deepcopy(orientation)
        goal = Pose(position=Point(x=float(x), y=float(y), z=float(grasp_z)))
        goal.orientation = copy.deepcopy(orientation)

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.group_name = "fr3_arm"
        request.start_state = RobotState()
        request.start_state.is_diff = True
        request.link_name = self.tcp_frame
        request.waypoints = [start, goal]
        request.max_step = 0.005
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        # Never silently execute with an unsupported/unspecified speed limit.
        if not all(hasattr(request, field) for field in (
            "max_velocity_scaling_factor", "max_acceleration_scaling_factor"
        )):
            self.get_logger().error(
                "Installed GetCartesianPath lacks speed scaling fields. "
                "Refusing execution; use a compatible retiming implementation."
            )
            return False
        request.max_velocity_scaling_factor = self.cartesian_velocity_scale
        request.max_acceleration_scaling_factor = self.cartesian_acceleration_scale

        future = self.cartesian_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error("No Cartesian-path response received")
            return False
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Cartesian descent failed with MoveIt code "
                f"{response.error_code.val}"
            )
            return False
        if not math.isfinite(response.fraction) or response.fraction < 1.0 - 1e-9:
            self.get_logger().error(
                f"Cartesian descent covers only {response.fraction:.1%}; refusing motion"
            )
            return False

        if not self.validate_cartesian_trajectory(response.solution):
            return False
        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory = response.solution
        send_future = self.execute_trajectory_client.send_goal_async(execute_goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Cartesian trajectory was rejected")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            return False
        success = wrapped_result.result.error_code.val == MoveItErrorCodes.SUCCESS
        if not success:
            self.get_logger().error(
                f"Cartesian descent execution failed with MoveIt code "
                f"{wrapped_result.result.error_code.val}"
            )
        return success

    def move_to_cartesian_pose(self, x, y, z, orientation, pose_name) -> bool:
        """Plan and execute a pose goal; called only from the main thread."""
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server unavailable; stopping")
            return False

        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = self.base_frame
        position.link_name = self.tcp_frame
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.002]  # 2 mm goal radius; actual pose checked before descent.

        region_pose = Pose(position=Point(x=float(x), y=float(y), z=float(z)))
        region_pose.orientation.w = 1.0  # A zero quaternion is invalid.
        position.constraint_region = BoundingVolume()
        position.constraint_region.primitives.append(primitive)
        position.constraint_region.primitive_poses.append(region_pose)
        position.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.base_frame
        orientation_constraint.link_name = self.tcp_frame
        orientation_constraint.orientation = orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation_constraint)

        goal = MoveGroup.Goal()
        goal.request.group_name = "fr3_arm"
        goal.request.start_state.is_diff = True
        goal.request.allowed_planning_time = 5.0
        goal.request.num_planning_attempts = 5
        goal.request.max_velocity_scaling_factor = 0.03
        goal.request.max_acceleration_scaling_factor = 0.03
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.get_logger().info(
            f"Planning {pose_name}: x={x:.3f}, y={y:.3f}, z={z:.3f}"
        )
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        if not rclpy.ok() or not future.done():
            return False
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{pose_name} goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        if not rclpy.ok() or not result_future.done():
            return False
        action_result = result_future.result()
        if action_result is None:
            self.get_logger().error(f"{pose_name}: no MoveIt result received")
            return False
        success = action_result.result.error_code.val == MoveItErrorCodes.SUCCESS
        if not success:
            self.get_logger().error(
                f"{pose_name} failed with MoveIt code {action_result.result.error_code.val}"
            )
        return success

    def execute_vision_sequence(self):
        """Preview, or execute one START-confirmed cycle with no auto-retry."""
        gripper = None
        try:
            object_position = self.transform_object_point()
            if object_position is None:
                return
            orientation = copy.deepcopy(self.grasp_orientation)

            x, y, z = object_position

            # Existing locally measured base-frame XY correction, applied once.
            x -= 0.012
            y -= 0.021
            
            # Do not claim a clearance check with unknown dimensions.
            if not self.check_geometry(x, y):
                return
            minimum_grasp_z = (
                self.table_top_z
                + self.fingertip_clearance
                + self.tcp_to_fingertip
            )
            grasp_z = z + self.grasp_offset
            if grasp_z < minimum_grasp_z:
                self.get_logger().error(
                    f"Requested GRASP z={grasp_z:.4f} is below the modelled "
                    f"minimum {minimum_grasp_z:.4f}. Recheck measured geometry "
                    "and contact height; target will not be silently raised."
                )
                return
            approach_height = self.pre_grasp_offset - self.grasp_offset
            pre_grasp_z = grasp_z + approach_height
            lift_z = grasp_z + self.lift_offset
            self.get_logger().info(
                f"Object after XY correction in {self.base_frame}: "
                f"x={x:.3f}, y={y:.3f}, z={z:.3f}"
            )
            self.get_logger().info(
                f"Targets [m]: PRE_GRASP=({x:.3f}, {y:.3f}, {pre_grasp_z:.3f}); "
                f"GRASP=({x:.3f}, {y:.3f}, {grasp_z:.3f}); "
                f"LIFT=({x:.3f}, {y:.3f}, {lift_z:.3f})"
            )
            self.get_logger().info(
                f"TCP orientation [xyzw]: [{orientation.x:.6f}, "
                f"{orientation.y:.6f}, {orientation.z:.6f}, {orientation.w:.6f}]"
            )
            self.get_logger().info(
                f"Modelled minimum GRASP TCP z={minimum_grasp_z:.3f} m "
                f"(table top={self.table_top_z:.3f}, "
                f"TCP-to-fingertip={self.tcp_to_fingertip:.3f}, "
                f"clearance={self.fingertip_clearance:.3f})"
            )

            if self.preview_only:
                self.get_logger().info(
                    "PREVIEW ONLY: no arm or gripper commands sent. This is a "
                    "coordinate preview, not a motion-plan or collision check. "
                    "After checking the setup, rerun with -p preview_only:=false."
                )
                return

            if not self.geometry_confirmed:
                self.get_logger().error(
                    "Execution blocked: verify horizontal table, TCP/finger "
                    "geometry, tool orientation and scene, then set geometry_confirmed."
                )
                return
            mode = "PRE_GRASP ONLY" if self.stop_after_pre_grasp else "FULL PICK AND PLACE"
            print(f"Type START to execute {mode}: ", flush=True)
            try:
                answer = input().strip().upper()
            except EOFError:
                self.get_logger().error(
                    "No keyboard input. Run this node directly with ros2 run."
                )
                return  # This return belongs INSIDE except.

            self.get_logger().info(f"Received command: {answer!r}")
            if answer != "START":
                self.get_logger().warning(f"Operation cancelled. Received: {answer!r}")
                return  # This return belongs INSIDE if, not after it.

            self.get_logger().info("START accepted; entering pick-and-place sequence")
            self.get_logger().info("Creating gripper controller")
            gripper = GripperController()
            self.get_logger().info("Gripper controller ready; requesting PRE_GRASP")

            if not self.add_table_collision():
                return
            if not self.move_to_cartesian_pose(x, y, pre_grasp_z, orientation, "PRE_GRASP"):
                return
            if self.stop_after_pre_grasp:
                self.get_logger().info(
                    "Reached PRE_GRASP. Stopping before opening/descent. "
                    "Inspect orientation and clearance; reacquire for the next run."
                )
                return
            if not gripper.open_gripper():
                self.get_logger().error("Could not open gripper; stopping")
                return
            if not self.move_vertical_cartesian(
                pre_grasp_z, grasp_z, x, y, orientation
            ):
                self.get_logger().error("Straight descent was not fully planned/executed")
                return
            if not self.close_on_object(width=0.045, speed=0.02, force=10.0):
                self.get_logger().error("Grasp failed; object will not be lifted")
                return
            if not self.move_vertical_cartesian(grasp_z, lift_z, x, y, orientation):
                return

            # Use your existing saved destination poses and motion helpers.
            for pose_name in ("PRE_BIN", "BIN"):
                self.get_logger().info(f"Moving to saved pose {pose_name}")
                if not self.move_to(pose_name):
                    self.get_logger().error(f"Saved pose {pose_name} failed; stopping")
                    return
            if not gripper.open_gripper():
                self.get_logger().error("Could not release object; stopping")
                return
            for pose_name in ("POST_BIN", "HOME"):
                self.get_logger().info(f"Moving to saved pose {pose_name}")
                if not self.move_to(pose_name):
                    self.get_logger().error(f"Saved pose {pose_name} failed; stopping")
                    return
            self.get_logger().info("Vision pick-and-place completed")

        except Exception as error:
            self.get_logger().error(f"Vision sequence failed: {type(error).__name__}: {error}")
        finally:
            if gripper is not None:
                gripper.destroy_node()
            # Do not re-arm the camera or retry after failure: the robot may
            # now be holding an object or be partway through a motion sequence.
            self.running = True
            self.sequence_timer.cancel()
            self.get_logger().info(
                "Cycle ended. No automatic restart. Inspect the robot/setup "
                "before running another cycle."
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionPickPlaceDemo()
        # spin_once returns BEFORE motion starts, so blocking action helpers
        # are never called inside a callback or beside another spinning thread.
        while rclpy.ok() and not node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
        if rclpy.ok() and node.running:
            node.execute_vision_sequence()
    except KeyboardInterrupt:
        # This is process interruption, NOT the robot's physical emergency stop.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
