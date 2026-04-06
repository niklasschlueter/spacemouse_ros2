import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node


class TwistToPoseNode(Node):
    """Maps SpaceMouse input directly to a target pose offset from the actual EE pose.

    SpaceMouse deflection [-1, 1] maps linearly to an offset within max_distance (m)
    and max_rotation (rad) from the robot's current pose. Zero input = target at actual
    pose. No integration, no accumulated state.

    Orientation is handled in quaternion space to avoid gimbal lock.

    The frame for translation and rotation inputs is configurable:
      translation_frame / rotation_frame: "world" (base frame) or "ee" (EE frame).
    """

    def __init__(self):
        super().__init__("twist_to_pose_node")
        self.get_logger().info("Initializing Twist-to-Pose node...")

        # Deadband — input must exceed this before any motion is applied;
        # once exceeded the threshold is subtracted so motion starts smoothly from zero.
        self.declare_parameter("linear_deadzone", 0.05)
        self.declare_parameter("angular_deadzone", 0.05)

        # Max offset from actual pose at full SpaceMouse deflection
        self.declare_parameter("max_distance", 0.15)  # meters
        self.declare_parameter("max_rotation", 0.5)  # radians

        # When true, the target freezes at the actual pose on release (stable setpoint).
        # When false, the target continuously tracks the actual pose while idle.
        self.declare_parameter("latch_on_idle", True)

        self.declare_parameter("timer_period", 0.01)  # 100Hz

        # "world" or "ee" — frame in which spacemouse inputs are interpreted
        self.declare_parameter("translation_frame", "ee")
        self.declare_parameter("rotation_frame", "ee")

        # Topics
        self.declare_parameter("current_pose_topic", "/current_pose")
        self.declare_parameter("target_pose_topic", "/target_pose")

        # Input frame mapping
        self.declare_parameter("input_frame_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("flip_input_x", False)
        self.declare_parameter("flip_input_y", False)
        self.declare_parameter("flip_input_z", False)

        self.linear_deadzone = self.get_parameter("linear_deadzone").value
        self.angular_deadzone = self.get_parameter("angular_deadzone").value
        self.max_distance = self.get_parameter("max_distance").value
        self.max_rotation = self.get_parameter("max_rotation").value
        self._latch_on_idle = self.get_parameter("latch_on_idle").value
        self.dt = self.get_parameter("timer_period").value
        self.translation_frame = self.get_parameter("translation_frame").value
        self.rotation_frame = self.get_parameter("rotation_frame").value
        current_pose_topic = self.get_parameter("current_pose_topic").value
        target_pose_topic = self.get_parameter("target_pose_topic").value
        rpy = self.get_parameter("input_frame_rpy").value
        self._input_qx, self._input_qy, self._input_qz, self._input_qw = (
            self._rpy_degrees_to_quaternion(rpy[0], rpy[1], rpy[2])
        )
        self._flip_input_x = self.get_parameter("flip_input_x").value
        self._flip_input_y = self.get_parameter("flip_input_y").value
        self._flip_input_z = self.get_parameter("flip_input_z").value

        # Allow runtime tuning via ros2 param set / rqt_reconfigure
        self.add_on_set_parameters_callback(self._on_parameter_change)

        # Robot's actual pose
        self.actual_x = 0.0
        self.actual_y = 0.0
        self.actual_z = 0.0
        self.actual_qx = 0.0
        self.actual_qy = 0.0
        self.actual_qz = 0.0
        self.actual_qw = 1.0

        self._pose_initialized = False
        self._last_pose_time = None
        # If current_pose stops publishing (robot reset/respawn), the node stops
        # publishing target_pose and re-latches to the new actual pose when it resumes.
        self._pose_timeout = 0.5  # seconds
        self.current_twist = Twist()
        self._was_active = False
        self._status_timer_ticks = 0  # for periodic status logging
        self._latched_x = 0.0
        self._latched_y = 0.0
        self._latched_z = 0.0
        self._latched_qx = 0.0
        self._latched_qy = 0.0
        self._latched_qz = 0.0
        self._latched_qw = 1.0

        # Optional secondary device for split translation/rotation control
        self.declare_parameter("secondary_twist_topic", "")
        secondary_twist_topic = self.get_parameter("secondary_twist_topic").value
        self._dual_device = bool(secondary_twist_topic)
        self._secondary_twist = Twist()

        # Publishers and Subscribers
        self.pose_pub = self.create_publisher(PoseStamped, target_pose_topic, 10)

        self.twist_sub = self.create_subscription(
            Twist,
            "space_mouse/target_cartesian_velocity_percent",
            self._twist_callback,
            10,
        )

        if self._dual_device:
            self.create_subscription(
                Twist,
                secondary_twist_topic,
                self._secondary_twist_callback,
                10,
            )
            self.get_logger().info(
                f"Dual device mode: linear from primary, angular from '{secondary_twist_topic}'"
            )

        self.actual_pose_sub = self.create_subscription(
            PoseStamped, current_pose_topic, self._actual_pose_callback, 10
        )

        self.get_logger().info(
            f"translation_frame={self.translation_frame}, rotation_frame={self.rotation_frame}"
        )

        self.timer = self.create_timer(self.dt, self._timer_callback)

    def _on_parameter_change(self, params):
        """Update cached values when parameters are changed at runtime."""
        for param in params:
            if param.name == "max_distance":
                self.max_distance = param.value
            elif param.name == "max_rotation":
                self.max_rotation = param.value
            elif param.name == "linear_deadzone":
                self.linear_deadzone = param.value
            elif param.name == "angular_deadzone":
                self.angular_deadzone = param.value
            elif param.name == "latch_on_idle":
                self._latch_on_idle = param.value
            elif param.name == "translation_frame":
                self.translation_frame = param.value
            elif param.name == "rotation_frame":
                self.rotation_frame = param.value
            elif param.name == "flip_input_x":
                self._flip_input_x = param.value
            elif param.name == "flip_input_y":
                self._flip_input_y = param.value
            elif param.name == "flip_input_z":
                self._flip_input_z = param.value
            elif param.name == "input_frame_rpy":
                self._input_qx, self._input_qy, self._input_qz, self._input_qw = (
                    self._rpy_degrees_to_quaternion(param.value[0], param.value[1], param.value[2])
                )
        return SetParametersResult(successful=True)

    def _twist_callback(self, msg: Twist):
        """Stores the latest Twist command from the primary SpaceMouse."""
        self.current_twist = msg

    def _secondary_twist_callback(self, msg: Twist):
        """Stores the latest Twist command from the secondary SpaceMouse."""
        self._secondary_twist = msg

    def _actual_pose_callback(self, msg: PoseStamped):
        """Updates the internal tracker with the robot's real physical location."""
        q = msg.pose.orientation
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if norm < 0.5:
            self.get_logger().warn(
                "Near-zero quaternion in /current_pose; skipping orientation update."
            )
            return

        self.actual_x = msg.pose.position.x
        self.actual_y = msg.pose.position.y
        self.actual_z = msg.pose.position.z
        self.actual_qx = q.x / norm
        self.actual_qy = q.y / norm
        self.actual_qz = q.z / norm
        self.actual_qw = q.w / norm

        was_stale = not self._pose_initialized or (
            self._last_pose_time is not None
            and (self.get_clock().now().nanoseconds - self._last_pose_time) * 1e-9
            > self._pose_timeout
        )
        self._last_pose_time = self.get_clock().now().nanoseconds

        if was_stale:
            self._latched_x = self.actual_x
            self._latched_y = self.actual_y
            self._latched_z = self.actual_z
            self._latched_qx = self.actual_qx
            self._latched_qy = self.actual_qy
            self._latched_qz = self.actual_qz
            self._latched_qw = self.actual_qw
            self._was_active = False
            if not self._pose_initialized:
                self._pose_initialized = True
            self.get_logger().info(
                f"Target latched to current_pose: "
                f"({self.actual_x:.3f}, {self.actual_y:.3f}, {self.actual_z:.3f})"
            )

    def _timer_callback(self):
        """Maps SpaceMouse input to a target pose offset from the actual pose."""
        # Log status every 2 seconds (200 ticks at 100 Hz).
        self._status_timer_ticks += 1
        if self._status_timer_ticks >= 200:
            self._status_timer_ticks = 0
            if not self._pose_initialized:
                self.get_logger().warn(
                    f"Waiting for first message on '{self.get_parameter('current_pose_topic').value}' ..."
                )
            elif (
                self._last_pose_time is not None
                and (self.get_clock().now().nanoseconds - self._last_pose_time) * 1e-9
                > self._pose_timeout
            ):
                age = (self.get_clock().now().nanoseconds - self._last_pose_time) * 1e-9
                self.get_logger().warn(
                    f"current_pose stale ({age:.1f}s > {self._pose_timeout}s timeout) — paused"
                )
            else:
                all_zero = all(
                    v == 0
                    for v in [
                        self.current_twist.linear.x,
                        self.current_twist.linear.y,
                        self.current_twist.linear.z,
                        self.current_twist.angular.x,
                        self.current_twist.angular.y,
                        self.current_twist.angular.z,
                    ]
                )
                if all_zero:
                    self.get_logger().info("Publishing target_pose (SpaceMouse idle — latched)")
                else:
                    self.get_logger().info("Publishing target_pose (SpaceMouse active)")

        if not self._pose_initialized:
            return

        # Stop publishing if current_pose has gone stale (robot stopped/respawning).
        if (
            self._last_pose_time is not None
            and (self.get_clock().now().nanoseconds - self._last_pose_time) * 1e-9
            > self._pose_timeout
        ):
            return

        # 1. Select angular source (secondary device in dual mode, primary otherwise).
        angular_src = self._secondary_twist if self._dual_device else self.current_twist

        # 2. Apply deadband to raw inputs.
        lx = self._apply_deadzone(self.current_twist.linear.x, self.linear_deadzone)
        ly = self._apply_deadzone(self.current_twist.linear.y, self.linear_deadzone)
        lz = self._apply_deadzone(self.current_twist.linear.z, self.linear_deadzone)
        ax = self._apply_deadzone(angular_src.angular.x, self.angular_deadzone)
        ay = self._apply_deadzone(angular_src.angular.y, self.angular_deadzone)
        az = self._apply_deadzone(angular_src.angular.z, self.angular_deadzone)

        # 3. Scale to max offset (input is [-1, 1] after deadzone).
        lx *= self.max_distance
        ly *= self.max_distance
        lz *= self.max_distance
        ax *= self.max_rotation
        ay *= self.max_rotation
        az *= self.max_rotation

        # 4. Apply input frame rotation and per-axis flips.
        if self._input_qw < 1.0:  # skip if identity quaternion
            lx, ly, lz = self._rotate_vector_by_quaternion(
                lx, ly, lz, self._input_qx, self._input_qy, self._input_qz, self._input_qw
            )
            ax, ay, az = self._rotate_vector_by_quaternion(
                ax, ay, az, self._input_qx, self._input_qy, self._input_qz, self._input_qw
            )
        if self._flip_input_x:
            lx, ax = -lx, -ax
        if self._flip_input_y:
            ly, ay = -ly, -ay
        if self._flip_input_z:
            lz, az = -lz, -az

        # 5. Rotate into world frame if EE-relative mode is active.
        if self.translation_frame == "ee":
            lx, ly, lz = self._rotate_vector_by_quaternion(
                lx, ly, lz, self.actual_qx, self.actual_qy, self.actual_qz, self.actual_qw
            )
        if self.rotation_frame == "ee":
            ax, ay, az = self._rotate_vector_by_quaternion(
                ax, ay, az, self.actual_qx, self.actual_qy, self.actual_qz, self.actual_qw
            )

        # 6. Check if idle — latch or track depending on config.
        all_idle = lx == 0 and ly == 0 and lz == 0 and ax == 0 and ay == 0 and az == 0

        if all_idle:
            if self._latch_on_idle:
                if self._was_active:
                    # Freeze target where the robot is right now
                    self._latched_x = self.actual_x
                    self._latched_y = self.actual_y
                    self._latched_z = self.actual_z
                    self._latched_qx = self.actual_qx
                    self._latched_qy = self.actual_qy
                    self._latched_qz = self.actual_qz
                    self._latched_qw = self.actual_qw
                    self._was_active = False
                target_x = self._latched_x
                target_y = self._latched_y
                target_z = self._latched_z
                target_qx = self._latched_qx
                target_qy = self._latched_qy
                target_qz = self._latched_qz
                target_qw = self._latched_qw
            else:
                target_x = self.actual_x
                target_y = self.actual_y
                target_z = self.actual_z
                target_qx = self.actual_qx
                target_qy = self.actual_qy
                target_qz = self.actual_qz
                target_qw = self.actual_qw
        else:
            # 7. Compute target = actual + offset.
            target_x = self.actual_x + lx
            target_y = self.actual_y + ly
            target_z = self.actual_z + lz

            # 8. Compute orientation target = delta_quat(offset) * actual.
            dqx, dqy, dqz, dqw = self._rotation_vector_to_quaternion(ax, ay, az)
            target_qx, target_qy, target_qz, target_qw = self._quaternion_multiply(
                dqx,
                dqy,
                dqz,
                dqw,
                self.actual_qx,
                self.actual_qy,
                self.actual_qz,
                self.actual_qw,
            )
            target_qx, target_qy, target_qz, target_qw = self._quaternion_normalize(
                target_qx, target_qy, target_qz, target_qw
            )
            self._was_active = True

        # 9. Publish the target pose.
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "base_link"
        pose_msg.pose.position.x = target_x
        pose_msg.pose.position.y = target_y
        pose_msg.pose.position.z = target_z
        pose_msg.pose.orientation.x = target_qx
        pose_msg.pose.orientation.y = target_qy
        pose_msg.pose.orientation.z = target_qz
        pose_msg.pose.orientation.w = target_qw
        self.pose_pub.publish(pose_msg)

    # ---------------------------------------------------------------------------
    # Helpers
    # ---------------------------------------------------------------------------

    def _rpy_degrees_to_quaternion(self, roll_deg, pitch_deg, yaw_deg):
        """Convert roll/pitch/yaw in degrees (intrinsic ZYX) to a unit quaternion."""
        r = math.radians(roll_deg)
        p = math.radians(pitch_deg)
        y = math.radians(yaw_deg)
        cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
        cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
        cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _apply_deadzone(self, value, threshold):
        """Returns 0 if |value| < threshold, otherwise subtracts threshold from the magnitude."""
        if abs(value) < threshold:
            return 0.0
        return math.copysign(abs(value) - threshold, value)

    def _quaternion_multiply(self, q1x, q1y, q1z, q1w, q2x, q2y, q2z, q2w):
        """Hamilton product q1 * q2."""
        return (
            q1w * q2x + q1x * q2w + q1y * q2z - q1z * q2y,
            q1w * q2y - q1x * q2z + q1y * q2w + q1z * q2x,
            q1w * q2z + q1x * q2y - q1y * q2x + q1z * q2w,
            q1w * q2w - q1x * q2x - q1y * q2y - q1z * q2z,
        )

    def _quaternion_normalize(self, qx, qy, qz, qw):
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-10:
            return 0.0, 0.0, 0.0, 1.0
        return qx / norm, qy / norm, qz / norm, qw / norm

    def _rotation_vector_to_quaternion(self, rx, ry, rz):
        """Convert a rotation vector (axis * angle) to a unit quaternion."""
        angle = math.sqrt(rx * rx + ry * ry + rz * rz)
        if angle < 1e-10:
            return 0.0, 0.0, 0.0, 1.0
        s = math.sin(angle * 0.5) / angle
        return rx * s, ry * s, rz * s, math.cos(angle * 0.5)

    def _rotate_vector_by_quaternion(self, vx, vy, vz, qx, qy, qz, qw):
        """Rotate vector by quaternion (Rodrigues formula, equivalent to q * v * q_conj)."""
        cx = qy * vz - qz * vy
        cy = qz * vx - qx * vz
        cz = qx * vy - qy * vx
        cx2 = qy * cz - qz * cy
        cy2 = qz * cx - qx * cz
        cz2 = qx * cy - qy * cx
        return (
            vx + 2.0 * (qw * cx + cx2),
            vy + 2.0 * (qw * cy + cy2),
            vz + 2.0 * (qw * cz + cz2),
        )


def main(args=None):
    rclpy.init(args=args)
    node = TwistToPoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
