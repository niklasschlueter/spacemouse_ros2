import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node


class TwistToPoseNode(Node):
    """
    Integrates Twist velocities into a target PoseStamped.
    The target is seeded from /current_pose on first receipt, so spacemouse
    inputs are applied as deltas relative to the robot's actual pose.
    Caps the maximum translation distance and rotation relative to the actual pose.

    Orientation is tracked as a quaternion throughout to avoid gimbal lock and
    Euler angle wrapping discontinuities.

    The frame for translation and rotation inputs is configurable:
      translation_frame / rotation_frame: "world" (base frame) or "ee" (EE frame).
    """

    def __init__(self):
        super().__init__("twist_to_pose_node")
        self.get_logger().info("Initializing Twist-to-Pose Integrator...")

        # Sensitivity
        self.declare_parameter("linear_scale", 0.1)  # m/s sensitivity
        self.declare_parameter("angular_scale", 0.25)  # rad/s sensitivity

        # Deadband — input must exceed this before any motion is applied;
        # once exceeded the threshold is subtracted so motion starts smoothly from zero.
        self.declare_parameter("linear_deadzone", 0.05)  # fraction of full input range
        self.declare_parameter("angular_deadzone", 0.05)  # fraction of full input range

        # Clipping — maximum deviation of target from actual pose
        self.declare_parameter("max_distance", 0.15)  # translational clip (m)
        self.declare_parameter("max_rotation", 0.5)  # rotational clip (rad)

        # Snap target back to actual pose when all inputs are idle.
        # snap_threshold controls when "idle" is declared — inputs below this trigger
        # the snap. Set higher than linear/angular_deadzone so the snap fires early
        # as the device springs back to center, rather than waiting for full settle.
        self.declare_parameter("snap_to_actual_on_idle", True)
        self.declare_parameter("linear_snap_threshold", 0.15)
        self.declare_parameter("angular_snap_threshold", 0.15)

        self.declare_parameter("timer_period", 0.01)  # 100Hz integration loop

        # "world" or "ee" — frame in which spacemouse inputs are interpreted
        self.declare_parameter("translation_frame", "ee")
        self.declare_parameter("rotation_frame", "ee")

        # Topics
        self.declare_parameter("current_pose_topic", "/current_pose")
        self.declare_parameter("target_pose_topic", "/target_pose")

        # Input frame mapping — applied to SpaceMouse inputs before EE/world-frame processing.
        # input_frame_rpy rotates the input frame (intrinsic ZYX, degrees).
        # flip_input_* inverts individual axes after the rotation (no right-hand constraint).
        self.declare_parameter("input_frame_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("flip_input_x", False)
        self.declare_parameter("flip_input_y", False)
        self.declare_parameter("flip_input_z", False)

        self.linear_scale = self.get_parameter("linear_scale").value
        self.angular_scale = self.get_parameter("angular_scale").value
        self.linear_deadzone = self.get_parameter("linear_deadzone").value
        self.angular_deadzone = self.get_parameter("angular_deadzone").value
        self.max_distance = self.get_parameter("max_distance").value
        self.max_rotation = self.get_parameter("max_rotation").value
        self.snap_to_actual_on_idle = self.get_parameter("snap_to_actual_on_idle").value
        self.linear_snap_threshold = self.get_parameter("linear_snap_threshold").value
        self.angular_snap_threshold = self.get_parameter("angular_snap_threshold").value
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

        # Integrated target pose — position and quaternion orientation
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        self.target_qx = 0.0
        self.target_qy = 0.0
        self.target_qz = 0.0
        self.target_qw = 1.0

        # Robot's actual pose — position and quaternion orientation
        self.actual_x = 0.0
        self.actual_y = 0.0
        self.actual_z = 0.0
        self.actual_qx = 0.0
        self.actual_qy = 0.0
        self.actual_qz = 0.0
        self.actual_qw = 1.0

        # Whether the target has been seeded from current_pose yet
        self._pose_initialized = False

        # Latest Twist commands
        self.current_twist = Twist()

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
            f"translation_frame={self.translation_frame}, rotation_frame={self.rotation_frame}, "
            f"snap_to_actual_on_idle={self.snap_to_actual_on_idle}"
        )

        # Integration Timer
        self.timer = self.create_timer(self.dt, self._integration_timer_callback)

    def _twist_callback(self, msg: Twist):
        """Stores the latest Twist command from the primary SpaceMouse."""
        self.current_twist = msg

    def _secondary_twist_callback(self, msg: Twist):
        """Stores the latest Twist command from the secondary SpaceMouse."""
        self._secondary_twist = msg

    def _actual_pose_callback(self, msg: PoseStamped):
        """Updates the internal tracker with the robot's real physical location."""
        self.actual_x = msg.pose.position.x
        self.actual_y = msg.pose.position.y
        self.actual_z = msg.pose.position.z

        q = msg.pose.orientation
        norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if norm < 0.5:
            self.get_logger().warn(
                "Near-zero quaternion in /current_pose; skipping orientation update."
            )
        else:
            self.actual_qx = q.x / norm
            self.actual_qy = q.y / norm
            self.actual_qz = q.z / norm
            self.actual_qw = q.w / norm

        if not self._pose_initialized:
            self.target_x = self.actual_x
            self.target_y = self.actual_y
            self.target_z = self.actual_z
            self.target_qx = self.actual_qx
            self.target_qy = self.actual_qy
            self.target_qz = self.actual_qz
            self.target_qw = self.actual_qw
            self._pose_initialized = True
            self.get_logger().info(
                f"Target seeded from current_pose: "
                f"({self.target_x:.3f}, {self.target_y:.3f}, {self.target_z:.3f})"
            )

    def _integration_timer_callback(self):
        """Integrates velocity to position and publishes the capped pose."""
        if not self._pose_initialized:
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

        # 3. Check if idle — if so, snap target to actual pose and skip integration.
        #    Uses snap_threshold on raw input (larger than deadzone) so the snap fires
        #    while the device is still springing back to center, not after full settle.
        linear_idle = (
            abs(self.current_twist.linear.x) < self.linear_snap_threshold
            and abs(self.current_twist.linear.y) < self.linear_snap_threshold
            and abs(self.current_twist.linear.z) < self.linear_snap_threshold
        )
        angular_idle = (
            abs(angular_src.angular.x) < self.angular_snap_threshold
            and abs(angular_src.angular.y) < self.angular_snap_threshold
            and abs(angular_src.angular.z) < self.angular_snap_threshold
        )
        if linear_idle and angular_idle:
            if self.snap_to_actual_on_idle:
                self.target_x = self.actual_x
                self.target_y = self.actual_y
                self.target_z = self.actual_z
                self.target_qx = self.actual_qx
                self.target_qy = self.actual_qy
                self.target_qz = self.actual_qz
                self.target_qw = self.actual_qw
        else:
            # 4. Scale by sensitivity and timestep.
            lx *= self.linear_scale * self.dt
            ly *= self.linear_scale * self.dt
            lz *= self.linear_scale * self.dt
            ax *= self.angular_scale * self.dt
            ay *= self.angular_scale * self.dt
            az *= self.angular_scale * self.dt

            # 5. Apply input frame rotation and per-axis flips.
            if self._input_qw < 1.0:  # skip if identity quaternion
                lx, ly, lz = self._rotate_vector_by_quaternion(
                    lx,
                    ly,
                    lz,
                    self._input_qx,
                    self._input_qy,
                    self._input_qz,
                    self._input_qw,
                )
                ax, ay, az = self._rotate_vector_by_quaternion(
                    ax,
                    ay,
                    az,
                    self._input_qx,
                    self._input_qy,
                    self._input_qz,
                    self._input_qw,
                )
            if self._flip_input_x:
                lx, ax = -lx, -ax
            if self._flip_input_y:
                ly, ay = -ly, -ay
            if self._flip_input_z:
                lz, az = -lz, -az

            # 6. Rotate into world frame if EE-relative mode is active.
            if self.translation_frame == "ee":
                lx, ly, lz = self._rotate_vector_by_quaternion(
                    lx,
                    ly,
                    lz,
                    self.actual_qx,
                    self.actual_qy,
                    self.actual_qz,
                    self.actual_qw,
                )
            if self.rotation_frame == "ee":
                ax, ay, az = self._rotate_vector_by_quaternion(
                    ax,
                    ay,
                    az,
                    self.actual_qx,
                    self.actual_qy,
                    self.actual_qz,
                    self.actual_qw,
                )

            # 7. Integrate translation.
            self.target_x += lx
            self.target_y += ly
            self.target_z += lz

            # 8. Integrate rotation via quaternion composition (avoids gimbal lock/wrapping).
            #    (ax, ay, az) is a rotation vector in world frame; convert to quaternion delta.
            dqx, dqy, dqz, dqw = self._rotation_vector_to_quaternion(ax, ay, az)
            # Apply world-frame rotation: q_new = dq * q_target
            self.target_qx, self.target_qy, self.target_qz, self.target_qw = (
                self._quaternion_multiply(
                    dqx,
                    dqy,
                    dqz,
                    dqw,
                    self.target_qx,
                    self.target_qy,
                    self.target_qz,
                    self.target_qw,
                )
            )
            # Keep target quaternion normalised
            self.target_qx, self.target_qy, self.target_qz, self.target_qw = (
                self._quaternion_normalize(
                    self.target_qx, self.target_qy, self.target_qz, self.target_qw
                )
            )

            # 9. Apply translational clip.
            dist = math.sqrt(
                (self.target_x - self.actual_x) ** 2
                + (self.target_y - self.actual_y) ** 2
                + (self.target_z - self.actual_z) ** 2
            )
            if dist > self.max_distance:
                scale_factor = self.max_distance / dist
                self.target_x = self.actual_x + (self.target_x - self.actual_x) * scale_factor
                self.target_y = self.actual_y + (self.target_y - self.actual_y) * scale_factor
                self.target_z = self.actual_z + (self.target_z - self.actual_z) * scale_factor

            # 10. Apply rotational clip (quaternion-space, avoids wrapping).
            (self.target_qx, self.target_qy, self.target_qz, self.target_qw) = (
                self._clip_rotation_to_actual(
                    self.target_qx,
                    self.target_qy,
                    self.target_qz,
                    self.target_qw,
                    self.actual_qx,
                    self.actual_qy,
                    self.actual_qz,
                    self.actual_qw,
                    self.max_rotation,
                )
            )

        # 11. Publish the target pose.
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "base_link"

        pose_msg.pose.position.x = self.target_x
        pose_msg.pose.position.y = self.target_y
        pose_msg.pose.position.z = self.target_z

        pose_msg.pose.orientation.x = self.target_qx
        pose_msg.pose.orientation.y = self.target_qy
        pose_msg.pose.orientation.z = self.target_qz
        pose_msg.pose.orientation.w = self.target_qw

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
            sr * cp * cy - cr * sp * sy,  # qx
            cr * sp * cy + sr * cp * sy,  # qy
            cr * cp * sy - sr * sp * cy,  # qz
            cr * cp * cy + sr * sp * sy,  # qw
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

    def _clip_rotation_to_actual(self, tqx, tqy, tqz, tqw, aqx, aqy, aqz, aqw, max_angle):
        """Clip target quaternion so its angular distance from actual does not exceed max_angle."""
        # Relative rotation: dq = q_target * q_actual_conj
        dqx, dqy, dqz, dqw = self._quaternion_multiply(
            tqx,
            tqy,
            tqz,
            tqw,
            -aqx,
            -aqy,
            -aqz,
            aqw,  # conjugate of actual (unit quat)
        )
        dqx, dqy, dqz, dqw = self._quaternion_normalize(dqx, dqy, dqz, dqw)

        # Ensure shortest-path representation (w >= 0)
        if dqw < 0:
            dqx, dqy, dqz, dqw = -dqx, -dqy, -dqz, -dqw

        angle = 2.0 * math.acos(min(1.0, dqw))
        if angle <= max_angle:
            return tqx, tqy, tqz, tqw

        # Scale delta quaternion down to max_angle
        sin_half_orig = math.sin(angle * 0.5)
        if sin_half_orig < 1e-10:
            return tqx, tqy, tqz, tqw

        scale = math.sin(max_angle * 0.5) / sin_half_orig
        dqx *= scale
        dqy *= scale
        dqz *= scale
        dqw = math.cos(max_angle * 0.5)

        # Reconstruct target: q_target = dq_clipped * q_actual
        return self._quaternion_multiply(dqx, dqy, dqz, dqw, aqx, aqy, aqz, aqw)

    def _rotate_vector_by_quaternion(self, vx, vy, vz, qx, qy, qz, qw):
        """Rotate vector (vx, vy, vz) by quaternion using the Rodrigues formula.

        Equivalent to the sandwich product v' = q * v * q_conj.
        Used to transform a vector from EE frame into world/base frame.
        """
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
