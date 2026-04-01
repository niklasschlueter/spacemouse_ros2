import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import pyspacemouse


class SpaceMousePublisher(Node):
    """
    A ROS2 Node that publishes 3D mouse input as Twist messages.

    This class initializes a ROS2 publisher to publish geometry_msgs/Twist messages
    based on the input from a 3D SpaceMouse device. It uses the pyspacemouse library
    to read the device state and publishes the corresponding linear and angular
    velocities at a fixed rate.
    """

    def __init__(self):
        super().__init__("spacemouse_publisher")
        self.get_logger().info("Initializing SpaceMouse publisher...")

        self.declare_parameter("operator_position_front", True)
        self._operator_position_front = (
            self.get_parameter("operator_position_front")
            .get_parameter_value()
            .bool_value
        )
        self.get_logger().info(
            f"Operator position front: {self._operator_position_front}"
        )

        self.declare_parameter("device_path", "")
        self._device_path = (
            self.get_parameter("device_path").get_parameter_value().string_value
        )

        self.declare_parameter("gripper_mode", "absolute")
        self._gripper_mode = (
            self.get_parameter("gripper_mode").get_parameter_value().string_value
        )

        self.declare_parameter("gripper_step", 0.01)
        self._gripper_step = (
            self.get_parameter("gripper_step").get_parameter_value().double_value
        )

        self.declare_parameter("gripper_interface", "action")
        self._gripper_interface = (
            self.get_parameter("gripper_interface").get_parameter_value().string_value
        )

        self._gripper_width = 0.0
        self._prev_buttons = [0, 0]
        self._gripper_publisher = None
        self._gripper_action_client = None
        self._goal_in_flight = False

        self._twist_publisher = self.create_publisher(
            Twist, "space_mouse/target_cartesian_velocity_percent", 10
        )

        if self._gripper_interface == "action":
            from rclpy.action import ActionClient
            from control_msgs.action import GripperCommand

            self.declare_parameter("gripper_action", "manipulators/arm_0_gripper_controller/gripper_cmd")
            gripper_action = (
                self.get_parameter("gripper_action").get_parameter_value().string_value
            )
            self.declare_parameter("gripper_max_position", 0.8)
            self._gripper_max_position = (
                self.get_parameter("gripper_max_position")
                .get_parameter_value()
                .double_value
            )
            self.declare_parameter("gripper_max_effort", 50.0)
            self._gripper_max_effort = (
                self.get_parameter("gripper_max_effort")
                .get_parameter_value()
                .double_value
            )
            self._GripperCommand = GripperCommand
            self._gripper_action_client = ActionClient(self, GripperCommand, gripper_action)
            self.get_logger().info(f"Gripper action client: {gripper_action}")
        else:
            self.declare_parameter("gripper_topic", "space_mouse/target_gripper_width_percent")
            gripper_topic = (
                self.get_parameter("gripper_topic").get_parameter_value().string_value
            )
            self._gripper_publisher = self.create_publisher(Float32, gripper_topic, 10)
            self.get_logger().info(f"Gripper topic: {gripper_topic}")

        self._timer = self.create_timer(0.01, self._timer_callback)
        self._device_open_success = pyspacemouse.open(
            dof_callback=None,
            path=self._device_path,
        )

    def _send_gripper_action(self, width_fraction):
        if not self._gripper_action_client.server_is_ready():
            return
        if self._goal_in_flight:
            return
        goal = self._GripperCommand.Goal()
        goal.command.position = width_fraction * self._gripper_max_position
        goal.command.max_effort = self._gripper_max_effort
        self._goal_in_flight = True
        future = self._gripper_action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_sent_callback)

    def _goal_sent_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._goal_in_flight = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: setattr(self, "_goal_in_flight", False))

    def _publish_gripper(self, width_fraction):
        if self._gripper_interface == "action":
            self._send_gripper_action(width_fraction)
        else:
            msg = Float32()
            msg.data = float(width_fraction)
            self._gripper_publisher.publish(msg)

    def _timer_callback(self):
        if not self._device_open_success:
            return

        state = pyspacemouse.read()

        twist_msg = Twist()
        twist_msg.linear.x = -float(state.y)
        twist_msg.linear.y = float(state.x)
        twist_msg.linear.z = float(state.z)
        twist_msg.angular.x = -float(state.roll)
        twist_msg.angular.y = -float(state.pitch)
        twist_msg.angular.z = -float(state.yaw)

        if not self._operator_position_front:
            twist_msg.linear.x *= -1
            twist_msg.linear.y *= -1
            twist_msg.angular.x *= -1
            twist_msg.angular.y *= -1

        self._twist_publisher.publish(twist_msg)

        buttons = list(state.buttons)

        if self._gripper_mode == "absolute":
            if buttons[0] and not self._prev_buttons[0]:
                self._publish_gripper(0.0)
            if buttons[1] and not self._prev_buttons[1]:
                self._publish_gripper(1.0)
        else:  # relative
            if buttons[0]:
                self._gripper_width = min(1.0, self._gripper_width + self._gripper_step)
                self._publish_gripper(self._gripper_width)
            elif buttons[1]:
                self._gripper_width = max(0.0, self._gripper_width - self._gripper_step)
                self._publish_gripper(self._gripper_width)

        self._prev_buttons = buttons


def main(args=None):
    rclpy.init(args=args)
    spacemouse_publisher = SpaceMousePublisher()
    rclpy.spin(spacemouse_publisher)
    spacemouse_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
