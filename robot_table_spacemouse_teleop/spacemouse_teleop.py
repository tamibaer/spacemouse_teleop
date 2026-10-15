import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Joy

from control_msgs.action import GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType


class SpacemouseTeleop(Node):
    """Spacemouse -> MoveIt Servo (Arm) und GripperCommand (Greifer)."""

    def __init__(self):
        super().__init__('spacemouse_teleop')
        self.max_linear = self.declare_parameter('max_linear', 1.0).value    # m/s
        self.max_angular = self.declare_parameter('max_angular', 2.0).value  # rad/s
        self.frame = self.declare_parameter('frame', 'ur5e_base_link').value

        self.twist_pub = self.create_publisher(TwistStamped, 'servo_node/delta_twist_cmds', 10)
        self.gripper = ActionClient(self, GripperCommand, 'robotiq_gripper_controller/gripper_cmd')
        self.gripper_closed = False
        self.last_buttons = []

        self.get_logger().info('Warte auf servo_node ...')
        servo = self.create_client(ServoCommandType, 'servo_node/switch_command_type')
        servo.wait_for_service()
        servo.call_async(ServoCommandType.Request(command_type=ServoCommandType.Request.TWIST))

        self.create_subscription(Joy, 'spacenav/joy', self.on_joy, 10)
        self.get_logger().info('Bereit: Spacemouse bewegt den Arm, linke Taste oeffnet/schliesst den Greifer')

    def on_joy(self, msg):
        self.send_twist(msg.axes)
        if msg.buttons and msg.buttons[0] and not (self.last_buttons and self.last_buttons[0]):
            self.toggle_gripper()
        self.last_buttons = list(msg.buttons)

    def send_twist(self, axes):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.frame
        mx, my, mz, rx, ry, rz = axes[:6]
        cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z = (self.max_linear * a for a in (-my, mx, mz))
        cmd.twist.angular.x, cmd.twist.angular.y, cmd.twist.angular.z = (self.max_angular * a for a in (-ry, rx, rz))
        self.twist_pub.publish(cmd)

    def toggle_gripper(self):
        self.gripper_closed = not self.gripper_closed
        goal = GripperCommand.Goal()
        goal.command.position = 0.8 if self.gripper_closed else 0.1
        goal.command.max_effort = 100.0
        self.gripper.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = SpacemouseTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
