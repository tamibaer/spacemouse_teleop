import math

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState

from control_msgs.action import GripperCommand
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType


JOINTS = ['shoulder_pan_joint', 
          'shoulder_lift_joint', 
          'elbow_joint',
          'wrist_1_joint', 
          'wrist_2_joint', 
          'wrist_3_joint']

# base_link -> DH-Basis ('base'): 180 Grad um z 
R_DH_FROM_BASELINK = np.diag([-1.0, -1.0, 1.0])


def dh_transform(theta, d, a, alpha):
    """T_{i-1}^i nach Standard-DH."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca,  st * sa, a * ct],
                     [st,  ct * ca, -ct * sa, a * st],
                     [0.0,      sa,       ca,      d],
                     [0.0,     0.0,      0.0,    1.0]])


def forward_kinematics(q, d, a, alpha, tcp_offset):
    """Alle Frames T_0^0..T_0^6 und die TCP-Position im DH-Basisframe."""
    T = np.eye(4)
    frames = [T]
    for i in range(6):
        T = T @ dh_transform(q[i], d[i], a[i], alpha[i])
        frames.append(T)
    p_tcp = T[:3, 3] + tcp_offset * T[:3, 2]
    return frames, p_tcp


def geometric_jacobian(frames, p_tcp):
    """J_i = [z_{i-1} x (p_tcp - o_{i-1}); z_{i-1}]."""
    J = np.zeros((6, 6))
    for i in range(6):
        z, o = frames[i][:3, 2], frames[i][:3, 3]
        J[:3, i] = np.cross(z, p_tcp - o)
        J[3:, i] = z
    return J


def damping(sigma_min, eps, lambda_max):
    """Variable Daempfung lambda^2: 0 weit weg, steigt stetig bis lambda_max^2 in der Singularitaet."""
    if sigma_min >= eps:
        return 0.0
    return (1.0 - (sigma_min / eps) ** 2) * lambda_max ** 2


def dls_solve(J, x_dot, eps, lambda_max):
    """q_dot = V diag(sigma / (sigma^2 + lambda^2)) U^T x_dot."""
    U, s, Vt = np.linalg.svd(J)
    lam2 = damping(s[-1], eps, lambda_max)
    q_dot = Vt.T @ ((s / (s ** 2 + lam2)) * (U.T @ x_dot))
    return q_dot, s[-1], lam2


class SpacemouseTeleop(Node):
    """Spacemouse -> Damped Least Squares -> MoveIt Servo (JointJog) und GripperCommand (Greifer)."""

    def __init__(self):
        super().__init__('spacemouse_teleop')

        # --- Spacemouse/Robot Parameter ---
        self.max_linear = self.declare_parameter('max_linear', 1.0).value    # m/s
        self.max_angular = self.declare_parameter('max_angular', 2.0).value  # rad/s
        self.frame = self.declare_parameter('frame', 'ur5e_base_link').value

        # --- DLS Parameter ---
        prefix = self.declare_parameter('joint_prefix', 'ur5e_').value
        self.tcp_offset = self.declare_parameter('tcp_offset', 0.0).value           # m
        self.sigma_threshold = self.declare_parameter('sigma_threshold', 0.05).value
        self.lambda_max = self.declare_parameter('lambda_max', 0.05).value
        self.max_joint_vel = self.declare_parameter('max_joint_vel', 1.5).value     # rad/s
        self.joy_timeout = self.declare_parameter('joy_timeout', 0.2).value         # s
        rate = self.declare_parameter('rate', 100.0).value                          # Hz

        # --- DH-Parameter ---
        self.dh_d = self.declare_parameter('dh_d', [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]).value
        self.dh_a = self.declare_parameter('dh_a', [0.0, -0.425, -0.3922, 0.0, 0.0, 0.0]).value
        self.dh_alpha = self.declare_parameter('dh_alpha', [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]).value


        self.joint_names = [prefix + j for j in JOINTS]
        self.q = None
        self.axes = [0.0] * 6
        self.last_joy_time = None

        self.jog_pub = self.create_publisher(JointJog, 'servo_node/delta_joint_cmds', 10)
        self.gripper = ActionClient(self, GripperCommand, 'robotiq_gripper_controller/gripper_cmd')
        self.gripper_closed = False
        self.last_buttons = []

        self.get_logger().info('Warte auf servo_node ...')
        servo = self.create_client(ServoCommandType, 'servo_node/switch_command_type')
        servo.wait_for_service()
        servo.call_async(ServoCommandType.Request(command_type=ServoCommandType.Request.JOINT_JOG))

        self.create_subscription(JointState, 'joint_states', self.on_joint_state, 10)
        self.create_subscription(Joy, 'spacenav/joy', self.on_joy, 10)
        self.create_timer(1.0 / rate, self.update)
        self.get_logger().info('Rdy: Spacemouse🪐 bewegt den Arm, MENU Taste oeffnet/schliesst den Greifer')

    def on_joint_state(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in self.joint_names):
            self.q = np.array([pos[n] for n in self.joint_names])

    def on_joy(self, msg):
        self.axes = list(msg.axes[:6])
        self.last_joy_time = self.get_clock().now()
        if msg.buttons and msg.buttons[0] and not (self.last_buttons and self.last_buttons[0]):
            self.toggle_gripper()
        self.last_buttons = list(msg.buttons)

    def desired_twist(self, axes):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self.frame
        mx, my, mz, rx, ry, rz = axes[:6]
        cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z = (self.max_linear * a for a in (-my, mx, mz))
        cmd.twist.angular.x, cmd.twist.angular.y, cmd.twist.angular.z = (self.max_angular * a for a in (-ry, rx, rz))
        return cmd

    def update(self):
        if self.q is None:
            return
    
        stale = (self.last_joy_time is None or (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9 > self.joy_timeout)
        q_dot = np.zeros(6)
        if not stale:
            t = self.desired_twist(self.axes).twist
            v = R_DH_FROM_BASELINK @ np.array([t.linear.x, t.linear.y, t.linear.z])
            w = R_DH_FROM_BASELINK @ np.array([t.angular.x, t.angular.y, t.angular.z])
            x_dot = np.concatenate([v, w])
            if np.any(x_dot):
                frames, p_tcp = forward_kinematics(self.q, self.dh_d, self.dh_a, self.dh_alpha, self.tcp_offset)
                J = geometric_jacobian(frames, p_tcp)
                q_dot, s_min, lam2 = dls_solve(J, x_dot, self.sigma_threshold, self.lambda_max)
                if lam2 > 0.0:
                    self.get_logger().warn(f'Nahe Singularitaet: sigma_min={s_min:.4f}', throttle_duration_sec=1.0)
                peak = np.max(np.abs(q_dot))
                if peak > self.max_joint_vel:       
                    q_dot *= self.max_joint_vel / peak

        cmd = JointJog()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.joint_names = self.joint_names
        cmd.velocities = [float(x) for x in q_dot]
        self.jog_pub.publish(cmd)

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