import os
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

TRAJECTORY_CONTROLLER = 'scaled_joint_trajectory_controller'
SERVO_CONTROLLER = 'forward_position_controller'


def switch_controllers(activate, deactivate):
    return ['ros2', 'control', 'switch_controllers', '--activate', activate, '--deactivate', deactivate]


def on_shutdown(event, context):
    # Zurueck auf den Trajektorien-Controller, damit MoveIt-Planung wieder funktioniert
    try:
        subprocess.run(switch_controllers(TRAJECTORY_CONTROLLER, SERVO_CONTROLLER), timeout=10)
    except subprocess.TimeoutExpired:
        print('controller_manager nicht erreichbar, Controller nicht zurueckgeschaltet')
    subprocess.run(['sudo', '-n', 'systemctl', 'stop', 'spacenavd'])


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder('robot_table', package_name='robot_table_moveit_config').to_moveit_configs()
    servo_yaml = os.path.join(get_package_share_directory('robot_table_spacemouse_teleop'), 'config', 'servo.yaml')
    with open(servo_yaml) as f:
        servo_params = {'moveit_servo': yaml.safe_load(f)}

    start_service = ExecuteProcess(cmd=['sudo', '-n', 'systemctl', 'start', 'spacenavd'], output='screen')
    use_servo_controller = ExecuteProcess(cmd=switch_controllers(SERVO_CONTROLLER, TRAJECTORY_CONTROLLER), output='screen')

    servo = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output='screen',
    )
    driver = Node(package='spacenav', executable='spacenav_node', output='screen')
    teleop = Node(package='robot_table_spacemouse_teleop', executable='spacemouse_teleop', output='screen')

    def on_service_started(event, context):
        if event.returncode == 0:
            return [driver, teleop]
        return [
            LogInfo(msg='spacenavd konnte nicht gestartet werden (sudo braucht ein Passwort). '
                        'Einmalig die sudoers-Regel installieren, siehe README.'),
            EmitEvent(event=Shutdown(reason='spacenavd nicht gestartet')),
        ]

    return LaunchDescription([
        start_service,
        use_servo_controller,
        servo,
        RegisterEventHandler(OnProcessExit(target_action=start_service, on_exit=on_service_started)),
        RegisterEventHandler(OnShutdown(on_shutdown=on_shutdown)),
    ])
