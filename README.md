# Robot Table Spacemouse Teleoperation

Spacemouse → MoveIt Servo → `forward_position_controller` (UR5e), linke Taste öffnet/schließt den Robotiq-Greifer.

## Einmalige Einrichtung

Der Launch startet/stoppt `spacenavd` per `sudo -n`. Damit das ohne Passwort geht:

```bash
sudo install -m 0440 $(ros2 pkg prefix robot_table_spacemouse_teleop)/share/robot_table_spacemouse_teleop/config/spacenavd.sudoers /etc/sudoers.d/spacenavd
sudo visudo -c
```

## Start

```bash
# Roboter (oder use_mock_hardware:=true zum Testen)
ros2 launch robot_table_control start_robot.launch.py use_mock_hardware:=false 
```

```bash
# Teleop
ros2 launch robot_table_spacemouse_teleop spacemouse_teleop.launch.py
```

Beim Start wird auf `forward_position_controller` umgeschaltet, beim Beenden zurück auf `scaled_joint_trajectory_controller`.
