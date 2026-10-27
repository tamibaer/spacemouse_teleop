# Robot Table Spacemouse Teleoperation

Spacemouse 🪐 → **Damped Least Squares (DLS)** → MoveIt Servo (`JointJog`) → `forward_position_controller` (UR5e).
Die MENU-Taste öffnet/schließt den Robotiq-Greifer.


## Einmalige Einrichtung

Der Launch startet/stoppt `spacenavd` per `sudo -n`. Damit das ohne Passwort geht:

```bash
sudo install -m 0440 $(ros2 pkg prefix robot_table_spacemouse_teleop)/share/robot_table_spacemouse_teleop/config/spacenavd.sudoers /etc/sudoers.d/spacenavd
sudo visudo -c
```

## Start

```bash
ros2 launch robot_table_control start_robot.launch.py use_mock_hardware:=false 
```

```bash
ros2 launch robot_table_spacemouse_teleop spacemouse_teleop.launch.py
```

Beim Start wird auf `forward_position_controller` umgeschaltet, beim Beenden zurück auf `scaled_joint_trajectory_controller`.
