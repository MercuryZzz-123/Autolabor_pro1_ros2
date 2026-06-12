# autolabor_tools

Utility package.

## Keyboard Control

ROS 2 port of the ROS 1 `autolabor_keyboard_control` node.

Launch:

```bash
ros2 launch autolabor_tools keyboard_control.launch.py
```

The node searches `/dev/input/by-path/*event-kbd*` by default. A device can be set explicitly:

```bash
ros2 launch autolabor_tools keyboard_control.launch.py port_name:=/dev/input/by-path/<keyboard-event>
```

Key mapping:

- Arrow up/down: forward/backward.
- Arrow left/right: rotate left/right.
- `1` / `2`: increase/decrease linear speed scale.
- `3` / `4`: increase/decrease angular speed scale.
- `9` / `0`: enable/disable `/cmd_vel` publishing.

Default parameters are in `config/keyboard_control.yaml`.

## Planned Scope

- Port calibration helper nodes.
- Add small C++ utilities that support bringup, debugging, and experiments.
- Keep RViz plugins here only if they are specific to this robot stack.
