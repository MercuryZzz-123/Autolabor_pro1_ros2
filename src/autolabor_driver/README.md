# autolabor_driver

Hardware driver package.

Planned scope:

- Port the Autolabor Pro1 chassis serial driver from ROS 1 to ROS 2.
- Publish wheel odometry, battery, current, and voltage topics.
- Subscribe to `cmd_vel` and send speed commands to the chassis.
- Integrate lidar drivers or wrappers needed by the platform.
