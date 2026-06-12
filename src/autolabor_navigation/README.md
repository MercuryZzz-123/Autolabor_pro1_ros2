# autolabor_navigation

Nav2 configuration for Autolabor Pro1.

## Launch

Generic Nav2 bringup:

```bash
ros2 launch autolabor_navigation navigation.launch.py
```

Mid360 point cloud projected to a 2D scan for AMCL:

```bash
ros2 launch autolabor_navigation mid360_navigation.launch.py
```

Shortcut launch files:

```bash
ros2 launch autolabor_navigation first_generation_navigation.launch.py
ros2 launch autolabor_navigation second_generation_navigation.launch.py
```

The first-generation launch uses `maps/map.yaml` and the single-laser Nav2
parameters. The second-generation launch uses `maps/103huang.yaml` and the
two-laser costmap parameters.

## Notes

- Nav2 is configured with AMCL, map server, Navfn global planner, DWB local
  planner, behavior server, waypoint follower, and velocity smoother.
- `mid360_navigation.launch.py` starts `pointcloud_to_scan_node`, which filters
  `/livox/lidar` by height and publishes `/scan` for AMCL. By default it first
  transforms the input cloud into `base_link` with TF, so the Mid360 extrinsic
  must be published.
- The Mid360 converter subscribes to `sensor_msgs/msg/PointCloud2`. With
  `livox_ros_driver2`, this matches the PointCloud2 mode such as
  `rviz_MID360_launch.py` or `xfer_format: 0`. The Livox custom-message mode
  used by `msg_MID360_launch.py` is not a PointCloud2 topic.
- If FAST-LIO is already running, project a body-frame cloud instead, for
  example `cloud_topic:=/cloud_registered_body target_frame:=base_link`, and
  make sure `body` can transform to `base_link`.
- The ROS1 TEB parameters were not copied directly because Nav2 does not use
  the ROS1 `teb_local_planner` plugin API.
- The two-laser configuration uses `/scan_1` for AMCL and both `/scan_1` and
  `/scan_2` for obstacle costmaps. If localization quality is poor, add a scan
  merger and point AMCL at the merged scan topic.
- Maps copied from the ROS1 workspace have relative `image:` paths so they work
  after installation.
