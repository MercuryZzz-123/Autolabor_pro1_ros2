# autolabor_mapping

Mapping package for Autolabor Pro1.

This package provides launch files and robot-specific configuration for LiDAR-inertial mapping.
FAST-LIO is kept as a git submodule under `third_party/FAST_LIO` so the mapping stack can be built reproducibly when the external SDK dependencies are installed.

## Supported Backends

```text
FAST-LIO:
  mid360
  ouster_os1_128

LIO-SAM:
  ouster_os1_128
```

Mid-360 with LIO-SAM is not a primary route for now. FAST-LIO is the preferred backend for Mid-360.

## Launch

FAST-LIO with Livox Mid-360:

```bash
ros2 launch autolabor_mapping fast_lio_mid360.launch.py
```

FAST-LIO with Ouster OS1-128:

```bash
ros2 launch autolabor_mapping fast_lio_ouster.launch.py
```

LIO-SAM with Ouster OS1-128:

```bash
ros2 launch autolabor_mapping lio_sam_ouster.launch.py
```

Unified entry:

```bash
ros2 launch autolabor_mapping mapping.launch.py slam:=fast_lio sensor:=mid360
ros2 launch autolabor_mapping mapping.launch.py slam:=fast_lio sensor:=ouster_os1_128
ros2 launch autolabor_mapping mapping.launch.py slam:=lio_sam sensor:=ouster_os1_128
```

## FAST-LIO Source

FAST-LIO is pinned as:

```text
repo: https://github.com/hku-mars/FAST_LIO.git
branch: ROS2
commit: a4743b095409588842a5b30ddfa27e29d2f99164
path: src/autolabor_mapping/third_party/FAST_LIO
```

Initialize it together with recursive dependencies:

```bash
git submodule update --init --recursive
```

Build with FAST-LIO and Livox support:

```bash
scripts/build.sh --with-fast-lio
```

The default `scripts/build.sh` keeps FAST-LIO out of the build because it depends on Livox SDK2 and `livox_ros_driver2`.
FAST-LIO also needs ROS PCL packages such as `pcl_ros` and `pcl_conversions`; on Humble these are usually installed with:

```bash
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions
```

## External Packages

`lio_sam` is still treated as an external package if the LIO-SAM launch path is used.
