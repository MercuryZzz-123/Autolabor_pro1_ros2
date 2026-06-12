#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
WITH_LIVOX=0
WITH_FAST_LIO=0
CLEAN=0
COLCON_ARGS=()

usage() {
  cat <<'EOF'
Usage: scripts/build.sh [options] [-- additional colcon args]

Options:
  --with-livox          Build livox_ros_driver2 together with this workspace.
  --with-fast-lio       Build FAST-LIO from autolabor_mapping/third_party. Implies --with-livox.
  --clean               Remove build/, install/, and log/ before building.
  --ros-distro DISTRO   ROS 2 distro to source. Default: $ROS_DISTRO or humble.
  -h, --help            Show this help message.

Examples:
  scripts/build.sh
  scripts/build.sh --clean
  scripts/build.sh --with-livox
  scripts/build.sh --with-fast-lio
  scripts/build.sh --with-livox -- --packages-select livox_ros_driver2
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-livox)
      WITH_LIVOX=1
      shift
      ;;
    --with-fast-lio)
      WITH_FAST_LIO=1
      WITH_LIVOX=1
      shift
      ;;
    --clean)
      CLEAN=1
      shift
      ;;
    --ros-distro)
      if [[ $# -lt 2 ]]; then
        echo "error: --ros-distro requires a value" >&2
        exit 2
      fi
      ROS_DISTRO_NAME="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      COLCON_ARGS+=("$@")
      break
      ;;
    *)
      COLCON_ARGS+=("$1")
      shift
      ;;
  esac
done

ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "error: ROS setup file not found: ${ROS_SETUP}" >&2
  exit 1
fi

set +u
source "${ROS_SETUP}"
set -u
cd "${ROOT_DIR}"

if [[ "${CLEAN}" -eq 1 ]]; then
  rm -rf build install log
fi

BASE_PATHS=(src)
if [[ "${WITH_FAST_LIO}" -eq 1 ]]; then
  FAST_LIO_DIR="${ROOT_DIR}/src/autolabor_mapping/third_party/FAST_LIO"
  if [[ ! -d "${FAST_LIO_DIR}" ]]; then
    echo "error: FAST_LIO submodule is missing." >&2
    echo "run: git submodule update --init --recursive" >&2
    exit 1
  fi
  if [[ ! -f "${FAST_LIO_DIR}/include/ikd-Tree/ikd_Tree.cpp" ]]; then
    echo "error: FAST_LIO recursive submodules are missing." >&2
    echo "run: git submodule update --init --recursive" >&2
    exit 1
  fi
  for ROS_PACKAGE in pcl_ros pcl_conversions; do
    if ! ros2 pkg prefix "${ROS_PACKAGE}" >/dev/null 2>&1; then
      echo "error: ROS package '${ROS_PACKAGE}' was not found." >&2
      echo "install FAST-LIO ROS dependencies, for example: sudo apt install ros-${ROS_DISTRO_NAME}-${ROS_PACKAGE//_/-}" >&2
      exit 1
    fi
  done
  BASE_PATHS+=(src/autolabor_mapping/third_party)
fi

if [[ "${WITH_LIVOX}" -eq 1 ]]; then
  LIVOX_DIR="${ROOT_DIR}/src/livox_ros_driver2"
  if [[ ! -d "${LIVOX_DIR}" ]]; then
    echo "error: livox_ros_driver2 submodule is missing." >&2
    echo "run: git submodule update --init --recursive" >&2
    exit 1
  fi
  if [[ ! -f "/usr/local/lib/liblivox_lidar_sdk_shared.so" ]]; then
    echo "error: Livox SDK2 shared library was not found in /usr/local/lib." >&2
    echo "install Livox-SDK2 before building livox_ros_driver2." >&2
    exit 1
  fi
  if [[ ! -f "/usr/local/include/livox_lidar_api.h" ]]; then
    echo "error: Livox SDK2 headers were not found in /usr/local/include." >&2
    echo "install Livox-SDK2 before building livox_ros_driver2." >&2
    exit 1
  fi

  cp -f "${LIVOX_DIR}/package_ROS2.xml" "${LIVOX_DIR}/package.xml"

  colcon build \
    --symlink-install \
    --base-paths "${BASE_PATHS[@]}" \
    "${COLCON_ARGS[@]}" \
    --cmake-args \
    -DROS_EDITION=ROS2 \
    -DDISTRO_ROS="${ROS_DISTRO_NAME}"
else
  colcon build \
    --symlink-install \
    --base-paths "${BASE_PATHS[@]}" \
    --packages-skip livox_ros_driver2 \
    "${COLCON_ARGS[@]}"
fi
