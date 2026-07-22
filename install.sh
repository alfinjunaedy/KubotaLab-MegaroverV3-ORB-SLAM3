#!/usr/bin/env bash
###############################################################################
# install.sh — megarover-v3-orbslam3
#
# Spreads this repository's files into the ROS 2 workspace layout used by the
# MegaRover V3 + ORB-SLAM3 project:
#
#   config/d435_rgbd.yaml  ->  ~/orb_ws/config/d435_rgbd.yaml
#   map_builder/           ->  ~/orb_ws/src/map_builder/
#   ORB_SLAM3_ROS2/        ->  ~/orb_ws/src/ORB_SLAM3_ROS2/
#   rover_control/         ->  ~/ros2_ws/src/rover_control/
#
# imgs/, README.md, LICENSE and this script stay in the repository.
#
# Usage:
#   ./install.sh              copy files only (idempotent; replaced cleanly)
#   ./install.sh --build      copy + colcon build orb_ws and ros2_ws
#   ./install.sh --help       show this header
#
# Custom locations / ROS distro via environment variables:
#   ORB_WS=~/my_orb_ws ROS2_WS=~/my_ros2_ws ROS_DISTRO=jazzy ./install.sh --build
###############################################################################
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ORB_WS="${ORB_WS:-$HOME/orb_ws}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
ORB_SLAM3_DIR="${ORB_SLAM3_DIR:-$HOME/ORB_SLAM3}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

DO_BUILD=0
case "${1:-}" in
  "") ;;
  --build) DO_BUILD=1 ;;
  -h|--help) sed -n '2,29p' "$0"; exit 0 ;;
  *) echo "Unknown option: '$1'  (valid: --build, --help)" >&2; exit 1 ;;
esac

info() { printf '\033[1;32m[install]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[ warn ]\033[0m %s\n' "$*"; }
err()  { printf '\033[1;31m[ error]\033[0m %s\n' "$*" >&2; }

copy_tree() { # copy_tree <src> <dst>
  if command -v rsync >/dev/null 2>&1; then
    rsync -a "$1" "$2"
  else
    cp -a "$1" "$2"
  fi
}

###############################################################################
# 1. Sanity checks
###############################################################################
info "repo:   $REPO_ROOT"
info "target: ORB_WS=$ORB_WS  ROS2_WS=$ROS2_WS  ROS_DISTRO=$ROS_DISTRO"

MISSING=0
for p in config/d435_rgbd.yaml map_builder ORB_SLAM3_ROS2 rover_control; do
  if [ ! -e "$REPO_ROOT/$p" ]; then
    err "missing from repository: $p"
    MISSING=1
  fi
done
[ "$MISSING" -eq 1 ] && { err "incomplete clone — aborting."; exit 1; }

[ -d "$ORB_SLAM3_DIR" ] \
  || warn "ORB_SLAM3 not found at $ORB_SLAM3_DIR — needed later to build ORB_SLAM3_ROS2"
[ -d "/opt/ros/$ROS_DISTRO" ] \
  || warn "/opt/ros/$ROS_DISTRO not found — install ROS 2 $ROS_DISTRO before --build"

###############################################################################
# 2. Create workspace skeletons and copy files
###############################################################################
mkdir -p "$ORB_WS/config" "$ORB_WS/src" "$ROS2_WS/src"

# --- camera config -----------------------------------------------------------
install -m 0644 "$REPO_ROOT/config/d435_rgbd.yaml" \
                "$ORB_WS/config/d435_rgbd.yaml"
info "config/d435_rgbd.yaml -> $ORB_WS/config/d435_rgbd.yaml"

# --- orb_ws packages (replace cleanly for idempotency) -----------------------
for pkg in map_builder ORB_SLAM3_ROS2; do
  rm -rf "${ORB_WS:?}/src/$pkg"
  copy_tree "$REPO_ROOT/$pkg" "$ORB_WS/src/$pkg"
  info "$pkg/ -> $ORB_WS/src/$pkg/"
done

# --- ros2_ws package ---------------------------------------------------------
rm -rf "${ROS2_WS:?}/src/rover_control"
copy_tree "$REPO_ROOT/rover_control" "$ROS2_WS/src/rover_control"
info "rover_control/ -> $ROS2_WS/src/rover_control/"

# sanity: the launch file and rviz config must have landed where the launch
# file expects them (~/orb_ws/src/map_builder/rviz/map_builder.rviz)
for f in "$ORB_WS/src/map_builder/rviz/map_builder.rviz" \
         "$ROS2_WS/src/rover_control/launch/rover_bringup_launch.py"; do
  [ -e "$f" ] || warn "expected file not found after copy: $f"
done

###############################################################################
# 3. Optional: colcon build both workspaces
###############################################################################
if [ "$DO_BUILD" -eq 1 ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/$ROS_DISTRO/setup.bash"

  info "colcon build: $ORB_WS (Release)"
  (cd "$ORB_WS" && colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release)

  info "colcon build: $ROS2_WS"
  (cd "$ROS2_WS" && colcon build --symlink-install)
fi

###############################################################################
# 4. Summary
###############################################################################
cat <<EOF

$(info "done.")
Workspace layout now:
  $ORB_WS/config/d435_rgbd.yaml
  $ORB_WS/src/map_builder/        (incl. rviz/map_builder.rviz)
  $ORB_WS/src/ORB_SLAM3_ROS2/
  $ROS2_WS/src/rover_control/     (incl. launch/rover_bringup_launch.py)

NOT installed by this script (install separately, see README):
  ~/ORB_SLAM3   ORB-SLAM3 library    (step 1)
  ~/uros_ws     micro-ROS Agent      (step 2)

Next steps:
  source /opt/ros/$ROS_DISTRO/setup.bash
  source ~/uros_ws/install/local_setup.bash
  source $ORB_WS/install/setup.bash
  source $ROS2_WS/install/setup.bash

Run the full system:
  ros2 launch rover_control rover_bringup_launch.py
EOF
