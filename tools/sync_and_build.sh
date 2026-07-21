#!/usr/bin/env bash
# -----------------------------------------------------------------------------
#  Sync this package from the Windows drive into the WSL filesystem, then build.
#
#  Why not build in place on /mnt/c ?
#    /mnt/c is drvfs, a Windows-filesystem bridge. colcon does thousands of small
#    file operations and drvfs makes each one a cross-OS round trip, so a build
#    that takes 5 s on the WSL ext4 disk can take several minutes on /mnt/c.
#    Authoring stays on the Windows side (so Explorer and Windows editors see it);
#    only the build happens on the Linux side.
#
#  Usage:
#      ./tools/sync_and_build.sh                # sync + build
#      ./tools/sync_and_build.sh --run          # sync + build + launch
#      WS=~/my_ws ./tools/sync_and_build.sh     # different colcon workspace
# -----------------------------------------------------------------------------
set -euo pipefail

PKG=ika_parkur_gazebo
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${WS:-$HOME/ros2_ws}"
ROS_DISTRO_DEFAULT=jazzy

echo "source : $SRC_DIR"
echo "target : $WS/src/$PKG"

if [[ "$SRC_DIR" != /mnt/* ]]; then
  echo "note   : source is already on the Linux filesystem, syncing anyway"
fi

# -- preflight ----------------------------------------------------------------
# Fail early with a copy-pasteable fix rather than dying mid-build.
DISTRO="${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}"
missing_apt=()

command -v rsync  >/dev/null || missing_apt+=("rsync")
command -v colcon >/dev/null || missing_apt+=("python3-colcon-common-extensions")
python3 -c "import yaml" 2>/dev/null || missing_apt+=("python3-yaml")

[[ -d "/opt/ros/$DISTRO" ]] || {
  echo "ERROR: /opt/ros/$DISTRO not found. Is ROS 2 $DISTRO installed in this WSL distro?"
  exit 1
}
[[ -d "/opt/ros/$DISTRO/share/ros_gz_sim"    ]] || missing_apt+=("ros-$DISTRO-ros-gz-sim")
[[ -d "/opt/ros/$DISTRO/share/ros_gz_bridge" ]] || missing_apt+=("ros-$DISTRO-ros-gz-bridge")
command -v gz >/dev/null || missing_apt+=("gz-harmonic")

if (( ${#missing_apt[@]} )); then
  echo
  echo "missing dependencies. run this, then try again:"
  echo "  sudo apt update && sudo apt install -y ${missing_apt[*]}"
  echo
  exit 1
fi
echo "preflight: ok"

mkdir -p "$WS/src"

# --delete keeps the target an exact mirror, so files removed from the generator
# do not linger in the build. Caches and build artefacts are never copied.
rsync -a --delete \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  --exclude '.git/' \
  --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  "$SRC_DIR/" "$WS/src/$PKG/"

# Regenerate the world on the Linux side so the .sdf always matches config.yaml.
( cd "$WS/src/$PKG/generator" && python3 generate.py )

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}/setup.bash"

cd "$WS"

# Wipe this package's build and install trees first.
#
# Gazebo loads the world from install/, never from src/. If an earlier build ran
# without --symlink-install, install/ holds a real COPY of the world, and colcon
# will happily leave that stale copy in place -- so you regenerate the parkur,
# rebuild, launch, and stare at the previous version. The package is a few
# hundred KB of data files, so rebuilding from scratch costs nothing.
rm -rf "$WS/build/$PKG" "$WS/install/$PKG"

colcon build --packages-select "$PKG" --symlink-install
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

echo
echo "built. to run:"
echo "  source $WS/install/setup.bash"
echo "  ros2 launch $PKG parkur.launch.py"

if [[ "${1:-}" == "--run" ]]; then
  exec ros2 launch "$PKG" parkur.launch.py
fi
