#!/usr/bin/env bash
# Install only the missing ROS 2 Jazzy packages used by this project.
# Default mode is a read-only APT simulation; installation is explicit.
set -euo pipefail

mode=dry-run
with_pointcloud=false
with_amcl=false
ros_setup="${HERMES_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
for arg in "$@"; do
  case "$arg" in
    --ros-setup=*) ros_setup="${arg#*=}" ;;
    --dry-run) mode=dry-run ;;
    --install) mode=install ;;
    --with-pointcloud-to-laserscan) with_pointcloud=true ;;
    --with-amcl) with_amcl=true ;;
    -h|--help)
      cat <<'USAGE'
Usage: bootstrap_ros2.sh [--dry-run|--install] [--with-amcl] [--with-pointcloud-to-laserscan] [--ros-setup=/path/setup.bash]

Default: simulate installing missing RViz2/Nav2 prerequisites without changing
packages. Run --install as root only after reviewing the simulation.

The default profile uses this project's launch files and simulation-truth
localization. It excludes the navigation2/nav2-bringup meta packages and their
Gazebo/TurtleBot simulation dependencies. AMCL is optional. The pointcloud
adapter is needed only when converting PointCloud2 to LaserScan.

This script uses existing APT sources/indexes. It does not add repositories,
run apt update, remove packages, or modify ROS shell startup files. Installed
Mesa/GBM/NVIDIA packages are kept at their existing versions using temporary APT
preferences. Other dependencies may require upgrades; review the plan first.
USAGE
      exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$arg" >&2; exit 2 ;;
  esac
done

source /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 24.04 ]]; then
  printf 'Expected Ubuntu 24.04 for this Jazzy bootstrap; found %s.\n' "${PRETTY_NAME:-unknown}" >&2
  exit 2
fi
if [[ ! -f "$ros_setup" ]]; then
  printf 'Missing ROS setup: %s; inspect the ROS installation first.\n' "$ros_setup" >&2
  exit 2
fi

required=(
  ros-jazzy-rviz2
  ros-jazzy-nav2-map-server
  ros-jazzy-nav2-controller
  ros-jazzy-nav2-planner
  ros-jazzy-nav2-bt-navigator
  ros-jazzy-nav2-behaviors
  ros-jazzy-nav2-lifecycle-manager
  ros-jazzy-nav2-navfn-planner
  ros-jazzy-nav2-dwb-controller
  ros-jazzy-nav2-velocity-smoother
  ros-jazzy-nav2-simple-commander
  ros-jazzy-tf2-tools
  ros-jazzy-robot-state-publisher
)
if "$with_pointcloud"; then
  required+=(ros-jazzy-pointcloud-to-laserscan)
fi
if "$with_amcl"; then
  required+=(ros-jazzy-nav2-amcl)
fi

missing=()
for package in "${required[@]}"; do
  status=$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)
  if [[ "$status" == 'install ok installed' ]]; then
    printf 'Already installed: %s\n' "$package"
  else
    missing+=("$package")
  fi
done
if [[ ${#missing[@]} -eq 0 ]]; then
  printf 'All requested packages are already installed.\n'
  exit 0
fi

printf 'Missing packages:'
printf ' %s' "${missing[@]}"
printf '\n'
if [[ "$mode" == install && "$EUID" -ne 0 ]]; then
  printf 'Installation requires root. Review --dry-run, then run:\n' >&2
  printf '  sudo bash %q --install' "$(readlink -f "$0")" >&2
  if "$with_pointcloud"; then printf ' --with-pointcloud-to-laserscan' >&2; fi
  if "$with_amcl"; then printf ' --with-amcl' >&2; fi
  printf '\n' >&2
  exit 3
fi

# Avoid altering APT manual/automatic marks by passing installed graphics
# packages as explicit installation targets. Temporary pins constrain them
# without changing /etc/apt/preferences or /etc/apt/preferences.d.
prefs_file=$(mktemp "${TMPDIR:-/tmp}/hermes-sim-apt-preferences.XXXXXX")
trap 'rm -f -- "$prefs_file"' EXIT
python3 - "$prefs_file" <<'PY'
from pathlib import Path
import subprocess
import sys

rows = subprocess.check_output(
    ['dpkg-query', '-W', '-f=${binary:Package}\t${Version}\t${Status}\n'],
    text=True,
).splitlines()
pins = []
count = 0
for row in rows:
    package, version, status = row.split('\t', 2)
    graphics = ('mesa' in package or package.startswith(('libgbm', 'nvidia', 'libnvidia')))
    if status != 'install ok installed' or not graphics:
        continue
    count += 1
    pins.append(f'Package: {package}\nPin: version {version}\nPin-Priority: 1001\n')
    pins.append(f'Package: {package}\nPin: version *\nPin-Priority: -1\n')
existing = Path('/etc/apt/preferences')
if existing.exists():
    pins.append(existing.read_text())
Path(sys.argv[1]).write_text('\n'.join(pins))
print(f'Preserving installed Mesa/GBM/NVIDIA versions: {count} packages')
PY
apt_args=(-o "Dir::Etc::preferences=$prefs_file" --no-remove --no-install-recommends install "${missing[@]}")
if [[ "$mode" == dry-run ]]; then
  apt-get --simulate "${apt_args[@]}"
else
  apt-get "${apt_args[@]}"
fi
