#!/bin/bash
# ==============================================================================
#  ROS2 Humble Full Stack Installer
#  Target: Jetson Orin Nano — Ubuntu 22.04 (Jammy)
#
#  Installs:
#    - ROS2 Humble Desktop + common dev tools
#    - Intel RealSense2 SDK + ROS2 camera node
#    - Web Video Server
#    - MAVROS + GeographicLib datasets
#    - OpenVINS (built from source in ~/ros2_ws)
# ==============================================================================

set -e  # Exit on any error

# ── Colors ────────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

log()     { echo -e "${GREEN}[✔ INFO]${NC}  $1"; }
warn()    { echo -e "${YELLOW}[⚠ WARN]${NC}  $1"; }
error()   { echo -e "${RED}[✘ ERROR]${NC} $1"; exit 1; }
section() { echo -e "\n${CYAN}${BOLD}══════════════════════════════════════════${NC}"; \
            echo -e "${CYAN}${BOLD}  $1${NC}"; \
            echo -e "${CYAN}${BOLD}══════════════════════════════════════════${NC}\n"; }

# ── Sanity checks ─────────────────────────────────────────────────────────────
if [ "$EUID" -eq 0 ]; then
    error "Do not run this script as root. Run as a normal user with sudo privileges."
fi

if ! grep -q "22.04" /etc/os-release 2>/dev/null; then
    warn "This script targets Ubuntu 22.04. Your OS may differ — proceed at your own risk."
fi

WORKSPACE="${ROS2_WS:-$HOME/ros2_ws}"
BASHRC="$HOME/.bashrc"

# ══════════════════════════════════════════════════════════════════════════════
section "1 / 6 — System Update & Base Dependencies"
# ══════════════════════════════════════════════════════════════════════════════

log "Updating package lists and upgrading system..."
sudo apt update && sudo apt upgrade -y

log "Installing base build tools and utilities..."
sudo apt install -y \
    curl wget gnupg2 lsb-release ca-certificates \
    build-essential cmake git ninja-build \
    python3-pip python3-dev python3-setuptools \
    software-properties-common \
    libeigen3-dev libboost-all-dev \
    libopencv-dev libpcl-dev \
    locales

# Locale setup (required for ROS2)
log "Configuring locale..."
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ══════════════════════════════════════════════════════════════════════════════
section "2 / 6 — ROS2 Humble"
# ══════════════════════════════════════════════════════════════════════════════

if [ -d "/opt/ros/humble" ]; then
    warn "ROS2 Humble already found at /opt/ros/humble — skipping install."
else
    log "Adding ROS2 apt repository..."
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update

    log "Installing ROS2 Humble Desktop..."
    sudo apt install -y ros-humble-desktop
fi

log "Installing ROS2 dev tools and Python helpers..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    python3-vcstool \
    python3-catkin-pkg \
    ros-dev-tools

log "Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

log "Sourcing ROS2 Humble in .bashrc..."
if ! grep -q "source /opt/ros/humble/setup.bash" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# ROS2 Humble" >> "$BASHRC"
    echo "source /opt/ros/humble/setup.bash" >> "$BASHRC"
fi
source /opt/ros/humble/setup.bash

# ══════════════════════════════════════════════════════════════════════════════
section "3 / 6 — Intel RealSense2 SDK + ROS2 Node"
# ══════════════════════════════════════════════════════════════════════════════

log "Adding Intel RealSense apt repository..."
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp \
    | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] \
    https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/librealsense.list > /dev/null

sudo apt update

log "Installing librealsense2 SDK..."
sudo apt install -y \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg

log "Installing ROS2 RealSense2 camera node..."
sudo apt install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description

# ══════════════════════════════════════════════════════════════════════════════
section "4 / 6 — Web Video Server"
# ══════════════════════════════════════════════════════════════════════════════

log "Installing Web Video Server and image transport plugins..."
sudo apt install -y \
    ros-humble-web-video-server \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-cv-bridge

# ══════════════════════════════════════════════════════════════════════════════
section "5 / 6 — MAVROS"
# ══════════════════════════════════════════════════════════════════════════════

log "Installing MAVROS and extras..."
sudo apt install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-mavros-msgs

log "Installing GeographicLib datasets (required by MAVROS)..."
GEOLIB_SCRIPT="/opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh"
if [ -f "$GEOLIB_SCRIPT" ]; then
    sudo bash "$GEOLIB_SCRIPT"
else
    warn "GeographicLib install script not found at expected path."
    warn "Run manually: sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh"
fi

# ══════════════════════════════════════════════════════════════════════════════
section "6 / 6 — OpenVINS (Build from Source)"
# ══════════════════════════════════════════════════════════════════════════════

log "Installing OpenVINS extra dependencies..."
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-sensor-msgs \
    ros-humble-diagnostic-updater \
    ros-humble-camera-info-manager \
    libsuitesparse-dev \
    libyaml-cpp-dev

log "Setting up ROS2 workspace at: $WORKSPACE"
mkdir -p "$WORKSPACE/src"

cd "$WORKSPACE/src"
if [ -d "open_vins" ]; then
    warn "OpenVINS already cloned — pulling latest changes..."
    git -C open_vins pull
else
    log "Cloning OpenVINS..."
    git clone https://github.com/rpng/open_vins.git
fi

log "Running rosdep for workspace dependencies..."
cd "$WORKSPACE"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

log "Building OpenVINS (this may take several minutes on Jetson)..."
colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers "$(nproc)"

log "Sourcing workspace in .bashrc..."
WORKSPACE_SOURCE="source $WORKSPACE/install/setup.bash"
if ! grep -q "$WORKSPACE_SOURCE" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# ROS2 Workspace" >> "$BASHRC"
    echo "$WORKSPACE_SOURCE" >> "$BASHRC"
fi

# ══════════════════════════════════════════════════════════════════════════════
section "✅ Installation Complete!"
# ══════════════════════════════════════════════════════════════════════════════

echo -e "${GREEN}${BOLD}"
echo "  Installed components:"
echo "    ✔  ROS2 Humble Desktop"
echo "    ✔  Intel RealSense2 SDK + ros2 camera node"
echo "    ✔  Web Video Server"
echo "    ✔  MAVROS + GeographicLib datasets"
echo "    ✔  OpenVINS (built at $WORKSPACE)"
echo ""
echo "  Next steps:"
echo "    1. Run:  source ~/.bashrc"
echo "    2. Test RealSense:  ros2 launch realsense2_camera rs_launch.py"
echo "    3. Test MAVROS:     ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:57600"
echo "    4. Test OpenVINS:   ros2 launch ov_msckf subscribe.launch.py"
echo -e "${NC}"
