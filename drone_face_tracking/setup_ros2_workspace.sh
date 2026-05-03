#!/bin/bash
# =============================================================================
# setup_ros2_workspace.sh
# Run this once on Ubuntu machine to:
#   1. Install ROS 2 Humble (Ubuntu 22.04) or prompt for Jazzy (24.04)
#   2. Install all required dependencies
#   3. Build the drone_face_tracking workspace
# =============================================================================

set -e  # Exit on any error

UBUNTU_VERSION=$(lsb_release -rs)
ROS_DISTRO=""

echo "============================================"
echo " Drone Face Tracking — ROS 2 Setup Script"
echo "============================================"
echo "Detected Ubuntu: $UBUNTU_VERSION"

if [[ "$UBUNTU_VERSION" == "22.04" ]]; then
    ROS_DISTRO="humble"
elif [[ "$UBUNTU_VERSION" == "24.04" ]]; then
    ROS_DISTRO="jazzy"
else
    echo "ERROR: This script supports Ubuntu 22.04 (Humble) or 24.04 (Jazzy)."
    echo "       Your version ($UBUNTU_VERSION) is not supported."
    exit 1
fi

echo "Using ROS 2 distro: $ROS_DISTRO"
echo ""

# ── Step 1: Install ROS 2 if not already installed ────────────────────
if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "[1/5] Installing ROS 2 $ROS_DISTRO..."

    sudo apt update && sudo apt install -y software-properties-common curl

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y ros-$ROS_DISTRO-desktop python3-rosdep python3-colcon-common-extensions
    echo "ROS 2 $ROS_DISTRO installed."
else
    echo "[1/5] ROS 2 $ROS_DISTRO already installed — skipping."
fi

# ── Step 2: Source ROS 2 ──────────────────────────────────────────────
source /opt/ros/$ROS_DISTRO/setup.bash

# Add to .bashrc if not already there
if ! grep -q "source /opt/ros/$ROS_DISTRO/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
    echo "Added ROS 2 source to ~/.bashrc"
fi

# ── Step 3: Install rosdep and dependencies ───────────────────────────
echo "[2/5] Initialising rosdep..."
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rosdep init
fi
rosdep update

# Python and OpenCV deps
echo "[3/5] Installing Python and OpenCV dependencies..."
sudo apt install -y \
    python3-pip \
    python3-opencv \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-vision-msgs \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-geometry-msgs

pip3 install opencv-python numpy --break-system-packages 2>/dev/null || \
pip3 install opencv-python numpy

# MAVROS
echo "[4/5] Installing MAVROS..."
sudo apt install -y \
    ros-$ROS_DISTRO-mavros \
    ros-$ROS_DISTRO-mavros-extras \
    ros-$ROS_DISTRO-mavros-msgs

# Install GeographicLib datasets (required by MAVROS)
sudo /opt/ros/$ROS_DISTRO/lib/mavros/install_geographiclib_datasets.sh || \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && sudo bash install_geographiclib_datasets.sh

# ── Step 4: Build the workspace ───────────────────────────────────────
echo "[5/5] Building the workspace..."

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"  # assumes script lives in ros2_ws/src/

cd "$WS_DIR"
colcon build --symlink-install

# Add workspace overlay to .bashrc
WS_SETUP="source $WS_DIR/install/setup.bash"
if ! grep -q "$WS_SETUP" ~/.bashrc; then
    echo "$WS_SETUP" >> ~/.bashrc
    echo "Added workspace overlay to ~/.bashrc"
fi

echo ""
echo "============================================"
echo " Setup complete!"
echo "============================================"
echo ""
echo "Next steps:"
echo "  1. Open a new terminal (or run: source ~/.bashrc)"
echo "  2. Start SITL:      cd ~/ardupilot/ArduCopter && sim_vehicle.py --console --map"
echo "  3. Start MAVROS:    ros2 launch mavros apm.launch fcu_url:=udp://:14550@localhost:14555"
echo "  4. Launch nodes:"
echo "       ros2 run drone_face_tracking camera_node"
echo "       ros2 run drone_face_tracking face_detect_node"
echo "       ros2 run drone_face_tracking flight_ctrl_node"
echo ""
echo "(Optional) Download YuNet model for better face detection:"
echo "  mkdir -p ~/models"
echo "  wget -O ~/models/face_detection_yunet_2023mar.onnx \\"
echo "    https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx"
