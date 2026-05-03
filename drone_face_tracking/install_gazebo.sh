#!/bin/bash
# =============================================================================
# install_gazebo.sh
# Installs Gazebo Classic 11, ROS 2 Gazebo bridge packages, and the
# ArduPilot Gazebo plugin — everything needed for SITL + Gazebo + ROS 2.
# Run from any directory. Ubuntu 22.04 + ROS 2 Humble assumed.
# =============================================================================
set -e

ROS_DISTRO="humble"

echo "============================================"
echo " Gazebo Sim Install — ArduPilot + ROS 2"
echo "============================================"

# ── 1. Gazebo Classic 11 ──────────────────────────────────────────────
echo "[1/4] Installing Gazebo Classic 11..."
sudo apt update
sudo apt install -y \
    gazebo \
    libgazebo11-dev \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-gazebo-ros \
    ros-$ROS_DISTRO-gazebo-plugins

echo "Gazebo 11 installed."

# ── 2. ArduPilot Gazebo Plugin ────────────────────────────────────────
# This plugin lets Gazebo act as the physics backend for ArduPilot SITL.
echo "[2/4] Building ArduPilot Gazebo plugin from source..."

sudo apt install -y \
    libgz-sim7-dev \
    rapidjson-dev \
    git \
    cmake \
    build-essential

PLUGIN_DIR="$HOME/ardupilot_gazebo"

if [ ! -d "$PLUGIN_DIR" ]; then
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$PLUGIN_DIR"
else
    echo "ardupilot_gazebo already cloned — pulling latest..."
    git -C "$PLUGIN_DIR" pull
fi

cd "$PLUGIN_DIR"
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
sudo make install

echo "ArduPilot Gazebo plugin installed."

# ── 3. Environment variables ──────────────────────────────────────────
echo "[3/4] Setting up environment variables..."

GAZEBO_ENV_BLOCK='
# ── ArduPilot Gazebo ──────────────────────────────────────────────────
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
# drone_face_tracking models and worlds
export GAZEBO_MODEL_PATH=$HOME/ros2_ws/src/drone_face_tracking/models:$HOME/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=$HOME/ros2_ws/src/drone_face_tracking/worlds:$GAZEBO_RESOURCE_PATH
'

if ! grep -q "ardupilot_gazebo" ~/.bashrc; then
    echo "$GAZEBO_ENV_BLOCK" >> ~/.bashrc
    echo "Environment variables added to ~/.bashrc"
else
    echo "Environment variables already in ~/.bashrc — skipping."
fi

source ~/.bashrc

# ── 4. Extra ROS 2 packages ───────────────────────────────────────────
echo "[4/4] Installing extra ROS 2 packages..."
sudo apt install -y \
    ros-$ROS_DISTRO-mavros \
    ros-$ROS_DISTRO-mavros-extras \
    ros-$ROS_DISTRO-mavros-msgs \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-rqt-image-view

echo ""
echo "============================================"
echo " Install complete!"
echo "============================================"
echo ""
echo "Next: open a new terminal then run:"
echo "  ros2 launch drone_face_tracking simulation.launch.py"
