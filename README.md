gqp_drone
Autonomous quadrotor that tracks a human face with vision-based control. The
drone uses an onboard camera to detect a face, computes the normalized error
between the face centroid and the frame center, and drives a
proportional–derivative (PD) velocity controller to keep the face centered at
a target standoff distance entirely from onboard compute, with no external
infrastructure required at runtime.
The same software stack runs in two contexts:

Simulation: Gazebo Harmonic + ArduPilot SITL on a Linux/WSL development host. Used for full mission-FSM and controller validation.
Real hardware: Raspberry Pi 4 companion computer paired with an Aero Selfie H743 flight controller, an Arducam 5MP camera (OV5647 sensor), a four-in-one ESC, and a 6S LiPo battery.

Only the source of camera frames and the destination of velocity commands
differ between the two contexts; the five ROS 2 nodes themselves are
unchanged.

Quick Start
Option A — Run the Simulation (no hardware required)
Tested on Ubuntu 22.04 (or Ubuntu 22.04 inside WSL2) with ROS 2 Humble.
Prerequisites:

Ubuntu 22.04
ROS 2 Humble (ros-humble-desktop or ros-humble-ros-base)
Gazebo Harmonic
ArduPilot SITL
The ardupilot_gazebo plugin

If you don't have these, the included script installs the ROS-side prerequisites:
bash./setup_ros2_workspace.sh
Build the workspace:
bashcd ~/ros2_ws
colcon build --symlink-install --packages-select drone_face_tracking
source install/setup.bash
Start ArduPilot SITL in one terminal:
bashcd ~/ardupilot/ArduCopter
sim_vehicle.py --console --map
Launch the simulation in another terminal:
bashros2 launch drone_face_tracking simulation.launch.py
Gazebo will spawn the drone in front of a textured face. Within ~30 seconds
the mission FSM will arm, take off, hover briefly, transition into
TRACKING, and hold the face centered for the configured tracking window
before landing autonomously.
To record and plot tracking errors during a run:
bashros2 run drone_face_tracking plot_errors_node
The plot is saved to ~/drone_tracking_plots/ on Ctrl-C.
Option B — Run on Real Hardware
Hardware setup is non-trivial and is documented in detail in
docs/raspberry_pi_setup.md. At a high level:

Flash Ubuntu Server 22.04 64-bit to the Pi 4's SD card.
Install ROS 2 Humble, MAVROS, libcamera, picamera2 on the Pi.
Flash ArduPilot Copter (MatekH743 target) to the Aero Selfie H743 over USB DFU using STM32CubeProgrammer.
Configure SERIAL2_PROTOCOL=2, SERIAL2_BAUD=57 on the FC via Mission Planner.
Wire the Pi GPIO 14/15/GND (pins 8/10/6) to the H743 TELEM2 RX/TX/GND. Do not connect the +5V conductor.
Power the FC from the ESC's BEC and the Pi from a separate 5 V buck converter sourced from the same battery.
On the Pi:

bash   ros2 launch drone_face_tracking hardware.launch.py
Several integration issues encountered during hardware bring-up are
documented in the setup guide and in the report's Hardware Implementation
section, including the Launchpad outage that required building libcamera
from source, the dtoverlay required for the OV5647 sensor on Ubuntu Server,
the user-site NumPy 2.x shadow that breaks cv_bridge, the YuNet model
version compatibility with apt-installed OpenCV 4.5.4, and the DMA-BUF
heap permission requirements.
Bench-test arming with the propellers removed before any flight attempt.

Hardware Bill of Materials
ComponentPart usedNotesFlight controllerAero Selfie H743 (STM32H743VIT6)Firmware-compatible with the MatekH743 ArduPilot targetCompanion computerRaspberry Pi 4 Model B (4 GB)Ubuntu Server 22.04 64-bitCameraArducam 5MP (OV5647 sensor)CSI-2; libcamera + picamera2PropulsionBrushless quad + 4-in-1 ESCESC's BEC powers the FCBattery6S LiPo (22.2 V nominal)Companion powerDC-DC buck converter, 5 V/3 A or higherSeparate from FC's BEC; do not share railsInter-board cablingPi GPIO 14/15/GND ↔ FC TELEM2 RX/TX/GND3 wires; +5V intentionally unused

Project Status
Working

Simulation (end-to-end): All five nodes run in Gazebo + ArduPilot SITL. The mission FSM completes the full INIT → ... → TRACKING → LAND → DONE sequence without manual intervention. The face is held within a few normalized error units of frame center for the duration of the TRACKING window. PD vs P-only controller comparison validates the derivative term as required for safe operation on the forward (standoff) axis.
Hardware (component-wise):

ArduPilot Copter v4.5 flashed and running on the H743.
Pi 4 running Ubuntu Server 22.04, ROS 2 Humble, MAVROS, the full drone_face_tracking workspace.
libcamera v0.7.1 + picamera2 built from source (PyPI was unavailable from the rpi-distro PPA at the time of deployment).
camera_node publishing /camera/image_raw at 30 Hz from the OV5647 sensor.
face_detect_node running YuNet inference on hardware frames.
Flight controller wired to ESC; motors spin under bench test (props off).

Visualizing the Camera Feed
For headless development on the Pi, run web_video_server to stream the
camera (and detector overlays) to any browser on the same network:
bashsudo apt install -y ros-humble-web-video-server
ros2 run web_video_server web_video_server
Then open http://<pi-host>:8080/stream?topic=/camera/image_raw (or
/face_tracking/debug_image to see frames with detection annotations).

Acknowledgments

ArduPilot for the open-source autopilot stack.
ROS 2 and the MAVROS maintainers for the middleware bridge.
OpenCV and the YuNet authors for the face detector.
The picamera2 and libcamera projects for the Pi camera stack.
