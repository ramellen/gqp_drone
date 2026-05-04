gqp_drone
Autonomous quadrotor that tracks a human face with vision-based control. The
drone uses an onboard camera to detect a face, computes the normalized error
between the face centroid and the frame center, and drives a
proportional–derivative (PD) velocity controller to keep the face centered at
a target standoff distance — entirely from onboard compute, with no external
infrastructure required at runtime.
The same software stack runs in two contexts:

Simulation — Gazebo Harmonic + ArduPilot SITL on a Linux/WSL development host. Used for full mission-FSM and controller validation.
Real hardware — Raspberry Pi 4 companion computer paired with an Aero Selfie H743 flight controller, an Arducam 5MP camera (OV5647 sensor), a four-in-one ESC, and a 6S LiPo battery.

Only the source of camera frames and the destination of velocity commands
differ between the two contexts; the five ROS 2 nodes themselves are
unchanged.

System Architecture
Five ROS 2 (Humble) nodes communicating over a fixed topic graph:
  Image source  ─── /camera/image_raw ───>  face_detect_node
 (Gazebo or                                       │
  Pi camera)                                      │
                                          /face_tracking/error
                                                  │
                                                  v
   mission_node ── /mission/*_enabled ─>  flight_ctrl_node
        ^                                         │
        │                                /mavros/.../cmd_vel
        │                                         │
        └───────── /mavros/state ──── MAVROS <────┘
                                       │
                                  MAVLink 2
                                       │
                                       v
                                 ArduPilot Copter
                              (SITL or H743 hardware)
NodeRolecamera_nodePublishes camera frames as sensor_msgs/Image (libcamera/picamera2 backend on Pi; OpenCV/MJPEG fallback for dev).face_detect_nodeYuNet ONNX face detection (Haar cascade fallback). Publishes normalized (x, y, area) error tuple.flight_ctrl_nodePer-axis PD controller; produces velocity setpoints for MAVROS at 20 Hz.mission_nodeFinite-state machine: INIT → WAIT_FCU → SET_GUIDED → ARM → TAKEOFF → HOVER → SEARCH ↔ TRACKING → LAND → DONE.plot_errors_nodeOffline utility: records the error/state topics during a run and renders a 3-panel matplotlib plot at shutdown.

Repository Layout
gqp_drone/
├── README.md                     # this file
├── package.xml                   # ROS 2 package manifest
├── setup.py                      # Python package entry points
├── setup.cfg
├── resource/                     # ROS 2 ament resource marker
├── drone_face_tracking/          # Python source
│   ├── camera_node.py
│   ├── face_detect_node.py
│   ├── flight_ctrl_node.py
│   ├── mission_node.py
│   └── plot_errors_node.py
├── launch/
│   ├── simulation.launch.py      # Gazebo + SITL + all nodes
│   └── hardware.launch.py        # Pi-side launch (no sim)
├── config/
│   ├── mavros_params.yaml        # MAVROS config for SITL
│   └── mavros_hardware_params.yaml  # MAVROS config for serial UART
├── worlds/                       # Gazebo world + face texture
├── models/                       # SDF models
├── docs/
│   ├── raspberry_pi_setup.md     # End-to-end Pi bring-up guide
│   └── report_sections.tex       # LaTeX report (intro/methods/results)
└── setup_ros2_workspace.sh       # Convenience installer for the dev host

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



Outstanding

TELEM2 UART link: wiring is in place and the FC is configured for MAVLink 2 at 57600 baud, but mavproxy does not yet acknowledge heartbeats from the Pi. The most likely cause is a wiring fault from cable preparation (one conductor was cut to remove the +5V line, but post-cut continuity was not verified before deployment, raising the possibility that a UART or GND line was severed by mistake). Resolution requires a multimeter check of the remaining conductors under power, then either an in-place splice or a fresh JST-GH pigtail.

Future Work
After the UART link is verified:

End-to-end mission execution on the physical vehicle (bench-test arming with props off, then short hover, then full mission).
Re-tuning the PD gains against the physical platform's dynamics (sim gains are conservative starting points, not flight-ready values).
Characterization against disturbances (wind gusts, target motion).
Optional: telemetry radio on TELEM1 for ground-station monitoring during flight.


Documentation

docs/raspberry_pi_setup.md — End-to-end Pi setup: flashing Ubuntu, installing ROS 2 + MAVROS + libcamera, wiring the H743, ArduPilot SERIAL configuration, link verification with mavproxy and MAVROS, troubleshooting cheat sheet.
docs/report_sections.tex — Capstone report sections (Introduction, Setup & Materials, Methods, Results & Discussion, Hardware Implementation, Conclusion) with TikZ diagrams for the runtime topology and mission FSM, IEEE math notation for the controller equations, and BibTeX entries for the references.


Visualizing the Camera Feed
For headless development on the Pi, run web_video_server to stream the
camera (and detector overlays) to any browser on the same network:
bashsudo apt install -y ros-humble-web-video-server
ros2 run web_video_server web_video_server
Then open http://<pi-host>:8080/stream?topic=/camera/image_raw (or
/face_tracking/debug_image to see frames with detection annotations).

License
MIT. See LICENSE for full text.

Acknowledgments

ArduPilot for the open-source autopilot stack.
ROS 2 and the MAVROS maintainers for the middleware bridge.
OpenCV and the YuNet authors for the face detector.
The picamera2 and libcamera projects for the Pi camera stack.
