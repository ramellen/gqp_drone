# Raspberry Pi 4 Companion Computer Setup

End-to-end setup guide for bringing the `drone_face_tracking` project up on a
Raspberry Pi 4 paired with an Aeroselfie H743 flight controller.

**Target stack:** Ubuntu Server 22.04.x LTS (64-bit) + ROS 2 Humble + MAVROS +
libcamera (for Pi Camera Module). This matches the WSL development environment
1:1, so the workspace builds the same way on both.

---

## 0. Decision summary (what we're building toward)

| Item | Choice | Why |
|------|--------|-----|
| Pi OS | Ubuntu Server 22.04.x 64-bit | First-class ROS 2 platform; matches WSL |
| ROS 2 distro | Humble Hawksbill | LTS until 2027; matches WSL |
| Pi ↔ FC link | UART/GPIO @ 921600 baud | Standard companion-computer pattern; frees H743 USB |
| Camera | Pi Camera Module via libcamera/picamera2 | Native CSI, low latency, no USB bandwidth used |
| Camera publisher | `camera_node.py` (in-repo, picamera2 backend) | Already in the project; one less moving part |

---

## 1. Flash Ubuntu Server 22.04 to the SD card

Do this on your laptop (the existing Pi OS image will be wiped).

1. Install **Raspberry Pi Imager** from <https://www.raspberrypi.com/software/>.
2. Insert the Pi's microSD card into your laptop (16 GB minimum, 32 GB+ recommended).
3. In Imager:
   - **Choose Device:** Raspberry Pi 4
   - **Choose OS:** *Other general-purpose OS* → *Ubuntu* → **Ubuntu Server 22.04.x LTS (64-bit)**
   - **Choose Storage:** the SD card
4. Click the gear icon (or *Edit Settings* on newer versions) and set:
   - Hostname: `drone-pi` (or anything you like)
   - Enable SSH, password authentication
   - Username: `rmellen`, password: *(your choice)*
   - Configure Wi-Fi: SSID, password, country
   - Locale + keyboard
5. Write, eject, insert into the Pi, power on.

First boot takes 1–3 minutes (cloud-init expands the filesystem).

---

## 2. First boot, SSH in, base updates

From your laptop / WSL terminal:

```bash
ssh rmellen@drone-pi.local
```

If `.local` mDNS doesn't resolve, find the IP from your router and use that.

Once in:

```bash
sudo apt update
sudo apt full-upgrade -y
sudo apt install -y build-essential git curl wget vim tmux python3-pip python3-venv
sudo reboot
```

Reconnect after the reboot.

---

## 3. Install ROS 2 Humble

These are the official steps from <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>, condensed.

```bash
# Add ROS 2 apt source
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble (ros-base = no GUI; the Pi is headless)
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-ros-base ros-dev-tools

# Auto-source on every shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Sanity check
ros2 topic list   # should print /parameter_events  /rosout
```

---

## 4. Install MAVROS + GeographicLib datasets

```bash
sudo apt install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-mavros-msgs \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-image-transport

# GeographicLib datasets (~30 MB, geoid model used for altitude conversions)
sudo bash /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

---

## 5. Install Pi Camera stack (libcamera + picamera2)

Ubuntu Server 22.04 doesn't ship picamera2 by default, but it's available
through the Raspberry Pi PPA.

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository ppa:rpi-distro/ppa -y
sudo apt update

# libcamera tooling (needed even if you only use Python bindings)
sudo apt install -y libcamera-apps libcamera-tools

# Python bindings + picamera2
sudo apt install -y python3-picamera2 python3-libcamera

# Quick smoke test (writes a still to /tmp/test.jpg)
rpicam-still -o /tmp/test.jpg --immediate
```

If `rpicam-still` errors with "no cameras available", check that the camera
ribbon is seated correctly and that the PPA package versions match (the rpi
PPA sometimes lags Pi OS by a release).

> **Alternative:** if you'd rather avoid picamera2, the
> [christianrauch/camera_ros](https://github.com/christianrauch/camera_ros)
> driver wraps libcamera as a ROS 2 node directly. It builds with colcon but
> needs `libcamera-dev`. Stick with picamera2 unless you hit a wall.

---

## 6. Install OpenCV + Python deps

```bash
sudo apt install -y python3-opencv python3-numpy
pip3 install --user matplotlib   # only needed if you want plot_errors_node on the Pi
```

---

## 7. Set up the colcon workspace

You have two options for getting the project onto the Pi.

### Option A — git clone (clean, recommended if the repo is on GitHub)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <your-repo-url> drone_face_tracking
```

### Option B — rsync from WSL (fast, no git needed)

From your WSL terminal:

```bash
rsync -avz --exclude='build' --exclude='install' --exclude='log' \
    ~/ros2_ws/src/drone_face_tracking/ \
    rmellen@drone-pi.local:~/ros2_ws/src/drone_face_tracking/
```

### Then build (on the Pi)

```bash
cd ~/ros2_ws

# Pull in any missing rosdeps
sudo rosdep init || true        # ok if already initialised
rosdep update
rosdep install --from-paths src -y --ignore-src

colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 7b. Download the YuNet face-detection model

`face_detect_node` looks for an ONNX model at `~/models/face_detection_yunet_2023mar.onnx`
and falls back to OpenCV's Haar cascade if it's missing. YuNet is much more
accurate, so download it:

```bash
mkdir -p ~/models
wget -O ~/models/face_detection_yunet_2023mar.onnx \
    https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx
```

About 230 KB. If this URL ever 404s, browse <https://github.com/opencv/opencv_zoo/tree/main/models/face_detection_yunet>
for the current filename.

---

## 8. Wire the Pi to the H743 (UART/GPIO)

Power off both devices first.

### Pin map

| Pi 4 (40-pin header) | Direction | H743 TELEM port |
|----------------------|-----------|-----------------|
| Pin 8  — GPIO 14 — TXD0 | →         | RX              |
| Pin 10 — GPIO 15 — RXD0 | ←         | TX              |
| Pin 6  — GND            | ↔         | GND             |

**Important rules of the road:**

- **Cross TX↔RX.** The Pi's TX goes to the FC's RX, and vice versa.
- **Connect GND.** Without a common ground the link won't work.
- **Do NOT connect 5V.** The Pi must be powered from its own supply (USB-C
  PSU or BEC). Backfeeding 5V from the H743 TELEM port can brown out the Pi
  or damage the FC's regulator.
- **Use a 4-pin JST-GH** connector if your H743 TELEM port uses it (most
  Aeroselfie boards do). Pre-made Pixhawk-style telemetry-to-Dupont cables
  work — just verify the pinout with a multimeter before plugging in.

A small ASCII diagram (looking at the Pi from above, USB ports facing you):

```
        Pi 4 GPIO header (40 pins)
        ┌───────────────────────────────┐
   3V3  │ 1   2 │ 5V
        │ 3   4 │ 5V
        │ 5   6 │ GND ────────────► H743 GND
TXD0    │ 7   8 │ TXD0 ───────────► H743 RX     ← GPIO 14
        │ 9  10 │ RXD0 ◄─────────── H743 TX     ← GPIO 15
        │ ...
        └───────────────────────────────┘
```

---

## 9. Enable hardware UART on the Pi

By default Ubuntu maps `/dev/serial0` to the mini UART (ttyS0), which is
clock-tied to the CPU and unreliable at 921600. We want the real PL011 UART
(ttyAMA0). The standard fix is to disable Bluetooth, which frees up the PL011.

Edit `/boot/firmware/config.txt` (note: on Ubuntu the path is
`/boot/firmware/config.txt`, **not** `/boot/config.txt` like on Pi OS):

```bash
sudo vim /boot/firmware/config.txt
```

Add at the bottom:

```ini
enable_uart=1
dtoverlay=disable-bt
```

Then disable the serial console so it doesn't hijack the port. Edit
`/boot/firmware/cmdline.txt` and remove any `console=serial0,115200` or
`console=ttyAMA0,115200` token (leave the rest of the line alone, all on one
line):

```bash
sudo vim /boot/firmware/cmdline.txt
```

Disable the serial-getty service and the BT-init service:

```bash
sudo systemctl disable --now serial-getty@ttyS0.service
sudo systemctl disable --now hciuart.service
```

Add yourself to the `dialout` group so you can open the serial port without sudo:

```bash
sudo usermod -aG dialout $USER
```

Reboot:

```bash
sudo reboot
```

After reboot, verify:

```bash
ls -l /dev/serial0
# Should be: /dev/serial0 -> ttyAMA0     (NOT ttyS0)
```

If it still points to `ttyS0`, the `disable-bt` overlay didn't take. Re-check
`config.txt` and confirm the file is `/boot/firmware/config.txt`.

---

## 10. Configure the H743 SERIAL parameters

This is a one-time configuration done over USB from your laptop using
**Mission Planner** or **QGroundControl** before you plug into the Pi.

Suppose you wired into TELEM2 on the H743. Set:

| Parameter           | Value | Meaning                                          |
|---------------------|-------|--------------------------------------------------|
| `SERIAL2_PROTOCOL`  | `2`   | MAVLink2                                          |
| `SERIAL2_BAUD`      | `921` | 921600 baud (matches Pi side)                     |
| `BRD_SER2_RTSCTS`   | `0`   | No hardware flow control (we only wired TX/RX/GND)|

If you wired into TELEM1, replace `SERIAL2_*` with `SERIAL1_*`. ArduPilot
serial port numbering: `SERIAL0` = USB, `SERIAL1` = Telem1, `SERIAL2` = Telem2,
etc. Check the Aeroselfie H743 docs to confirm which physical port maps to
which `SERIALn` index — silkscreen labels are usually correct but double-check.

Reboot the FC after writing parameters (or it'll re-read on next power cycle).

Finally, while you're in Mission Planner, also confirm:

- `BRD_SAFETY_DEFLT = 0` (no safety switch press required) — only if your
  build doesn't have a safety button wired.
- `LOG_BACKEND_TYPE = 1` (SD card logging enabled, useful for tuning).

---

## 11. Verify the Pi ↔ FC link

Two quick checks, in order.

### 11a. Raw MAVLink with mavproxy

```bash
pip3 install --user mavproxy
mavproxy.py --master=/dev/serial0 --baudrate=921600
```

You should see heartbeat output within a few seconds:

```
Got HEARTBEAT 0
ARMING_CHECK enabled ...
Mode COPTER ...
```

`Ctrl+]` then `quit` to exit.

If you see "no link" or only your own GCS heartbeats, re-check wiring (TX/RX
crossed, GND connected) and that you set `SERIAL2_PROTOCOL=2` on the FC.

### 11b. MAVROS

The hardware launch file in this repo (`launch/hardware.launch.py`, see
the "Hardware Launch File" section below) handles this for you. To test
MAVROS standalone:

```bash
ros2 launch mavros apm.launch fcu_url:=serial:///dev/serial0:921600
```

In another shell on the Pi:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /mavros/state
```

Look for `connected: true`. That's the green light.

---

## 12. Camera smoke test (ROS 2 side)

After step 5 worked at the libcamera level, confirm the ROS 2 publisher works:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run drone_face_tracking camera_node
```

In another shell:

```bash
ros2 topic hz /camera/image_raw
```

You should see ~30 Hz. If you see "Could not open camera", check that
`python3-picamera2` installed cleanly (`python3 -c 'from picamera2 import Picamera2'`
should not raise). The updated `camera_node.py` autoselects picamera2 on the
Pi and falls back to OpenCV otherwise — see source comments for the
`CAMERA_SOURCE` switch.

---

## 12b. MAVROS topic-path note

This project intentionally targets the **real** MAVROS service paths (which on
this install are `/mavros/mavros/*`, not the top-level `/mavros/*` ghost
paths). That quirk has held across the same MAVROS version on WSL, so it
should hold on the Pi too — but if you ever run `ros2 topic list` on the Pi
and see `/mavros/cmd_vel` instead of `/mavros/mavros/cmd_vel_unstamped`,
that's a sign you have a different MAVROS build and `flight_ctrl_node`'s
hardcoded paths will need updating.

---

## 13. End-to-end run

Once the link and camera are both verified:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch drone_face_tracking hardware.launch.py
```

This launches:

1. `camera_node` — publishes `/camera/image_raw` from Pi camera
2. MAVROS — connects to H743 over `/dev/serial0`
3. `face_detect_node`
4. `flight_ctrl_node`
5. `mission_node`

The mission node will run the same FSM as in sim
(INIT → WAIT_FCU → SET_GUIDED → ARM → TAKEOFF → ...). **Do not run this with
props attached the first time.** Bench-test arming and motor spin-up first.

---

## 14. Troubleshooting cheat sheet

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `/dev/serial0` points to `ttyS0` | Bluetooth not disabled | Re-check `dtoverlay=disable-bt` in `/boot/firmware/config.txt`, reboot |
| MAVROS shows `connected: false` | Wiring or baud mismatch | Verify TX/RX crossed; check `SERIAL2_BAUD=921` on FC |
| `Permission denied: '/dev/serial0'` | User not in `dialout` | `sudo usermod -aG dialout $USER`, log out + back in |
| `picamera2` import fails | rpi PPA not added or missing package | Re-run step 5; verify with `apt show python3-picamera2` |
| Arming rejected: "Pre-arm: ..." | ArduPilot pre-arm checks failing | Open Mission Planner, read the specific check; usually GPS lock or compass calibration |
| `rosdep install` errors on `mavros` | source list not added | Re-run step 3 (ROS 2 apt source) |
| Pi reboots when motors spin | Power supply too weak / shared with FC 5V | Use a separate BEC or USB-C PSU for the Pi; never share the FC 5V rail |

---

## 15. Quality-of-life: auto-source ROS 2 on every login

Already done in steps 3 and 7, but worth confirming. Your `~/.bashrc` should end with:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

For tmux convenience:

```bash
echo "set -g default-terminal 'tmux-256color'" > ~/.tmux.conf
echo "set -g mouse on" >> ~/.tmux.conf
```

---

## 16. (Future) Auto-start on boot

Once everything is reliable, you can set up a systemd unit that launches
`hardware.launch.py` on boot. That's a separate exercise — don't do this until
you've verified end-to-end behavior manually a few times. A starter unit
template:

```ini
# /etc/systemd/system/drone-tracking.service
[Unit]
Description=Drone face tracking
After=network-online.target

[Service]
Type=simple
User=rmellen
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source /home/rmellen/ros2_ws/install/setup.bash && ros2 launch drone_face_tracking hardware.launch.py'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable with `sudo systemctl enable --now drone-tracking.service`.
