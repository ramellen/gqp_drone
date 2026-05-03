"""
hardware.launch.py
------------------
Launches the drone_face_tracking stack on real hardware (Raspberry Pi 4
companion + Aeroselfie H743 flight controller, connected over UART).

Differences from simulation.launch.py:
  * No Gazebo, no spawn, no gz_bridge.
  * camera_node publishes /camera/image_raw from the Pi Camera (via
    picamera2) instead of being bridged from a Gazebo sensor topic.
  * MAVROS is configured with a serial fcu_url
    (serial:///dev/serial0:921600) via config/mavros_hardware_params.yaml.
  * Looser startup ordering since there is no sim warm-up needed; MAVROS
    just needs to be up before mission_node leaves WAIT_FCU.

The face_detect_node, flight_ctrl_node, and mission_node binaries are
identical to the sim build — they consume / publish the same topics, and
the FSM/PD logic is unchanged.

Run on the Pi after sourcing the workspace:
    ros2 launch drone_face_tracking hardware.launch.py

Optional launch arguments:
    fcu_url:=<override>    (default: serial:///dev/serial0:921600)
    takeoff_altitude:=<m>  (default: 1.5)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg = get_package_share_directory('drone_face_tracking')
    config_file = os.path.join(pkg, 'config', 'mavros_hardware_params.yaml')

    # ── Launch arguments ──────────────────────────────────────────────
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='serial:///dev/serial0:921600',
        description='MAVROS FCU URL. Override e.g. for USB: serial:///dev/ttyACM0:115200',
    )
    takeoff_altitude_arg = DeclareLaunchArgument(
        'takeoff_altitude',
        default_value='1.5',
        description='Takeoff altitude in metres AGL.',
    )

    # ── 1. camera_node — Pi Camera Module via picamera2 ───────────────
    camera = Node(
        package='drone_face_tracking',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # ── 2. MAVROS — serial link to the H743 ───────────────────────────
    # We pass fcu_url both via parameters file (defaults) and via
    # launch arg (override). The launch-arg form wins because it's
    # passed directly, not loaded from YAML.
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': False},
            {'fcu_url': LaunchConfiguration('fcu_url')},
        ],
    )

    # ── 3. face_detect_node ───────────────────────────────────────────
    # Delay a couple of seconds so the camera publisher has time to
    # bind to libcamera and start emitting frames before the detector
    # subscribes. Not strictly required (DDS handles late joiners) but
    # avoids a noisy "no images yet" warning at startup.
    face_detect = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='drone_face_tracking',
                executable='face_detect_node',
                name='face_detect_node',
                output='screen',
                parameters=[{'use_sim_time': False}],
            )
        ],
    )

    # ── 4. flight_ctrl_node ───────────────────────────────────────────
    flight_ctrl = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='drone_face_tracking',
                executable='flight_ctrl_node',
                name='flight_ctrl_node',
                output='screen',
                parameters=[
                    {'use_sim_time':    False},
                    {'search_yaw_rate': 0.4},   # rad/s, must match mission
                ],
            )
        ],
    )

    # ── 5. mission_node ───────────────────────────────────────────────
    # Started a touch later so MAVROS has had time to come up; mission
    # has its own WAIT_FCU state that polls /mavros/state, so the exact
    # timing isn't critical, just polite.
    mission = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='drone_face_tracking',
                executable='mission_node',
                name='mission_node',
                output='screen',
                parameters=[
                    {'use_sim_time':         False},
                    # Wrap the LaunchConfiguration in ParameterValue with
                    # value_type=float so the string '1.5' is coerced to
                    # a double — mission_node declares this parameter as
                    # a float and would reject a raw string.
                    {'takeoff_altitude':     ParameterValue(
                        LaunchConfiguration('takeoff_altitude'),
                        value_type=float)},
                    {'altitude_tolerance':   0.2},
                    {'hover_seconds':        3.0},
                    {'tracking_seconds':    30.0},
                    {'search_seconds':      60.0},
                    {'search_yaw_rate':      0.4},
                    {'face_min_area':       0.001},
                    {'face_persist_ticks':     3},
                ],
            )
        ],
    )

    return LaunchDescription([
        fcu_url_arg,
        takeoff_altitude_arg,
        camera,
        mavros,
        face_detect,
        flight_ctrl,
        mission,
    ])
