import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg         = get_package_share_directory('drone_face_tracking')
    world_file  = os.path.join(pkg, 'worlds', 'face_tracking.world')
    model_dir   = os.path.join(pkg, 'models')
    config_file = os.path.join(pkg, 'config', 'mavros_params.yaml')
    model_sdf   = os.path.expanduser('~/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf')

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )

    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            model_dir, ':',
            os.path.expanduser('~/ardupilot_gazebo/models'), ':',
            os.path.expanduser('~/ardupilot_gazebo/worlds'), ':',
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )

    set_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            os.path.expanduser('~/ardupilot_gazebo/build'), ':',
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        ]
    )

    # ── 1. Gazebo Harmonic ─────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '--verbose', '-r', world_file],
        output='screen'
    )

    # ── 2. Spawn drone — increased delay + timeout to 10s ─────────────
    spawn_drone = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/face_tracking_world/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '10000',
                    '--req',
                    f'name: "iris_with_camera", '
                    f'sdf_filename: "{model_sdf}", '
                    f'pose: {{position: {{x: 0.0, y: 0.0, z: 0.2}}}}'
                ],
                output='screen'
            )
        ]
    )

    # ── 3. MAVROS — delayed to 12s to ensure Gazebo is stable ─────────
    mavros = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='mavros',
                executable='mavros_node',
                name='mavros',
                output='screen',
                parameters=[
                    config_file,
                    {'use_sim_time': False}
                ],
            )
        ]
    )

    # ── 3b. ros_gz_bridge — bridge Gazebo Harmonic camera into ROS 2 ──
    # The world loads gz-sim-sensors-system which produces
    #   /front_camera         (gz.msgs.Image)
    #   /front_camera/camera_info  (gz.msgs.CameraInfo)
    # parameter_bridge converts these into sensor_msgs/Image and
    # sensor_msgs/CameraInfo, and we remap the names to match what
    # face_detect_node subscribes to (/camera/image_raw).
    #
    # Bridge direction syntax:
    #   '[' = gz -> ros (we want this for camera frames)
    gz_bridge = TimerAction(
        period=12.5,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gz_bridge',
                output='screen',
                arguments=[
                    '/front_camera@sensor_msgs/msg/Image[gz.msgs.Image',
                    # Gazebo Harmonic quirk: camera_info is published at
                    # the bare /camera_info, NOT under /<topic>/camera_info.
                    '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                ],
                remappings=[
                    ('/front_camera', '/camera/image_raw'),
                    ('/camera_info',  '/camera/camera_info'),
                ],
                parameters=[{'use_sim_time': False}],
            )
        ]
    )

    # ── 4. face_detect_node ────────────────────────────────────────────
    face_detect = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='drone_face_tracking',
                executable='face_detect_node',
                name='face_detect_node',
                output='screen',
                parameters=[{'use_sim_time': False}]
            )
        ]
    )

    # ── 5. flight_ctrl_node ────────────────────────────────────────────
    flight_ctrl = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='drone_face_tracking',
                executable='flight_ctrl_node',
                name='flight_ctrl_node',
                output='screen',
                parameters=[
                    {'use_sim_time':     False},
                    {'search_yaw_rate':  0.4},   # rad/s, must match mission
                ]
            )
        ]
    )


    # ── 6. mission_node — orchestrates arm/takeoff/search/track/land ──
    mission = TimerAction(
        period=14.0,
        actions=[
            Node(
                package='drone_face_tracking',
                executable='mission_node',
                name='mission_node',
                output='screen',
                parameters=[
                    {'use_sim_time':         False},
                    {'takeoff_altitude':     1.5},
                    {'altitude_tolerance':   0.2},
                    {'hover_seconds':        3.0},
                    {'tracking_seconds':    30.0},
                    {'search_seconds':      60.0},  # max yaw-scan time
                    {'search_yaw_rate':      0.4},  # rad/s
                    {'face_min_area':       0.001},
                    {'face_persist_ticks':     3},  # consecutive ticks
                ]
            )
        ]
    )

    return LaunchDescription([
        use_sim_time,
        set_model_path,
        set_plugin_path,
        gazebo,
        spawn_drone,
        mavros,
        gz_bridge,
        face_detect,
        flight_ctrl,
        mission,
    ])
