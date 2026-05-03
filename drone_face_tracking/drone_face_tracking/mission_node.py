"""
mission_node.py
---------------
Top-level mission sequencer. Runs a state machine that walks the drone
through a full autonomous flight:

    INIT  ->  WAIT_FCU  ->  SET_GUIDED  ->  ARM  ->  TAKEOFF
          ->  HOVER     ->  TRACKING    ->  LAND ->  DONE

It uses MAVROS services to set mode, arm, takeoff, and land, and
publishes std_msgs/Bool on /mission/tracking_enabled to tell
flight_ctrl_node when it should be issuing face-tracking velocity cmds.

Topics published:
  /mission/tracking_enabled   (std_msgs/Bool)  -> gates flight_ctrl_node
  /mission/state              (std_msgs/String) -> current FSM state

Topics subscribed:
  /mavros/state               (mavros_msgs/State)     -> connection / armed / mode
  /mavros/altitude            (mavros_msgs/Altitude)  -> .relative field used as altitude

Services used:
  /mavros/set_mode            (mavros_msgs/SetMode)
  /mavros/cmd/arming          (mavros_msgs/CommandBool)
  /mavros/cmd/takeoff         (mavros_msgs/CommandTOL)
  /mavros/cmd/land            (mavros_msgs/CommandTOL)

Parameters (declared with defaults, overridable from launch):
  takeoff_altitude        float   1.5    target altitude in metres
  altitude_tolerance      float   0.2    +/- m considered "at altitude"
  hover_seconds           float   3.0    pause between takeoff and tracking
  tracking_seconds        float   30.0   how long to track before landing
                                          (set <= 0 to track indefinitely)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point

try:
    from mavros_msgs.msg import State, Altitude
    from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, CommandLong
    MAVROS_AVAILABLE = True
except ImportError:
    MAVROS_AVAILABLE = False


# ── FSM states ────────────────────────────────────────────────────────
INIT        = 'INIT'
WAIT_FCU    = 'WAIT_FCU'
SET_GUIDED  = 'SET_GUIDED'
ARM         = 'ARM'
TAKEOFF     = 'TAKEOFF'
HOVER       = 'HOVER'
SEARCH      = 'SEARCH'      # slowly yaw until a face is in view
TRACKING    = 'TRACKING'
LAND        = 'LAND'
DONE        = 'DONE'
ABORT       = 'ABORT'


class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('takeoff_altitude',     1.5)
        self.declare_parameter('altitude_tolerance',   0.2)
        self.declare_parameter('hover_seconds',        3.0)
        self.declare_parameter('tracking_seconds',    30.0)
        # SEARCH-state parameters
        self.declare_parameter('search_seconds',      60.0)   # max time to scan
        self.declare_parameter('search_yaw_rate',      0.4)   # rad/s, ≈23°/s
        self.declare_parameter('face_min_area',       0.001)  # min face_error.z
        self.declare_parameter('face_persist_ticks',     1)   # consec. ticks

        self.target_alt    = self.get_parameter('takeoff_altitude').value
        self.alt_tol       = self.get_parameter('altitude_tolerance').value
        self.hover_secs    = self.get_parameter('hover_seconds').value
        self.track_secs    = self.get_parameter('tracking_seconds').value
        self.search_secs   = self.get_parameter('search_seconds').value
        self.search_yaw    = self.get_parameter('search_yaw_rate').value
        self.face_min_area = self.get_parameter('face_min_area').value
        self.face_persist  = self.get_parameter('face_persist_ticks').value

        # ── State ─────────────────────────────────────────────────────
        self.state         = INIT
        self.fcu_state     = None       # latest mavros/state msg
        self.current_alt   = 0.0
        self.state_entered = self.get_clock().now()
        self.services_ready = False     # set True after wait completes
        # Face-detection telemetry (filled by face_error_callback)
        self.face_area          = 0.0   # latest face_error.z
        self.face_seen_streak   = 0     # consecutive ticks face seen
        self._last_arm_attempt  = None  # clock time of last arm() call

        # ── Callback groups so timer & service responses don't deadlock ─
        # The timer runs on its own group; services share a separate
        # mutually-exclusive group so call_async futures actually resolve
        # while the timer is also active.
        self.timer_cb_group   = MutuallyExclusiveCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()

        # ── QoS for /mavros/state — best effort, transient local ──────
        mavros_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── Subscribers ───────────────────────────────────────────────
        if MAVROS_AVAILABLE:
            self.create_subscription(
                State, '/mavros/state', self.state_callback, mavros_qos
            )
            # NOTE: /mavros/local_position/pose is NOT actually published
            # by this MAVROS install — only subscribed to. The reliable
            # source for relative altitude is /mavros/altitude (.relative
            # field = metres above takeoff/home).
            self.create_subscription(
                Altitude, '/mavros/altitude',
                self.altitude_callback, mavros_qos
            )
        self.create_subscription(
            Point, '/face_tracking/error',
            self.face_error_callback, 10
        )

        # ── Publishers ────────────────────────────────────────────────
        self.tracking_pub = self.create_publisher(
            Bool, '/mission/tracking_enabled', 10
        )
        self.search_pub = self.create_publisher(
            Bool, '/mission/search_enabled', 10
        )
        # Transient-local so late subscribers (e.g. plot_errors_node started
        # mid-flight) immediately receive the current state rather than missing
        # all transitions that happened before they connected.
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.state_pub = self.create_publisher(
            String, '/mission/state', latched_qos
        )

        # ── MAVROS service clients ────────────────────────────────────
        # NOTE: in this MAVROS install, the command-plugin services are
        # advertised under /mavros/mavros/* (the UAS sub-node), NOT under
        # /mavros/cmd/* (those paths show up in `ros2 service list` as
        # discovery ghosts but have no real server attached). set_mode is
        # the exception — it lives at /mavros/set_mode at the top level.
        # If you ever switch to a different MAVROS launch, verify with:
        #   ros2 node info /mavros/mavros
        if MAVROS_AVAILABLE:
            self.set_mode_cli = self.create_client(
                SetMode, '/mavros/set_mode',
                callback_group=self.service_cb_group)
            self.arm_cli = self.create_client(
                CommandBool, '/mavros/mavros/arming',
                callback_group=self.service_cb_group)
            self.takeoff_cli = self.create_client(
                CommandTOL, '/mavros/mavros/takeoff',
                callback_group=self.service_cb_group)
            self.land_cli = self.create_client(
                CommandTOL, '/mavros/mavros/land',
                callback_group=self.service_cb_group)
            # Force-arm client: MAV_CMD_COMPONENT_ARM_DISARM with param2=21196
            # bypasses all ArduPilot pre-arm checks (gyro consistency, etc.).
            # Used in SITL where virtual sensor settling can block normal arming.
            self.cmd_long_cli = self.create_client(
                CommandLong, '/mavros/mavros/command',
                callback_group=self.service_cb_group)

        # ── Main FSM tick ─────────────────────────────────────────────
        self.create_timer(0.5, self.tick, callback_group=self.timer_cb_group)

        # One-shot startup timer that waits for MAVROS services to come up
        # without blocking the constructor (so the executor can spin and
        # discovery can actually happen). Fires once 1 second after start.
        self._startup_timer = self.create_timer(
            1.0, self._wait_for_services_once,
            callback_group=self.timer_cb_group)

        self.get_logger().info(
            f'Mission node started\n'
            f'  takeoff alt    : {self.target_alt} m\n'
            f'  alt tolerance  : ±{self.alt_tol} m\n'
            f'  hover time     : {self.hover_secs} s\n'
            f'  tracking time  : {self.track_secs} s '
            f'({"indefinite" if self.track_secs <= 0 else "limited"})'
        )

    # ──────────────────────────────────────────────────────────────────
    # Startup
    # ──────────────────────────────────────────────────────────────────
    def _wait_for_services_once(self):
        """
        Run once after the executor starts. Waits up to 30 s for each
        MAVROS service to be advertised, logging which ones failed.
        Cancels itself after one execution.
        """
        # Make sure this only fires once
        self._startup_timer.cancel()

        if not MAVROS_AVAILABLE:
            return

        services = [
            ('/mavros/set_mode',          self.set_mode_cli),
            ('/mavros/mavros/arming',     self.arm_cli),
            ('/mavros/mavros/takeoff',    self.takeoff_cli),
            ('/mavros/mavros/land',       self.land_cli),
            ('/mavros/mavros/command',    self.cmd_long_cli),
        ]
        self.get_logger().info(
            'Probing MAVROS services (10 s each)...'
        )
        for path, cli in services:
            if cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().info(f'  ✓ {path}')
            else:
                self.get_logger().warn(f'  ⚠ {path} not visible after 10 s')
        self.services_ready = True
        self.get_logger().info('Mission FSM enabled.')

    # ──────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────
    def state_callback(self, msg):
        self.fcu_state = msg

    def altitude_callback(self, msg: Altitude):
        # msg.relative = metres above takeoff/home position. This is the
        # value the takeoff target is in (CommandTOL.altitude is also
        # interpreted as relative to home in ArduPilot).
        self.current_alt = msg.relative

    def face_error_callback(self, msg: Point):
        # face_detect_node uses error.z as the normalised face area:
        # 0.0 → no face, > 0 → face detected.
        self.face_area = msg.z

    def _face_currently_seen(self) -> bool:
        return self.face_area >= self.face_min_area

    # ──────────────────────────────────────────────────────────────────
    # State helpers
    # ──────────────────────────────────────────────────────────────────
    def transition(self, new_state):
        self.get_logger().info(f'STATE: {self.state} → {new_state}')
        self.state = new_state
        self.state_entered = self.get_clock().now()
        # Reset face-detection streak on every state change so the
        # SEARCH state requires a fresh confirmation window.
        self.face_seen_streak = 0
        # Reset arm-attempt timer so re-entry into ARM starts fresh.
        self._last_arm_attempt = None
        # Publish state for any monitoring tool (rqt, etc.)
        self.state_pub.publish(String(data=new_state))
        # Tracking gate is ON only in TRACKING
        self.tracking_pub.publish(Bool(data=(new_state == TRACKING)))
        # Search gate is ON only in SEARCH
        self.search_pub.publish(Bool(data=(new_state == SEARCH)))

    def time_in_state(self):
        delta = self.get_clock().now() - self.state_entered
        return delta.nanoseconds / 1e9

    # ──────────────────────────────────────────────────────────────────
    # Service helpers — async, non-blocking
    # ──────────────────────────────────────────────────────────────────
    def _log_service_result(self, label: str):
        """
        Returns a done-callback that logs the result of an async MAVROS
        service call. Without this, rejection reasons (pre-arm failures,
        mode-change refusals, etc.) are silently dropped.
        """
        def _cb(future):
            try:
                resp = future.result()
            except Exception as exc:
                self.get_logger().error(f'{label} call raised: {exc}')
                return

            if resp is None:
                self.get_logger().warn(f'{label}: no response')
                return

            # MAVROS service responses expose either a `success` bool
            # (CommandBool / CommandTOL) or `mode_sent` (SetMode), plus
            # a numeric MAV_RESULT in `result` for the Command* services.
            success = getattr(resp, 'success', None)
            if success is None:
                success = getattr(resp, 'mode_sent', None)
            mav_result = getattr(resp, 'result', None)

            if success:
                self.get_logger().info(
                    f'{label} OK'
                    + (f' (mav_result={mav_result})' if mav_result is not None else '')
                )
            else:
                self.get_logger().warn(
                    f'{label} REJECTED'
                    + (f' (mav_result={mav_result})' if mav_result is not None else '')
                )
        return _cb

    def call_set_mode(self, mode_str: str):
        # Skip service_is_ready() — it's type-hash-pedantic and can return
        # False even when the service is fully callable. The done-callback
        # will surface real failures.
        req = SetMode.Request()
        req.custom_mode = mode_str
        future = self.set_mode_cli.call_async(req)
        future.add_done_callback(self._log_service_result(f'set_mode({mode_str})'))
        return future

    def call_arm(self, value: bool):
        req = CommandBool.Request()
        req.value = value
        future = self.arm_cli.call_async(req)
        future.add_done_callback(self._log_service_result(f'arm({value})'))
        return future

    def call_force_arm(self):
        """
        Force-arm via MAV_CMD_COMPONENT_ARM_DISARM (400) with param2=21196.
        ArduPilot's magic number to skip all pre-arm checks — safe for SITL
        where 'Gyros inconsistent' blocks normal arming.
        """
        req = CommandLong.Request()
        req.broadcast    = False
        req.command      = 400   # MAV_CMD_COMPONENT_ARM_DISARM
        req.confirmation = 0
        req.param1       = 1.0   # 1 = arm
        req.param2       = 21196.0  # ArduPilot force-arm bypass
        req.param3       = 0.0
        req.param4       = 0.0
        req.param5       = 0.0
        req.param6       = 0.0
        req.param7       = 0.0
        future = self.cmd_long_cli.call_async(req)
        future.add_done_callback(self._log_service_result('force_arm'))
        return future

    def call_takeoff(self, altitude: float):
        req = CommandTOL.Request()
        req.altitude = altitude
        future = self.takeoff_cli.call_async(req)
        future.add_done_callback(self._log_service_result(f'takeoff({altitude}m)'))
        return future

    def call_land(self):
        req = CommandTOL.Request()
        future = self.land_cli.call_async(req)
        future.add_done_callback(self._log_service_result('land'))
        return future

    # ──────────────────────────────────────────────────────────────────
    # Main FSM tick
    # ──────────────────────────────────────────────────────────────────
    def tick(self):
        if not MAVROS_AVAILABLE:
            self.get_logger().error('mavros_msgs not installed — aborting.')
            self.transition(ABORT)
            return

        # Hold the FSM in INIT until startup service-discovery has run.
        if not self.services_ready:
            return

        # ── INIT: just kick off ──────────────────────────────────────
        if self.state == INIT:
            self.transition(WAIT_FCU)

        # ── WAIT_FCU: wait until MAVROS reports connected ────────────
        elif self.state == WAIT_FCU:
            if self.fcu_state is not None and self.fcu_state.connected:
                self.get_logger().info('FCU connected.')
                self.transition(SET_GUIDED)
            elif self.time_in_state() > 30.0:
                self.get_logger().error('Timeout waiting for FCU.')
                self.transition(ABORT)

        # ── SET_GUIDED: switch to GUIDED mode ────────────────────────
        elif self.state == SET_GUIDED:
            if self.fcu_state.mode == 'GUIDED':
                self.transition(ARM)
            elif self.time_in_state() > 1.0:   # retry every 1 s
                self.get_logger().info('Requesting GUIDED mode...')
                self.call_set_mode('GUIDED')
                self.state_entered = self.get_clock().now()

        # ── ARM: arm the vehicle ─────────────────────────────────────
        elif self.state == ARM:
            if self.fcu_state.armed:
                self.get_logger().info('Vehicle armed.')
                self.transition(TAKEOFF)
            else:
                # MAV_STATE values: 0=UNINIT 1=BOOT 2=CALIBRATING 3=STANDBY 4=ACTIVE
                # Only attempt arming once ArduPilot reaches STANDBY — that means
                # gyro calibration has genuinely completed and pre-arm checks will pass.
                # Never force-arm: bypassing the check lets an uncalibrated IMU through,
                # producing garbage flight dynamics.
                status = self.fcu_state.system_status
                MAV_STATE_STANDBY = 3
                if status != MAV_STATE_STANDBY:
                    self.get_logger().info(
                        f'Waiting for STANDBY (system_status={status}, '
                        f'need {MAV_STATE_STANDBY})...',
                        throttle_duration_sec=2.0,
                    )
                else:
                    now = self.get_clock().now()
                    since_last = (
                        (now - self._last_arm_attempt).nanoseconds / 1e9
                        if self._last_arm_attempt is not None else 999.0
                    )
                    if since_last >= 3.0:
                        self.get_logger().info('Requesting ARM...')
                        self.call_arm(True)
                        self._last_arm_attempt = now

        # ── TAKEOFF: command takeoff and wait for altitude ───────────
        elif self.state == TAKEOFF:
            # Send the takeoff once, on entry; then just monitor altitude
            if self.time_in_state() < 0.5:
                self.get_logger().info(
                    f'Commanding takeoff to {self.target_alt} m'
                )
                self.call_takeoff(self.target_alt)

            # Log altitude every ~2 s so we can see whether the drone is
            # actually climbing (vs stuck at 0 due to an interfering
            # velocity setpoint, no EKF home, etc.).
            self.get_logger().info(
                f'Climbing... alt={self.current_alt:.2f} m '
                f'(target {self.target_alt:.2f} m)',
                throttle_duration_sec=2.0,
            )

            if self.time_in_state() > 10.0:
                self.get_logger().info(
                    f'Reached target altitude ({self.current_alt:.2f} m).'
                )
                self.transition(HOVER)
            elif self.time_in_state() > 60.0:
                self.get_logger().error('Takeoff timeout.')
                self.transition(LAND)

        # ── HOVER: stabilise for a few seconds before tracking ───────
        elif self.state == HOVER:
            if self.time_in_state() >= self.hover_secs:
                # If the face is already in view at the end of HOVER, go
                # straight to TRACKING — skipping SEARCH avoids sending
                # CONDITION_YAW which would spin the drone away from a face
                # that's already visible. (Early-flight lateral drift from
                # the GUID_TIMEOUT coasting bug is fixed separately by
                # publishing zero velocity on face loss in flight_ctrl_node.)
                if self._face_currently_seen():
                    self.get_logger().info(
                        'Face in view at hover end — skipping search.'
                    )
                    self.transition(TRACKING)
                else:
                    self.get_logger().info(
                        f'No face in view — beginning yaw search '
                        f'(rate={self.search_yaw} rad/s, '
                        f'timeout={self.search_secs} s).'
                    )
                    self.transition(SEARCH)

        # ── SEARCH: slowly yaw, looking for a face ───────────────────
        elif self.state == SEARCH:
            # Accumulate a confidence streak toward TRACKING.
            # On a positive tick the streak increments; on a missed tick it
            # decrements by 1 rather than resetting to zero. This tolerates
            # intermittent Haar detections — 3 of the last 4 ticks seeing the
            # face is enough, rather than requiring 3 perfectly consecutive
            # ones. At 22.9 deg/s the face sweeps the 80° FOV in ~3.5 s, and
            # individual frames are often missed as it moves across the edge.
            if self._face_currently_seen():
                self.face_seen_streak += 1
                if self.face_seen_streak >= self.face_persist:
                    self.get_logger().info(
                        f'Face acquired (area={self.face_area:.3f}) — '
                        f'stopping search, beginning tracking.'
                    )
                    self.transition(TRACKING)
                    return
            else:
                self.face_seen_streak = max(0, self.face_seen_streak - 1)

            # if self.time_in_state() >= self.search_secs:
            #     self.get_logger().warn(
            #         f'Search timed out after {self.search_secs} s — landing.'
            #     )
            #     self.transition(LAND)

        # ── TRACKING: face tracking handled by flight_ctrl_node ──────
        elif self.state == TRACKING:
            # If we lose the face for the entire persistence window while
            # tracking, fall back to SEARCH so we can re-acquire instead
            # of just hovering blindly until tracking_seconds elapses.
            if not self._face_currently_seen():
                self.face_seen_streak += 1   # reuse counter as "lost streak"
                if self.face_seen_streak >= self.face_persist * 20:
                    self.get_logger().warn(
                        'Face lost during tracking — returning to SEARCH.'
                    )
                    self.transition(SEARCH)
                    return
            else:
                # Decay the lost-streak on a positive detection rather than
                # resetting immediately — mirrors the acquire logic above so
                # brief re-detections don't reset the loss counter entirely.
                self.face_seen_streak = max(0, self.face_seen_streak - 1)


        # ── LAND: command land and wait for disarm ───────────────────
        elif self.state == LAND:
            if self.time_in_state() < 0.5:
                self.get_logger().info('Commanding LAND.')
                self.call_land()
            if not self.fcu_state.armed:
                self.get_logger().info('Vehicle disarmed — mission complete.')
                self.transition(DONE)
            elif self.time_in_state() > 60.0:
                self.get_logger().warn('Land timeout — giving up.')
                self.transition(DONE)

        # ── DONE / ABORT: just sit ───────────────────────────────────
        elif self.state in (DONE, ABORT):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    # MultiThreadedExecutor is required so that service-call futures
    # actually resolve while the FSM timer is also running.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
