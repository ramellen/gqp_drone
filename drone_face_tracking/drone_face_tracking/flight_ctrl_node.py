"""
flight_ctrl_node.py
--------------------
Velocity-setpoint publisher, gated by mission_node.

Three modes:
  * tracking_enabled (TRACKING state): proportional face-tracking
    controller — steers toward the detected face.
  * search_enabled (SEARCH state): publishes a constant slow yaw rate
    so the drone scans the area looking for a face.
  * neither: zero setpoints, autopilot's GUIDED mode holds position.

Topics subscribed:
  /face_tracking/error       (geometry_msgs/Point)
  /mission/tracking_enabled  (std_msgs/Bool)
  /mission/search_enabled    (std_msgs/Bool)

Topics published:
  /mavros/mavros/cmd_vel_unstamped  (geometry_msgs/Twist)
      Used during TRACKING — non-zero velocity components.
      NOTE: /mavros/setpoint_velocity/cmd_vel_unstamped exists in the topic
      list but has 0 subscribers (ghost topic). Real subscriber is under
      the /mavros/mavros/* UAS sub-node, consistent with all services.

Services used:
  /mavros/mavros/command  (mavros_msgs/CommandLong)
      MAV_CMD_CONDITION_YAW (115) is sent on SEARCH entry and
      periodically re-sent so the drone yaws continuously while
      SEARCH is active. We use this instead of a velocity-based
      yaw_rate because ArduPilot's GUIDED handler silently drops
      yaw_rate from SET_POSITION_TARGET_LOCAL_NED when linear
      velocity is zero — verified by inspecting the published
      PositionTarget (correct type_mask and yaw_rate, no rotation).
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool

try:
    from mavros_msgs.srv import SetMode, CommandBool, CommandLong
    from mavros_msgs.msg import PositionTarget
    MAVROS_AVAILABLE = True
except ImportError:
    MAVROS_AVAILABLE = False
    PositionTarget = None  # type: ignore
    CommandLong = None     # type: ignore

# MAVLink command IDs we use directly
MAV_CMD_CONDITION_YAW = 115


# PD gain constants — tune in sim first.
# D terms damp overshoot by opposing rapid changes in error.
# Start KD values at 0 and increase until oscillation settles without sluggishness.
KP_YAW           = 0.8
KP_ALT           = 0.25
KP_FWD           = 1.0

KD_YAW           = 0.02   # damps lateral oscillation
KD_ALT           = 0.02   # damps vertical oscillation
KD_FWD           = 0.05   # damps forward/back hunting

TARGET_FACE_AREA = 0.035   # face occupancy

MAX_VEL_LATERAL  = 1.5   # m/s — max lateral (y) velocity during TRACKING
MAX_VEL_VERTICAL = 1.5
MAX_VEL_FORWARD  = 3.0

SETPOINT_HZ      = 20

DEFAULT_SEARCH_YAW_RATE = 0.4


class FlightCtrlNode(Node):
    def __init__(self):
        super().__init__('flight_ctrl_node')

        # Parameter for search yaw rate (overridable from launch)
        self.declare_parameter('search_yaw_rate', DEFAULT_SEARCH_YAW_RATE)
        self.search_yaw_rate = float(
            self.get_parameter('search_yaw_rate').value
        )

        self.face_error       = Point()
        self.face_seen        = False
        self.tracking_enabled = False     # gated by mission_node
        self.search_enabled   = False     # gated by mission_node

        # Derivative state — previous errors and timestamp for each axis.
        # Reset to None whenever tracking is disabled so the first tick after
        # re-entry doesn't produce a spurious large derivative spike.
        self._prev_err_x    = None
        self._prev_err_y    = None
        self._prev_area_err = None
        self._prev_time     = None

        # Subscribers
        self.create_subscription(
            Point, '/face_tracking/error', self.face_error_cb, 10
        )
        self.create_subscription(
            Bool, '/mission/tracking_enabled', self.tracking_cb, 10
        )
        self.create_subscription(
            Bool, '/mission/search_enabled', self.search_cb, 10
        )

        # Publisher for TRACKING — non-zero linear velocity components
        # are honored by ArduPilot's GUIDED mode without issue.
        # NOTE: in this MAVROS install all topics (like services) live under
        # /mavros/mavros/* (the UAS sub-node). /mavros/setpoint_velocity/cmd_vel_unstamped
        # exists in the topic list but has 0 subscribers — it is a ghost.
        # The real subscriber is /mavros/mavros/cmd_vel_unstamped.
        self.vel_pub = self.create_publisher(
            Twist,
            '/mavros/mavros/cmd_vel_unstamped',
            10
        )

        # Service client for SEARCH yaw rotation.
        #
        # We don't use velocity setpoints to drive the SEARCH yaw because
        # ArduPilot's GUIDED handler silently drops yaw_rate from
        # SET_POSITION_TARGET_LOCAL_NED when the linear velocity is zero
        # (verified: PositionTarget arrives at MAVROS with the right
        # type_mask and yaw_rate, but the drone doesn't rotate). The
        # canonical ArduPilot mechanism for yaw control independent of
        # translation is MAV_CMD_CONDITION_YAW.
        # NOTE: in this MAVROS install, command-plugin services live at
        # /mavros/mavros/* (the UAS sub-node), not /mavros/cmd/* — see
        # the matching comment in mission_node.py. /mavros/cmd/command
        # appears in `ros2 service list` but is a discovery ghost with
        # no server attached.
        if MAVROS_AVAILABLE:
            self.yaw_cmd_cli = self.create_client(
                CommandLong, '/mavros/mavros/command'
            )
        else:
            self.yaw_cmd_cli = None

        # Track when we last sent a CONDITION_YAW so we can re-send
        # before the previous 360° rotation completes, keeping the
        # drone rotating continuously until SEARCH ends.
        self._last_yaw_cmd_time = None
        self._yaw_cmd_period_s  = 0.0
        self._yaw_cmd_warned    = False

        self.create_timer(1.0 / SETPOINT_HZ, self.publish_setpoint)

        self.get_logger().info(
            f'Flight control node ready '
            f'(search_yaw_rate={self.search_yaw_rate} rad/s).'
        )

    # ------------------------------------------------------------------
    def face_error_cb(self, msg: Point):
        self.face_error = msg
        self.face_seen  = msg.z > 0.0

    def tracking_cb(self, msg: Bool):
        if msg.data != self.tracking_enabled:
            self.get_logger().info(
                f'Tracking {"ENABLED" if msg.data else "DISABLED"}'
            )
            if not msg.data:
                # Clear derivative history so the first tick after re-entry into
                # TRACKING doesn't generate a large spurious derivative spike.
                self._prev_err_x    = None
                self._prev_err_y    = None
                self._prev_area_err = None
                self._prev_time     = None
        self.tracking_enabled = msg.data

    def search_cb(self, msg: Bool):
        was_enabled = self.search_enabled
        self.search_enabled = msg.data
        if msg.data and not was_enabled:
            self.get_logger().info('Search ENABLED')
            # Kick off the first yaw command immediately on entry.
            self._send_condition_yaw()
        elif not msg.data and was_enabled:
            self.get_logger().info('Search DISABLED.')
            # Do NOT send a stop-yaw command here. Sending CONDITION_YAW with
            # 0° @ 0 deg/s puts ArduPilot into a position-hold sub-mode that
            # refuses all subsequent velocity commands — confirmed by the drone
            # ignoring manual velocity pubs immediately after stop_yaw fires.
            #
            # Instead, we rely on the fact that velocity setpoints sent during
            # TRACKING naturally override an in-progress CONDITION_YAW sweep
            # in ArduPilot's GUIDED mode (confirmed: velocity pubs work fine
            # while CONDITION_YAW is actively spinning during SEARCH). The
            # residual yaw will stop on its own once the sweep completes or
            # the velocity controller takes authority.
            self._last_yaw_cmd_time = None

    # ------------------------------------------------------------------
    def _send_condition_yaw(self):
        """
        Fire-and-forget MAV_CMD_CONDITION_YAW for a relative 360°
        rotation at search_yaw_rate. ArduPilot honors this in GUIDED
        mode and the drone holds its current position while rotating.
        """
        if not MAVROS_AVAILABLE or self.yaw_cmd_cli is None:
            return
        # NOTE: deliberately not calling service_is_ready() — see the
        # comment in mission_node: in this MAVROS install it is
        # type-hash-pedantic and can return False even when the service
        # is fully callable. The done-callback below surfaces real
        # failures, including service-not-found.
        rate_deg_s = max(1.0, math.degrees(abs(self.search_yaw_rate)))
        direction  = 1.0 if self.search_yaw_rate >= 0 else -1.0
        req = CommandLong.Request()
        req.broadcast    = False
        req.command      = MAV_CMD_CONDITION_YAW
        req.confirmation = 0
        req.param1 = 360.0      # target angle (deg) — full sweep
        req.param2 = rate_deg_s # angular speed (deg/s)
        req.param3 = direction  # +1 = CW, -1 = CCW
        req.param4 = 1.0        # 1 = relative to current heading
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        future = self.yaw_cmd_cli.call_async(req)
        future.add_done_callback(self._yaw_cmd_done_cb)

        self._last_yaw_cmd_time = self.get_clock().now()
        # Re-send slightly before the rotation completes so there is no
        # stop-and-restart pause. 95% of the full-sweep duration.
        self._yaw_cmd_period_s = (360.0 / rate_deg_s) * 0.95
        self.get_logger().info(
            f'CONDITION_YAW sent (rate={rate_deg_s:.1f} deg/s, '
            f'next refresh in {self._yaw_cmd_period_s:.1f} s).'
        )

    def _yaw_cmd_done_cb(self, future):
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f'CONDITION_YAW raised: {exc}')
            return
        success = getattr(resp, 'success', None)
        result  = getattr(resp, 'result', None)
        if success:
            self.get_logger().info(
                f'CONDITION_YAW accepted (mav_result={result}).'
            )
        else:
            self.get_logger().warn(
                f'CONDITION_YAW rejected (mav_result={result}).'
            )

    def _send_stop_yaw(self):
        """
        Cancel any in-progress CONDITION_YAW rotation.
        Sending a relative 0° command at 0 deg/s tells ArduPilot to hold
        the current heading immediately rather than finishing the sweep.
        """
        if not MAVROS_AVAILABLE or self.yaw_cmd_cli is None:
            return
        req = CommandLong.Request()
        req.broadcast    = False
        req.command      = MAV_CMD_CONDITION_YAW
        req.confirmation = 0
        req.param1 = 0.0   # 0° relative → hold current heading
        req.param2 = 0.0   # 0 deg/s → stop immediately
        req.param3 = 0.0
        req.param4 = 1.0   # 1 = relative to current heading
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        future = self.yaw_cmd_cli.call_async(req)
        future.add_done_callback(self._yaw_cmd_done_cb)
        self.get_logger().info('Stop-yaw sent (CONDITION_YAW 0° @ 0 deg/s).')

    # ------------------------------------------------------------------
    def publish_setpoint(self):
        # IMPORTANT: only publish a setpoint when we actively want the
        # vehicle to move. ArduPilot's GUIDED mode treats any incoming
        # velocity setpoint as the new target, including zero. Publishing
        # zero at 20 Hz during takeoff would pin the drone to the ground.
        # By staying silent in HOVER / TAKEOFF / LAND etc., we let
        # ArduPilot use its native behaviour (climb to takeoff target,
        # then hold position via GUID_TIMEOUT after ~2 s of silence).

        # ── Diagnostics (throttled so they don't flood the terminal) ──
        self.get_logger().info(
            f'[flight_ctrl] tracking={self.tracking_enabled} '
            f'search={self.search_enabled} '
            f'face_seen={self.face_seen} '
            f'face_z={self.face_error.z:.4f}',
            throttle_duration_sec=2.0
        )

        if self.tracking_enabled and self.face_seen:
            twist = Twist()
            err_x      = self.face_error.x
            err_y      = self.face_error.y
            face_area  = self.face_error.z
            area_error = face_area - TARGET_FACE_AREA

            now = self.get_clock().now()

            # ── Derivative terms ──────────────────────────────────────
            # On the first tick after entering TRACKING, prev values are
            # None — use zero derivative to avoid a spike from a
            # discontinuous jump in error.
            if self._prev_time is not None:
                dt = (now - self._prev_time).nanoseconds / 1e9
                dt = max(dt, 1e-4)   # guard against zero/negative dt
                d_err_x    = (err_x    - self._prev_err_x)    / dt
                d_err_y    = (err_y    - self._prev_err_y)    / dt
                d_area_err = (area_error - self._prev_area_err) / dt
            else:
                d_err_x = d_err_y = d_area_err = 0.0

            # Store for next tick
            self._prev_err_x    = err_x
            self._prev_err_y    = err_y
            self._prev_area_err = area_error
            self._prev_time     = now

            # ── PD outputs ────────────────────────────────────────────
            # Lateral: err_x > 0 → face right of centre → slide right → -Y
            twist.linear.y = float(self._clamp(
                -(KP_YAW * err_x + KD_YAW * d_err_x),
                -MAX_VEL_LATERAL, MAX_VEL_LATERAL
            ))
            # Vertical: err_y > 0 → face below centre → climb → -Z
            twist.linear.z = float(self._clamp(
                -(KP_ALT * err_y + KD_ALT * d_err_y),
                -MAX_VEL_VERTICAL, MAX_VEL_VERTICAL
            ))
            # Forward: area_error < 0 → too far → advance → +X
            twist.linear.x = float(self._clamp(
                -(KP_FWD * area_error + KD_FWD * d_area_err),
                -MAX_VEL_FORWARD, MAX_VEL_FORWARD
            ))

            self.get_logger().info(
                f'[flight_ctrl] vel: '
                f'x={twist.linear.x:.3f} y={twist.linear.y:.3f} z={twist.linear.z:.3f} '
                f'| err x={err_x:.3f}({d_err_x:+.3f}/s) '
                f'y={err_y:.3f}({d_err_y:+.3f}/s) '
                f'area={area_error:.4f}({d_area_err:+.4f}/s)',
                throttle_duration_sec=1.0
            )
            self.vel_pub.publish(twist)

        elif self.tracking_enabled and not self.face_seen:
            # Face dropped out during TRACKING. Publish a zero-velocity Twist
            # immediately so ArduPilot doesn't coast on the last non-zero
            # command for the full GUID_TIMEOUT (~3 s). Without this, the
            # drone keeps moving laterally/forward for up to 3 s every time
            # the face detector misses a frame, which looks like instability
            # in early flight phases if it happens soon after TRACKING starts.
            self.vel_pub.publish(Twist())
            self.get_logger().warn(
                '[flight_ctrl] TRACKING enabled but face_seen=False '
                f'(face_z={self.face_error.z:.4f}) — zero-vel published',
                throttle_duration_sec=2.0
            )

        elif self.search_enabled:
            # SEARCH yaw is driven by MAV_CMD_CONDITION_YAW, sent in
            # search_cb() on entry. While SEARCH is active, periodically
            # re-send so the rotation continues past the first 360°
            # sweep. We deliberately publish NO velocity setpoint here —
            # after GUID_TIMEOUT (~2 s of silence) ArduPilot holds the
            # current position, while CONDITION_YAW continues spinning
            # the heading independently.
            now = self.get_clock().now()
            if (
                self._last_yaw_cmd_time is None
                or (now - self._last_yaw_cmd_time).nanoseconds / 1e9
                   >= self._yaw_cmd_period_s
            ):
                self._send_condition_yaw()

        # else: publish nothing. After GUID_TIMEOUT (~2 s), ArduPilot
        # automatically reverts to position-hold at the last position.

    # ------------------------------------------------------------------
    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))


def main(args=None):
    rclpy.init(args=args)
    node = FlightCtrlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
