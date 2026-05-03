"""
plot_errors_node.py
-------------------
Subscribes to the face-tracking error and mission state topics during a
simulation run, records everything in memory, and saves a multi-panel
matplotlib figure on shutdown (Ctrl-C or SIGTERM).

Run alongside the simulation in a separate terminal:
    ros2 run drone_face_tracking plot_errors_node

A timestamped PNG is written to ~/drone_tracking_plots/ on exit.

Panels
------
  1. Horizontal error  (error.x, −1 = far left … +1 = far right)
  2. Vertical error    (error.y, −1 = far top  … +1 = far bottom)
  3. Face area         (error.z, normalised bounding-box area used as
                        the distance proxy, with TARGET_FACE_AREA line)

Mission state transitions are drawn as coloured background bands so you
can see exactly which phase produced each section of the error curves.
"""

import os
import sys
import datetime
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import String

import matplotlib
matplotlib.use('Agg')          # headless-safe; works without a display
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


# Must match flight_ctrl_node.py so the dashed line lands in the right place
TARGET_FACE_AREA = 0.035

# Background colour for each FSM state band
STATE_COLOURS = {
    'INIT':       '#cccccc',
    'WAIT_FCU':   '#cccccc',
    'SET_GUIDED': '#cccccc',
    'ARM':        '#aad4f5',
    'TAKEOFF':    '#a8e6cf',
    'HOVER':      '#dcedc1',
    'SEARCH':     '#ffd3b6',
    'TRACKING':   '#ff8b94',
    'LAND':       '#d4a5d4',
    'DONE':       '#cccccc',
    'ABORT':      '#ff4444',
}


class PlotErrorsNode(Node):
    def __init__(self):
        super().__init__('plot_errors_node')

        self._t0 = self.get_clock().now()

        # Raw data buffers
        self._times  = []
        self._err_x  = []
        self._err_y  = []
        self._face_z = []

        # State transition log: list of (elapsed_s, state_name)
        self._state_log = []

        self.create_subscription(
            Point, '/face_tracking/error', self._error_cb, 10
        )
        # Must match the transient-local QoS used by mission_node's state_pub
        # so this subscriber receives the last-published state immediately on
        # connection even when started mid-flight.
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, '/mission/state', self._state_cb, latched_qos
        )

        self.get_logger().info(
            'plot_errors_node recording — press Ctrl-C to save plot.'
        )

    # ------------------------------------------------------------------
    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._t0).nanoseconds / 1e9

    def _error_cb(self, msg: Point):
        t = self._elapsed()
        self._times.append(t)
        self._err_x.append(msg.x)
        self._err_y.append(msg.y)
        self._face_z.append(msg.z)

    def _state_cb(self, msg: String):
        self._state_log.append((self._elapsed(), msg.data))

    # ------------------------------------------------------------------
    def save_plot(self):
        if not self._times:
            self.get_logger().warn('No data recorded — nothing to plot.')
            return

        out_dir = os.path.expanduser('~/drone_tracking_plots')
        os.makedirs(out_dir, exist_ok=True)
        stamp    = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        out_path = os.path.join(out_dir, f'tracking_errors_{stamp}.png')

        fig, axes = plt.subplots(3, 1, figsize=(15, 9), sharex=True)
        # Title is set after masking so we can note if state data was missing
        _title_base = 'Face Tracking Errors Over Time'

        t_end = self._times[-1]

        # ── Build state intervals ──────────────────────────────────────
        intervals = []
        if self._state_log:
            # Fill any gap before the first logged state
            if self._state_log[0][0] > 0:
                intervals.append((0.0, self._state_log[0][0], 'INIT'))
            for i, (t, state) in enumerate(self._state_log):
                end = self._state_log[i + 1][0] if i + 1 < len(self._state_log) else t_end
                intervals.append((t, end, state))

        # ── Build a per-sample TRACKING mask (NaN outside TRACKING) ──
        # For each recorded timestamp, check whether it falls inside a
        # TRACKING interval. Samples outside TRACKING become NaN so
        # matplotlib draws a gap rather than a misleading line.
        tracking_intervals = [(t0, t1) for t0, t1, s in intervals if s == 'TRACKING']

        if not tracking_intervals:
            # No TRACKING state was observed — either the node was started
            # before any state transitions fired, or state messages were missed.
            # Fall back to showing all recorded data unmasked so the plot is
            # still useful; annotate the title to make the situation clear.
            self.get_logger().warn(
                'No TRACKING intervals found in state log — '
                'plotting all data unmasked. '
                'Start plot_errors_node before the simulation for best results.'
            )
            err_x_masked  = self._err_x
            err_y_masked  = self._err_y
            face_z_masked = self._face_z
            no_state_data = True
        else:
            def tracking_mask(values):
                masked = []
                for t, v in zip(self._times, values):
                    in_tracking = any(t0 <= t <= t1 for t0, t1 in tracking_intervals)
                    masked.append(v if in_tracking else float('nan'))
                return masked

            err_x_masked  = tracking_mask(self._err_x)
            err_y_masked  = tracking_mask(self._err_y)
            face_z_masked = tracking_mask(self._face_z)
            no_state_data = False

        # ── Draw state bands on every panel ───────────────────────────
        for ax in axes:
            for t_start, t_stop, state in intervals:
                colour = STATE_COLOURS.get(state, '#eeeeee')
                ax.axvspan(t_start, t_stop, alpha=0.22, color=colour, linewidth=0)

        # ── Panel 1: Horizontal error ──────────────────────────────────
        axes[0].plot(self._times, err_x_masked,
                     color='#e74c3c', linewidth=1.0, label='err_x (horizontal)')
        axes[0].axhline(0, color='black', linewidth=0.7, linestyle='--', alpha=0.6)
        axes[0].set_ylabel('Horizontal error\n(← −1  ·  0  ·  +1 →)', fontsize=9)
        axes[0].set_ylim(-1.15, 1.15)
        axes[0].legend(loc='upper right', fontsize=8)
        axes[0].grid(True, alpha=0.3)

        # ── Panel 2: Vertical error ────────────────────────────────────
        axes[1].plot(self._times, err_y_masked,
                     color='#3498db', linewidth=1.0, label='err_y (vertical)')
        axes[1].axhline(0, color='black', linewidth=0.7, linestyle='--', alpha=0.6)
        axes[1].set_ylabel('Vertical error\n(↑ −1  ·  0  ·  +1 ↓)', fontsize=9)
        axes[1].set_ylim(-1.15, 1.15)
        axes[1].legend(loc='upper right', fontsize=8)
        axes[1].grid(True, alpha=0.3)

        # ── Panel 3: Face area (distance proxy) ───────────────────────
        axes[2].plot(self._times, face_z_masked,
                     color='#2ecc71', linewidth=1.0, label='face area (z)')
        axes[2].axhline(
            TARGET_FACE_AREA, color='#e67e22', linewidth=1.3,
            linestyle='--', label=f'target area ({TARGET_FACE_AREA:.3f})'
        )
        axes[2].set_ylabel('Normalised face area\n(distance proxy)', fontsize=9)
        axes[2].set_xlabel('Time (s)', fontsize=10)
        axes[2].legend(loc='upper right', fontsize=8)
        axes[2].grid(True, alpha=0.3)

        # ── State legend at the bottom ─────────────────────────────────
        seen = {state for (_, _, state) in intervals}
        ordered = ['ARM', 'TAKEOFF', 'HOVER', 'SEARCH', 'TRACKING', 'LAND', 'ABORT']
        patches = [
            mpatches.Patch(color=STATE_COLOURS[s], alpha=0.6, label=s)
            for s in ordered if s in seen
        ]
        if patches:
            fig.legend(
                handles=patches,
                loc='lower center',
                ncol=len(patches),
                fontsize=8,
                title='Mission state',
                title_fontsize=8,
                bbox_to_anchor=(0.5, 0.0),
                framealpha=0.8,
            )

        title = _title_base + (' [no state data — all samples shown]' if no_state_data else '')
        fig.suptitle(title, fontsize=14, fontweight='bold')

        plt.tight_layout(rect=[0, 0.06, 1, 0.97])
        fig.savefig(out_path, dpi=150, bbox_inches='tight')
        self.get_logger().info(f'Plot saved → {out_path}')
        plt.close(fig)


# ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PlotErrorsNode()

    def _on_shutdown(sig, frame):
        node.save_plot()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _on_shutdown)
    signal.signal(signal.SIGTERM, _on_shutdown)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.save_plot()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
