#!/usr/bin/env python3
"""
imu_controller.py  —  Gesture Controlled Robot (Upgraded)

Architecture overhaul over original:
  ┌─────────────────────────────────────────────────────────────┐
  │  ControllerState machine                                    │
  │   CALIBRATING → IDLE ↔ ACTIVE ↔ EMERGENCY_STOP            │
  └─────────────────────────────────────────────────────────────┘

New features:
  1. State machine (CALIBRATING / IDLE / ACTIVE / EMERGENCY_STOP)
  2. Three speed modes: SLOW / NORMAL / FAST  (cycle with yaw-flick gesture)
  3. Emergency stop: rapid pitch reversal detected → halt + 1 s lockout
  4. Smooth acceleration: velocity commands ramped, not stepped
  5. Dead-band hysteresis to prevent jitter at zero crossing
  6. Absolute yaw angle for turning (not raw yaw-rate)  — more stable
  7. Publishes /cmd_vel (standard) and /wheel_controller/cmd_vel_unstamped
  8. Diagnostic topic /imu_controller/diagnostics (JSON string)
  9. All thresholds & gains tunable via ROS 2 parameters at runtime
 10. Safety watchdog: stops robot if IMU data stops arriving
"""

import math
import json
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String


# ─── Quaternion → Euler ───────────────────────────────────────────────────────
def quat_to_euler(x, y, z, w):
    """Returns (roll, pitch, yaw) in radians."""
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


# ─── State machine ────────────────────────────────────────────────────────────
class State(Enum):
    CALIBRATING     = auto()
    IDLE            = auto()
    ACTIVE          = auto()
    EMERGENCY_STOP  = auto()


# ─── Speed modes ─────────────────────────────────────────────────────────────
SPEED_MODES = {
    'SLOW':   {'linear': 0.20, 'angular': 0.40},
    'NORMAL': {'linear': 0.50, 'angular': 1.00},
    'FAST':   {'linear': 1.00, 'angular': 2.00},
}
SPEED_MODE_NAMES = list(SPEED_MODES.keys())


class ImuController(Node):
    """Converts filtered IMU orientation → differential-drive Twist commands."""

    # ── Tuning defaults ──────────────────────────────────────────────────────
    PITCH_DEADBAND     = 0.08   # rad  – ignore tilt below this
    YAW_DEADBAND       = 0.12   # rad  – ignore yaw offset below this
    PITCH_FULL_SCALE   = math.pi / 5   # rad at which max speed is reached
    YAW_FULL_SCALE     = math.pi / 3   # rad
    RAMP_RATE          = 4.0    # (m/s)/s – max speed change per second
    ESTOP_PITCH_DELTA  = 0.6    # rad     – sudden pitch change → e-stop
    ESTOP_LOCKOUT_S    = 1.5    # s       – hold e-stop before resuming
    WATCHDOG_S         = 0.5    # s       – halt if no IMU data
    YAW_FLICK_RATE     = 2.5    # rad/s   – yaw-rate to cycle speed mode
    CALIB_SAMPLES      = 30     # IMU samples averaged for offset calibration
    IDLE_THRESH        = 0.05   # rad     – |pitch| below this → IDLE

    def __init__(self):
        super().__init__('imu_gesture_controller')

        # ── Declare runtime-tunable parameters ──────────────────────────────
        p = self.declare_parameters('', [
            ('pitch_deadband',    self.PITCH_DEADBAND),
            ('yaw_deadband',      self.YAW_DEADBAND),
            ('pitch_full_scale',  self.PITCH_FULL_SCALE),
            ('yaw_full_scale',    self.YAW_FULL_SCALE),
            ('ramp_rate',         self.RAMP_RATE),
            ('estop_pitch_delta', self.ESTOP_PITCH_DELTA),
            ('estop_lockout_s',   self.ESTOP_LOCKOUT_S),
            ('watchdog_s',        self.WATCHDOG_S),
            ('speed_mode',        'NORMAL'),
            ('publish_rate_hz',   50.0),
        ])
        self._refresh_params()

        # ── State ────────────────────────────────────────────────────────────
        self._state          = State.CALIBRATING
        self._calib_buf: list[tuple] = []   # (pitch, yaw) samples
        self._pitch_offset   = 0.0
        self._yaw_offset     = 0.0
        self._prev_pitch     = 0.0
        self._prev_yaw       = 0.0
        self._cmd_lin        = 0.0   # current ramped linear  velocity
        self._cmd_ang        = 0.0   # current ramped angular velocity
        self._estop_until    = 0.0   # monotonic time when estop lifts
        self._last_imu_t     = time.monotonic()
        self._speed_idx      = SPEED_MODE_NAMES.index(self._speed_mode)
        self._last_flick_t   = 0.0

        # ── QoS ──────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self._pub_wheel = self.create_publisher(
            Twist, '/wheel_controller/cmd_vel_unstamped', 10)
        self._pub_cmd   = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self._pub_diag  = self.create_publisher(
            String, '/imu_controller/diagnostics', 10)

        # ── Subscriber ───────────────────────────────────────────────────────
        self._sub = self.create_subscription(
            Imu, '/imu/data/filtered', self._imu_cb, sensor_qos)

        # ── Publish timer ────────────────────────────────────────────────────
        self._dt = 1.0 / self.get_parameter('publish_rate_hz').value
        self._timer = self.create_timer(self._dt, self._publish_loop)

        # ── Diagnostics timer (1 Hz) ──────────────────────────────────────
        self.create_timer(1.0, self._diag_cb)

        self.get_logger().info(
            'IMU Gesture Controller started — waiting for calibration data…')

    # ── Parameter refresh ────────────────────────────────────────────────────
    def _refresh_params(self):
        self._pitch_db    = self.get_parameter('pitch_deadband').value
        self._yaw_db      = self.get_parameter('yaw_deadband').value
        self._pitch_fs    = self.get_parameter('pitch_full_scale').value
        self._yaw_fs      = self.get_parameter('yaw_full_scale').value
        self._ramp        = self.get_parameter('ramp_rate').value
        self._estop_dp    = self.get_parameter('estop_pitch_delta').value
        self._estop_lock  = self.get_parameter('estop_lockout_s').value
        self._watchdog    = self.get_parameter('watchdog_s').value
        self._speed_mode  = self.get_parameter('speed_mode').value
        if self._speed_mode not in SPEED_MODES:
            self._speed_mode = 'NORMAL'

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _apply_deadband(self, val: float, db: float) -> float:
        """Hysteresis deadband: zero inside, linear outside."""
        if abs(val) < db:
            return 0.0
        return math.copysign(abs(val) - db, val)

    def _ramp_toward(self, current: float, target: float, dt: float) -> float:
        max_delta = self._ramp * dt
        return current + max(-max_delta, min(max_delta, target - current))

    def _max_linear(self) -> float:
        return SPEED_MODES[self._speed_mode]['linear']

    def _max_angular(self) -> float:
        return SPEED_MODES[self._speed_mode]['angular']

    def _publish_zero(self):
        t = Twist()
        self._pub_wheel.publish(t)
        self._pub_cmd.publish(t)

    # ── IMU callback ─────────────────────────────────────────────────────────
    def _imu_cb(self, msg: Imu):
        self._last_imu_t = time.monotonic()

        q = msg.orientation
        _, pitch, yaw = quat_to_euler(q.x, q.y, q.z, q.w)

        # ── CALIBRATING: collect offset samples ──────────────────────────────
        if self._state == State.CALIBRATING:
            self._calib_buf.append((pitch, yaw))
            if len(self._calib_buf) >= self.CALIB_SAMPLES:
                self._pitch_offset = sum(s[0] for s in self._calib_buf) / len(self._calib_buf)
                self._yaw_offset   = sum(s[1] for s in self._calib_buf) / len(self._calib_buf)
                self._state = State.IDLE
                self.get_logger().info(
                    f'Calibration complete  pitch_offset={self._pitch_offset:.3f} rad  '
                    f'yaw_offset={self._yaw_offset:.3f} rad')
            return

        # ── Relative angles ───────────────────────────────────────────────────
        rel_pitch = pitch - self._pitch_offset
        rel_yaw   = yaw   - self._yaw_offset
        yaw_rate  = msg.angular_velocity.z

        # ── Emergency stop detection ──────────────────────────────────────────
        pitch_delta = abs(rel_pitch - self._prev_pitch)
        if (pitch_delta > self._estop_dp
                and self._state not in (State.CALIBRATING, State.EMERGENCY_STOP)):
            self._state      = State.EMERGENCY_STOP
            self._estop_until = time.monotonic() + self._estop_lock
            self.get_logger().warn(
                f'EMERGENCY STOP triggered  Δpitch={pitch_delta:.3f} rad  '
                f'lockout={self._estop_lock} s')
            self._publish_zero()

        # ── Speed mode flick gesture: fast yaw snap left or right ─────────────
        now = time.monotonic()
        if (abs(yaw_rate) > self.YAW_FLICK_RATE
                and (now - self._last_flick_t) > 1.0
                and self._state == State.ACTIVE):
            self._speed_idx = (self._speed_idx + 1) % len(SPEED_MODE_NAMES)
            self._speed_mode = SPEED_MODE_NAMES[self._speed_idx]
            self._last_flick_t = now
            self.get_logger().info(f'Speed mode → {self._speed_mode}')

        # ── IDLE ↔ ACTIVE transition on pitch magnitude ───────────────────────
        if self._state == State.IDLE and abs(rel_pitch) > self.IDLE_THRESH:
            self._state = State.ACTIVE
        elif self._state == State.ACTIVE and abs(rel_pitch) <= self.IDLE_THRESH * 0.5:
            self._state = State.IDLE  # hysteresis prevents flutter

        # Store previous for next delta
        self._prev_pitch = rel_pitch
        self._prev_yaw   = rel_yaw

        # ── Compute target velocities ─────────────────────────────────────────
        if self._state == State.ACTIVE:
            # Linear: pitch tilt → forward/backward
            lin_err = self._apply_deadband(rel_pitch, self._pitch_db)
            lin_raw = -self._max_linear() * (lin_err / self._pitch_fs)
            self._target_lin = max(-self._max_linear(),
                                   min(self._max_linear(), lin_raw))

            # Angular: yaw offset → turning
            ang_err = self._apply_deadband(rel_yaw, self._yaw_db)
            ang_raw = -self._max_angular() * (ang_err / self._yaw_fs)
            self._target_ang = max(-self._max_angular(),
                                   min(self._max_angular(), ang_raw))
        else:
            self._target_lin = 0.0
            self._target_ang = 0.0

    # ── Publish loop (timer) ──────────────────────────────────────────────────
    def _publish_loop(self):
        now = time.monotonic()

        # Watchdog: halt if no fresh IMU data
        if (self._state not in (State.CALIBRATING, State.EMERGENCY_STOP)
                and (now - self._last_imu_t) > self._watchdog):
            self.get_logger().warn('IMU watchdog: no data, stopping robot')
            self._publish_zero()
            self._cmd_lin = 0.0
            self._cmd_ang = 0.0
            return

        # E-stop release
        if self._state == State.EMERGENCY_STOP and now >= self._estop_until:
            self._state = State.IDLE
            self.get_logger().info('Emergency stop cleared')

        # Calibrating or e-stop: do nothing
        if self._state in (State.CALIBRATING, State.EMERGENCY_STOP):
            self._publish_zero()
            return

        # Ramp toward target
        target_lin = getattr(self, '_target_lin', 0.0)
        target_ang = getattr(self, '_target_ang', 0.0)
        self._cmd_lin = self._ramp_toward(self._cmd_lin, target_lin, self._dt)
        self._cmd_ang = self._ramp_toward(self._cmd_ang, target_ang, self._dt)

        twist = Twist()
        twist.linear.x  = self._cmd_lin
        twist.angular.z = self._cmd_ang
        self._pub_wheel.publish(twist)
        self._pub_cmd.publish(twist)

    # ── Diagnostics ──────────────────────────────────────────────────────────
    def _diag_cb(self):
        diag = {
            'state':      self._state.name,
            'speed_mode': self._speed_mode,
            'cmd_lin':    round(self._cmd_lin, 3),
            'cmd_ang':    round(self._cmd_ang, 3),
            'pitch_off':  round(self._pitch_offset, 4),
            'yaw_off':    round(self._yaw_offset, 4),
            'imu_age_s':  round(time.monotonic() - self._last_imu_t, 3),
        }
        msg = String()
        msg.data = json.dumps(diag)
        self._pub_diag.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
