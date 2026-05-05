#!/usr/bin/env python3
"""
imu_smoother.py  —  Gesture Controlled Robot (Upgraded)

Improvements over original:
  - Adaptive window size based on motion magnitude
  - Median pre-filter before mean (rejects spike outliers)
  - Per-axis Butterworth-style exponential smoothing (EMA) fallback
  - Publishes /imu/data_filtered  (topic rename for clarity)
  - Monitors publish rate and logs a warning if it drops below 80 Hz
  - Covariance matrices forwarded correctly (not zeroed)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from collections import deque
import numpy as np
import time


class ImuSmoother(Node):
    # ── defaults ────────────────────────────────────────────────────────────
    WINDOW_MIN   = 5
    WINDOW_MAX   = 25
    MOTION_LOW   = 0.5    # rad/s  – threshold for "calm" → big window
    MOTION_HIGH  = 3.0    # rad/s  – threshold for "fast" → small window
    EMA_ALPHA    = 0.25   # fallback EMA weight for first few samples
    RATE_WARN_HZ = 80.0

    def __init__(self):
        super().__init__('imu_smoother_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('window_min',   self.WINDOW_MIN)
        self.declare_parameter('window_max',   self.WINDOW_MAX)
        self.declare_parameter('motion_low',   self.MOTION_LOW)
        self.declare_parameter('motion_high',  self.MOTION_HIGH)
        self.declare_parameter('ema_alpha',    self.EMA_ALPHA)
        self.declare_parameter('use_median',   True)

        self.win_min      = self.get_parameter('window_min').value
        self.win_max      = self.get_parameter('window_max').value
        self.motion_low   = self.get_parameter('motion_low').value
        self.motion_high  = self.get_parameter('motion_high').value
        self.alpha        = self.get_parameter('ema_alpha').value
        self.use_median   = self.get_parameter('use_median').value

        # ── Buffers (max size = window_max) ─────────────────────────────────
        maxlen = self.win_max
        self._bufs = {k: deque(maxlen=maxlen) for k in
                      ('ax','ay','az','gx','gy','gz')}

        # ── EMA state (used before buffer fills) ─────────────────────────────
        self._ema = {k: None for k in self._bufs}

        # ── Rate monitoring ──────────────────────────────────────────────────
        self._rate_window = deque(maxlen=100)
        self._last_pub_t  = None

        # ── QoS: best-effort, keep-last-10 to match micro-ROS publisher ──────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self._sub = self.create_subscription(
            Imu, '/imu/data_raw', self._cb, sensor_qos)
        self._pub = self.create_publisher(
            Imu, '/imu/data_filtered', 10)

        # ── Rate-check timer (every 5 s) ─────────────────────────────────────
        self.create_timer(5.0, self._check_rate)

        self.get_logger().info(
            f'ImuSmoother ready  window=[{self.win_min}..{self.win_max}]  '
            f'median={self.use_median}  ema_alpha={self.alpha}')

    # ── helpers ─────────────────────────────────────────────────────────────
    def _ema_update(self, key: str, value: float) -> float:
        if self._ema[key] is None:
            self._ema[key] = value
        else:
            self._ema[key] = self.alpha * value + (1.0 - self.alpha) * self._ema[key]
        return self._ema[key]

    def _adaptive_window(self, gyro_mag: float) -> int:
        if gyro_mag <= self.motion_low:
            return self.win_max
        if gyro_mag >= self.motion_high:
            return self.win_min
        t = (gyro_mag - self.motion_low) / (self.motion_high - self.motion_low)
        return int(self.win_max - t * (self.win_max - self.win_min))

    def _smooth(self, key: str, window: int) -> float:
        buf = self._bufs[key]
        if len(buf) < 2:
            return self._ema[key] if self._ema[key] is not None else 0.0
        arr = np.array(list(buf)[-window:])
        if self.use_median:
            # Reject outliers: mask values > 3σ from median
            med = np.median(arr)
            std = np.std(arr) + 1e-9
            arr = arr[np.abs(arr - med) < 3.0 * std]
            if len(arr) == 0:
                return float(med)
        return float(np.mean(arr))

    # ── callback ─────────────────────────────────────────────────────────────
    def _cb(self, msg: Imu):
        # Push raw values into buffers & update EMA
        raw = {
            'ax': msg.linear_acceleration.x,
            'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z,
            'gx': msg.angular_velocity.x,
            'gy': msg.angular_velocity.y,
            'gz': msg.angular_velocity.z,
        }
        for k, v in raw.items():
            self._bufs[k].append(v)
            self._ema_update(k, v)

        # Adaptive window driven by angular speed magnitude
        gyro_mag = np.sqrt(raw['gx']**2 + raw['gy']**2 + raw['gz']**2)
        win = self._adaptive_window(gyro_mag)

        # Build filtered message
        out = Imu()
        out.header                        = msg.header
        out.orientation                   = msg.orientation
        out.orientation_covariance        = msg.orientation_covariance
        out.linear_acceleration_covariance  = msg.linear_acceleration_covariance
        out.angular_velocity_covariance     = msg.angular_velocity_covariance

        out.linear_acceleration.x = self._smooth('ax', win)
        out.linear_acceleration.y = self._smooth('ay', win)
        out.linear_acceleration.z = self._smooth('az', win)
        out.angular_velocity.x    = self._smooth('gx', win)
        out.angular_velocity.y    = self._smooth('gy', win)
        out.angular_velocity.z    = self._smooth('gz', win)

        self._pub.publish(out)

        # Rate tracking
        now = time.monotonic()
        if self._last_pub_t is not None:
            self._rate_window.append(1.0 / max(now - self._last_pub_t, 1e-6))
        self._last_pub_t = now

    def _check_rate(self):
        if len(self._rate_window) < 10:
            return
        hz = float(np.mean(self._rate_window))
        if hz < self.RATE_WARN_HZ:
            self.get_logger().warn(
                f'IMU smoother publish rate low: {hz:.1f} Hz  '
                f'(expected >= {self.RATE_WARN_HZ} Hz) — check USB/serial connection')
        else:
            self.get_logger().debug(f'IMU smoother rate: {hz:.1f} Hz')


def main(args=None):
    rclpy.init(args=args)
    node = ImuSmoother()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
