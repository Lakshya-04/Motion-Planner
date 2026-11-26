"""A planar IMU derived from the base twist, for simulators with no IMU of their own (no ROS)."""
import numpy as np

GRAVITY = 9.81


class ImuModel:
    """Gyro and accelerometer in the body frame from (vx, vy, wz) samples.

    The accelerometer reads specific force: d(v)/dt + w x v, plus gravity on z for an upright base.
    `*_noise` is the white std per sample, `*_bias` the std of a constant offset drawn once per axis and run.
    The derivative is smoothed with the factor `alpha` (1 = none) because a twist from a stepped base is spiky.
    """

    def __init__(self, gyro_noise=0.0, gyro_bias=0.0, accel_noise=0.0, accel_bias=0.0, seed=0, alpha=0.3):
        self.rng = np.random.default_rng(seed)
        self.gyro_noise, self.accel_noise, self.alpha = gyro_noise, accel_noise, alpha
        self.gyro_bias = self.rng.normal(0.0, gyro_bias, 3) if gyro_bias else np.zeros(3)
        self.accel_bias = self.rng.normal(0.0, accel_bias, 3) if accel_bias else np.zeros(3)
        self.prev = None
        self.acc = np.zeros(2)

    def step(self, t, vx, vy, wz):
        """(gyro xyz rad/s, accel xyz m/s^2) at time t."""
        if self.prev is not None:
            pt, pvx, pvy = self.prev
            dt = t - pt
            if 1e-6 < dt < 1.0:
                d = np.array([(vx - pvx) / dt, (vy - pvy) / dt])
                self.acc += self.alpha * (d - self.acc)
        self.prev = (t, vx, vy)
        gyro = np.array([0.0, 0.0, wz])
        accel = np.array([self.acc[0] - wz * vy, self.acc[1] + wz * vx, GRAVITY])
        if self.gyro_noise:
            gyro = gyro + self.rng.normal(0.0, self.gyro_noise, 3)
        if self.accel_noise:
            accel = accel + self.rng.normal(0.0, self.accel_noise, 3)
        return gyro + self.gyro_bias, accel + self.accel_bias

    def covariances(self, floor=1e-6):
        """(gyro variance, accel variance) for the message; a floor so a zero is not read as 'exact'."""
        g = max(self.gyro_noise ** 2 + float(np.mean(self.gyro_bias ** 2)), floor)
        a = max(self.accel_noise ** 2 + float(np.mean(self.accel_bias ** 2)), floor)
        return g, a
