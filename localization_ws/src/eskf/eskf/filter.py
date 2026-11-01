"""Eight-state SO(2) ESKF. No ROS dependencies and no absolute observations.
Nominal/error ordering: x,y,vx_world,vy_world,yaw,bg_z,ba_x,ba_y.
SO(2)'s additive angle error has identity injection/reset Jacobian.
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class Noise:
    acceleration: float = 0.6       # m/s²/sqrt(Hz), continuous white-noise density
    gyroscope: float = 0.035       # rad/s/sqrt(Hz)
    gyro_bias_walk: float = 0.0005
    accel_bias_walk: float = 0.04
    wheel_x: float = 0.05          # m/s, observation standard deviation
    wheel_y: float = 0.05


class PlanarESKF:
    def __init__(self, noise=None, lever_arm=(0.08, 0.0, 0.0), alpha_time_constant=0.05,
                 stationary_bias_only=True):
        self.noise = noise or Noise()
        if any(not np.isfinite(v) or v <= 0 for v in vars(self.noise).values()):
            raise ValueError('all noise values must be positive and finite')
        self.lever_arm = np.asarray(lever_arm, dtype=float)
        if self.lever_arm.shape != (3,) or not np.isfinite(self.lever_arm).all():
            raise ValueError('lever arm must contain three finite values')
        if not np.isfinite(alpha_time_constant) or alpha_time_constant < 0:
            raise ValueError('alpha_time_constant must be nonnegative')
        self.alpha_tau = alpha_time_constant
        self.stationary_bias_only = stationary_bias_only
        self.x = np.zeros(8)
        self.p = np.diag([.01, .01, .04, .04, .03, .0004, .04, .04])
        self.ready = False
        self.previous_omega = None
        self.alpha = self.yaw_rate = self.wheel_nis = 0.0

    def initialize(self, accel_body, gyro_body):
        accel = np.asarray(accel_body, dtype=float)
        gyro = np.asarray(gyro_body, dtype=float)
        if accel.shape != (3,) or gyro.shape != (3,) or not np.isfinite([accel, gyro]).all():
            raise ValueError('invalid calibration sample')
        self.x[:] = 0
        self.x[5] = gyro[2]
        self.x[6:8] = accel[:2]
        self.previous_omega = 0.0
        self.alpha = self.yaw_rate = 0.0
        self.ready = True

    def predict(self, accel_body, gyro_body, dt):
        if not self.ready:
            raise ValueError('filter is not initialized')
        a, w = np.asarray(accel_body, dtype=float), np.asarray(gyro_body, dtype=float)
        if a.shape != (3,) or w.shape != (3,) or not np.isfinite([a, w]).all() or not np.isfinite(dt) or not 0 < dt <= .1:
            raise ValueError('invalid IMU input or prediction interval')
        omega = w[2] - self.x[5]
        if self.previous_omega is not None:
            smoothing = dt / (self.alpha_tau + dt)
            self.alpha += smoothing * ((omega-self.previous_omega)/dt - self.alpha)
        self.previous_omega = omega
        rx, ry = self.lever_arm[:2]
        # a_sensor = a_center + alpha x r + omega x (omega x r).
        body_a = a[:2] + [self.alpha*ry + omega*omega*rx,
                         -self.alpha*rx + omega*omega*ry] - self.x[6:8]
        c, s = np.cos(self.x[4]), np.sin(self.x[4])
        rotation = np.array([[c, -s], [s, c]])
        world_a = rotation @ body_a
        f = np.eye(8)
        f[0:2, 2:4] = np.eye(2)*dt
        yaw_jac = np.array([-world_a[1], world_a[0]])
        f[0:2, 4] = yaw_jac * .5*dt*dt
        f[2:4, 4] = yaw_jac * dt
        f[0:2, 6:8] = -rotation*.5*dt*dt
        f[2:4, 6:8] = -rotation*dt
        f[4, 5] = -dt
        self.x[0:2] += self.x[2:4]*dt + .5*world_a*dt*dt
        self.x[2:4] += world_a*dt
        self.x[4] += omega*dt
        self.yaw_rate = omega
        # Continuous white-noise integration; independent of timer frequency.
        q = np.zeros((8, 8))
        variance = self.noise.acceleration**2
        for pos, vel in [(0, 2), (1, 3)]:
            q[pos, pos] = variance*dt**3/3
            q[pos, vel] = q[vel, pos] = variance*dt**2/2
            q[vel, vel] = variance*dt
        q[4, 4] = self.noise.gyroscope**2*dt
        q[5, 5] = self.noise.gyro_bias_walk**2*dt
        q[6, 6] = q[7, 7] = self.noise.accel_bias_walk**2*dt
        self.p = f @ self.p @ f.T + q
        self.p = (self.p+self.p.T)*.5
        if not np.isfinite(self.x).all() or not np.isfinite(self.p).all():
            raise FloatingPointError('non-finite prediction')

    def update(self, residual, h, r, lock_biases=False):
        innovation_cov = h @ self.p @ h.T + r
        gain = np.linalg.solve(innovation_cov, h @ self.p).T
        if lock_biases:
            # Biases are estimated only from verified stationary observations.
            # Joseph form remains valid for this constrained gain.
            gain[5:8, :] = 0
        self.x += gain @ residual
        reset = np.eye(8)-gain @ h
        self.p = reset @ self.p @ reset.T + gain @ r @ gain.T
        self.p = (self.p+self.p.T)*.5
        if not np.isfinite(self.x).all() or not np.isfinite(self.p).all():
            raise FloatingPointError('non-finite filter state')

    def wheel_update(self, velocity_body):
        measured = np.asarray(velocity_body, dtype=float)
        if measured.shape != (2,) or not np.isfinite(measured).all():
            raise ValueError('wheel update requires two finite body velocities')
        c, s = np.cos(self.x[4]), np.sin(self.x[4])
        rotation = np.array([[c, s], [-s, c]])
        predicted = rotation @ self.x[2:4]
        h = np.zeros((2, 8))
        h[:, 2:4] = rotation
        h[:, 4] = [predicted[1], -predicted[0]]
        residual = measured-predicted
        r = np.diag([self.noise.wheel_x**2, self.noise.wheel_y**2])
        innovation_cov = h @ self.p @ h.T+r
        self.wheel_nis = float(residual @ np.linalg.solve(innovation_cov, residual))
        # A large residual must not permanently remove wheel velocity feedback.
        self.update(residual, h, r, lock_biases=self.stationary_bias_only)

    def stationary_update(self, accel_body, gyro_body):
        # Caller confirms a stationary dwell, not just a single zero encoder frame.
        h = np.zeros((5, 8))
        h[0, 2] = h[1, 3] = h[2, 5] = h[3, 6] = h[4, 7] = 1
        observation = np.array([0, 0, gyro_body[2], accel_body[0], accel_body[1]])
        predicted = self.x[[2, 3, 5, 6, 7]]
        self.update(observation-predicted, h, np.diag([.015**2, .015**2, .01**2, .08**2, .08**2]))
