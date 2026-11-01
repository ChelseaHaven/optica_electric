"""ROS-independent holonomic position PID. SI units; derivative on measurement."""
from dataclasses import dataclass
import math


def angle_error(target, actual):
    return math.atan2(math.sin(target-actual), math.cos(target-actual))


def rotate(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c*x-s*y, s*x+c*y


def limit_vector(x, y, maximum):
    magnitude = math.hypot(x, y)
    scale = min(1.0, maximum/magnitude) if magnitude else 1.0
    return x*scale, y*scale


@dataclass
class Config:
    kp_xy: float = .8
    ki_xy: float = 0.0
    kd_xy: float = .15
    kp_yaw: float = 1.5
    ki_yaw: float = 0.0
    kd_yaw: float = .1
    max_linear_speed: float = .2
    max_angular_speed: float = .5
    max_linear_accel: float = .3
    max_angular_accel: float = .8
    integral_limit_xy: float = .5
    integral_limit_yaw: float = 1.0
    position_tolerance: float = .02
    yaw_tolerance: float = .03
    settled_linear_speed: float = .03
    settled_angular_speed: float = .05

    def validate(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(name+' must be nonnegative and finite')
            if not name.startswith(('kp_', 'ki_', 'kd_')) and value == 0:
                raise ValueError(name+' must be positive')


class PositionPID:
    def __init__(self, config=None):
        self.cfg = config or Config()
        self.cfg.validate()
        self.reset()

    def reset(self, keep_command=False):
        previous = self.previous.copy() if keep_command else [0.0]*3
        self.integral = [0.0]*3
        self.previous = previous
        self.error = [0.0]*3
        self.reached = False

    def step(self, pose, body_velocity, goal, dt):
        if len(pose) != 3 or len(body_velocity) != 3 or len(goal) != 3:
            raise ValueError('pose, velocity and goal must have three components')
        if not all(math.isfinite(v) for v in (*pose, *body_velocity, *goal, dt)) or not 0 < dt <= .1:
            raise ValueError('invalid control input or period')
        cfg = self.cfg
        error = [goal[0]-pose[0], goal[1]-pose[1], angle_error(goal[2], pose[2])]
        self.error = error.copy()
        linear_inside = math.hypot(error[0], error[1]) <= cfg.position_tolerance
        angular_inside = abs(error[2]) <= cfg.yaw_tolerance
        self.reached = (linear_inside and angular_inside and
                        math.hypot(*body_velocity[:2]) <= cfg.settled_linear_speed and
                        abs(body_velocity[2]) <= cfg.settled_angular_speed)
        if linear_inside:
            error[0] = error[1] = 0.0
            self.integral[0] = self.integral[1] = 0.0
        if angular_inside:
            error[2] = 0.0
            self.integral[2] = 0.0
        world_velocity = [*rotate(*body_velocity[:2], pose[2]), body_velocity[2]]
        kp, ki, kd = [cfg.kp_xy]*2+[cfg.kp_yaw], [cfg.ki_xy]*2+[cfg.ki_yaw], [cfg.kd_xy]*2+[cfg.kd_yaw]
        limits = [cfg.integral_limit_xy]*2+[cfg.integral_limit_yaw]
        proposed = [max(-limits[i], min(limits[i], self.integral[i]+error[i]*dt)) if ki[i] else 0.0 for i in range(3)]
        raw = [kp[i]*error[i]+ki[i]*proposed[i]-kd[i]*world_velocity[i] for i in range(3)]
        # Inside the target deadband, stop instead of amplifying measurement noise.
        if linear_inside:
            raw[0] = raw[1] = 0.0
        if angular_inside:
            raw[2] = 0.0
        vx, vy = limit_vector(*raw[:2], cfg.max_linear_speed)
        vx, vy = rotate(vx, vy, -pose[2])
        wz = max(-cfg.max_angular_speed, min(cfg.max_angular_speed, raw[2]))
        dx, dy = limit_vector(vx-self.previous[0], vy-self.previous[1], cfg.max_linear_accel*dt)
        dz = max(-cfg.max_angular_accel*dt, min(cfg.max_angular_accel*dt, wz-self.previous[2]))
        command = [self.previous[0]+dx, self.previous[1]+dy, self.previous[2]+dz]
        applied = [*rotate(*command[:2], pose[2]), command[2]]
        # Conditional integration also accounts for speed and acceleration limits.
        for i in range(3):
            if error[i]*(raw[i]-applied[i]) <= 1e-12:
                self.integral[i] = proposed[i]
        self.previous = command
        return tuple(command)
