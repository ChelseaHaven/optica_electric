"""Pure wheel velocity odometry: constant-body-twist SE(2) integration."""
import math
import numpy as np


def coefficients(theta):
    if abs(theta) < 1e-4:
        s = 1-theta**2/6+theta**4/120
        c = theta/2-theta**3/24+theta**5/720
        ds = -theta/3+theta**3/30
        dc = .5-theta**2/8+theta**4/144
    else:
        s = math.sin(theta)/theta
        c = (1-math.cos(theta))/theta
        ds = (theta*math.cos(theta)-math.sin(theta))/theta**2
        dc = (theta*math.sin(theta)-(1-math.cos(theta)))/theta**2
    return np.array([[s,-c],[c,s]]), np.array([[ds,-dc],[dc,ds]])


def twist_covariance(radius, track, wheelbase, wheel_std):
    values = [radius, track, wheelbase, wheel_std]
    if not all(math.isfinite(v) and v > 0 for v in values):
        raise ValueError('wheel geometry and noise must be positive and finite')
    arm = (track+wheelbase)/2
    matrix = radius/4*np.array([[1,1,1,1],[-1,1,1,-1],[-1/arm,1/arm,-1/arm,1/arm]])
    return matrix @ (np.eye(4)*wheel_std**2) @ matrix.T


class WheelOdometry:
    def __init__(self, velocity_covariance, position_std=.001, yaw_std=.001):
        self.velocity_covariance = np.asarray(velocity_covariance,dtype=float)
        if self.velocity_covariance.shape != (3,3) or not np.isfinite(self.velocity_covariance).all():
            raise ValueError('velocity covariance must be finite 3x3')
        if not np.allclose(self.velocity_covariance,self.velocity_covariance.T) or np.linalg.eigvalsh(self.velocity_covariance).min() < 0:
            raise ValueError('velocity covariance must be symmetric positive semidefinite')
        if not all(math.isfinite(v) and v > 0 for v in (position_std,yaw_std)):
            raise ValueError('initial standard deviations must be positive and finite')
        self.pose = np.zeros(3)
        self.covariance = np.diag([position_std**2,position_std**2,yaw_std**2])

    def step(self, velocity, dt):
        v = np.asarray(velocity,dtype=float)
        if v.shape != (3,) or not np.isfinite(v).all() or not math.isfinite(dt) or not 0 < dt <= .1:
            raise ValueError('invalid body velocity or integration period')
        b, db = coefficients(v[2]*dt)
        c,s = math.cos(self.pose[2]),math.sin(self.pose[2])
        rotation = np.array([[c,-s],[s,c]])
        delta = rotation @ b @ v[:2]*dt
        f = np.eye(3)
        f[:2,2] = [-delta[1],delta[0]]
        g = np.zeros((3,3))
        g[:2,:2] = rotation @ b*dt
        g[:2,2] = rotation @ db @ v[:2]*dt**2
        g[2,2] = dt
        self.pose[:2] += delta
        self.pose[2] += v[2]*dt
        self.covariance = f @ self.covariance @ f.T+g @ self.velocity_covariance @ g.T
        self.covariance = (self.covariance+self.covariance.T)/2
        if not np.isfinite(self.pose).all() or not np.isfinite(self.covariance).all():
            raise FloatingPointError('non-finite wheel odometry')

    def ros_covariances(self):
        result=[]
        for covariance in (self.covariance,self.velocity_covariance):
            matrix = np.zeros((6,6))
            matrix[2,2] = matrix[3,3] = matrix[4,4] = 1e6
            matrix[np.ix_([0,1,5],[0,1,5])] = covariance
            result.append(matrix.ravel().tolist())
        return result
