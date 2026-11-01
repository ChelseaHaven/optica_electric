"""ROS-independent sensor geometry, with SI units and X forward/Y left/Z up."""
import numpy as np

WHEEL_NAMES = ('front_left_wheel', 'front_right_wheel', 'rear_left_wheel', 'rear_right_wheel')


def rotation_matrix(values):
    matrix = np.asarray(values, dtype=float).reshape(3, 3)
    if not np.isfinite(matrix).all() or not np.allclose(matrix @ matrix.T, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(matrix), 1, atol=1e-5):
        raise ValueError('imu_to_body must be a proper orthogonal 3x3 rotation')
    return matrix


def wheel_velocity(names, velocities, radius, track, wheelbase):
    if len(names) != len(velocities) or len(set(names)) != len(names):
        raise ValueError('wheel names must be unique and match velocity entries')
    mapping = dict(zip(names, velocities))
    values = np.array([mapping[name] for name in WHEEL_NAMES], dtype=float) * radius
    if not np.isfinite(values).all():
        raise ValueError('invalid wheel velocity')
    a, b, c, d = values
    return np.array([(a+b+c+d)/4, (-a+b+c-d)/4, (-a+b-c+d)/(2*(track+wheelbase))])


def odometry_covariances(state, covariance, gyro_variance):
    pose = np.zeros((6, 6))
    pose[2, 2] = pose[3, 3] = pose[4, 4] = 1e6
    indices = [0, 1, 5]
    pose[np.ix_(indices, indices)] = covariance[np.ix_([0, 1, 4], [0, 1, 4])]
    yaw = state[4]
    c, s = np.cos(yaw), np.sin(yaw)
    body_vx, body_vy = c*state[2]+s*state[3], -s*state[2]+c*state[3]
    jac = np.zeros((3, 8))
    jac[0, 2:5] = [c, s, body_vy]
    jac[1, 2:5] = [-s, c, -body_vx]
    jac[2, 5] = -1
    body = jac @ covariance @ jac.T
    body[2, 2] += gyro_variance
    twist = np.zeros((6, 6))
    twist[2, 2] = twist[3, 3] = twist[4, 4] = 1e6
    twist[np.ix_(indices, indices)] = body
    return pose.ravel().tolist(), twist.ravel().tolist()
