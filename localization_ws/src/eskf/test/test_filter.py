import numpy as np
import pytest
from localization_common.geometry import wheel_velocity, rotation_matrix, odometry_covariances, WHEEL_NAMES
from eskf.filter import PlanarESKF, Noise


def ready(**kwargs):
    f = PlanarESKF(**kwargs)
    f.initialize([0, 0, 9.80665], [0, 0, 0])
    return f


def test_geometry_and_named_order():
    assert np.allclose(wheel_velocity(WHEEL_NAMES, [1]*4, 1, 1, 1), [1, 0, 0])
    assert np.allclose(wheel_velocity(WHEEL_NAMES, [-1, 1, 1, -1], 1, 1, 1), [0, 1, 0])
    values = dict(zip(WHEEL_NAMES, [-1, 1, -1, 1]))
    assert np.allclose(wheel_velocity(WHEEL_NAMES[::-1], [values[n] for n in WHEEL_NAMES[::-1]], 1, 1, 1), [0, 0, 1])
    with pytest.raises(ValueError):
        rotation_matrix([1]*9)


def test_straight_and_stop():
    f = ready()
    for i in range(200):
        f.predict([0, 0, 9.80665], [0, 0, 0], .02)
        f.wheel_update([.2 if i < 100 else 0, 0])
    assert abs(f.x[0]-.4) < .02
    assert abs(f.x[2]) < .001
    assert abs(f.x[1]) < 1e-8


def test_rotating_lever_arm_no_false_translation():
    f = ready(alpha_time_constant=0)
    previous = yaw = 0
    for i in range(300):
        omega = .6*np.sin(i*.02)
        alpha = (omega-previous)/.02
        f.predict([-.08*omega**2, .08*alpha, 9.80665], [0, 0, omega], .02)
        f.wheel_update([0, 0])
        yaw += omega*.02
        previous = omega
    assert np.linalg.norm(f.x[:4]) < 1e-8
    assert abs(f.x[4]-yaw) < 1e-8


def test_acceleration_disturbance_does_not_lock_out_wheels_or_learn_bad_bias():
    f = ready()
    for i in range(200):
        f.predict([5.9 if i < 100 else 0, 0, 9.80665], [0, 0, 0], .02)
        f.wheel_update([0, 0])
    assert abs(f.x[0]) < .05
    assert abs(f.x[2]) < 1e-6
    assert np.allclose(f.x[5:8], 0)


def test_stationary_bias_and_covariance():
    f = ready()
    for _ in range(200):
        f.predict([.02, -.03, 9.80665], [0, 0, .005], .02)
        f.wheel_update([0, 0])
        f.stationary_update([.02, -.03, 9.80665], [0, 0, .005])
        assert np.linalg.eigvalsh(f.p).min() > -1e-12
        assert np.allclose(f.p, f.p.T)
    assert np.allclose(f.x[5:8], [.005, .02, -.03], atol=.002)
    pose, twist = odometry_covariances(f.x, f.p, .035**2)
    assert len(pose) == len(twist) == 36
    assert pose[14] == 1e6


def test_noise_integration_not_dependent_on_sample_rate():
    states = []
    for dt in [.01, .02]:
        f = ready(noise=Noise(gyro_bias_walk=1e-12, accel_bias_walk=1e-12))
        f.p[:] = 0
        for _ in range(round(1/dt)):
            f.predict([0, 0, 9.80665], [0, 0, 0], dt)
        states.append(f.p)
    assert np.allclose(states[0][:4, :4], states[1][:4, :4], atol=1e-10)
