import math
import numpy as np
import pytest
from wheel_odometry.odometry import WheelOdometry,twist_covariance


def estimator():
    return WheelOdometry(twist_covariance(.0385,.196,.160,.1))


def test_straight_and_lateral():
    f=estimator()
    for _ in range(100):f.step([.2,0,0],.02)
    assert np.allclose(f.pose,[.4,0,0])
    for _ in range(100):f.step([0,.1,0],.02)
    assert np.allclose(f.pose,[.4,.2,0])


def test_rotation_and_closed_circle():
    f=estimator()
    for _ in range(100):f.step([0,0,.5],.02)
    assert np.allclose(f.pose,[0,0,1])
    f=estimator()
    for _ in range(1000):f.step([.2,0,2*math.pi/20],.02)
    assert np.linalg.norm(f.pose[:2])<1e-10
    assert abs(f.pose[2]-2*math.pi)<1e-10


def test_exact_constant_twist_and_near_zero():
    f=estimator();f.step([.2,.1,.8],.1)
    expected=[(.2*math.sin(.08)-.1*(1-math.cos(.08)))/.8,
              (.2*(1-math.cos(.08))+.1*math.sin(.08))/.8,.08]
    assert np.allclose(f.pose,expected,atol=1e-12)
    f=estimator();f.step([.2,.1,1e-12],.1)
    assert np.allclose(f.pose,[.02,.01,1e-13])


def test_covariance_and_invalid_input():
    f=estimator();rng=np.random.default_rng(12)
    for _ in range(200):
        f.step(rng.normal(size=3)*.1,.02)
        assert np.linalg.eigvalsh(f.covariance).min()>0
        assert np.allclose(f.covariance,f.covariance.T)
    pose,twist=f.ros_covariances()
    assert len(pose)==len(twist)==36 and pose[14]==1e6
    with pytest.raises(ValueError):f.step([math.nan,0,0],.02)
    with pytest.raises(ValueError):f.step([0,0,0],1)
