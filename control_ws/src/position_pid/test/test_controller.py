import math
import pytest
from position_pid.controller import Config, PositionPID, angle_error, rotate


def test_yaw_shortest_path_and_world_body_conversion():
    assert abs(angle_error(-math.pi+.05, math.pi-.05)-.1) < 1e-9
    p = PositionPID(Config(max_linear_accel=100, max_angular_accel=100))
    vx, vy, wz = p.step((0, 0, math.pi/2), (0, 0, 0), (1, 0, math.pi/2), .02)
    assert abs(vx) < 1e-9 and vy < 0 and abs(wz) < 1e-9


def test_vector_speed_and_acceleration_bounds():
    p = PositionPID()
    previous = (0, 0, 0)
    for _ in range(100):
        cmd = p.step((0, 0, 0), (0, 0, 0), (10, 10, 2), .02)
        assert math.hypot(*cmd[:2]) <= .2+1e-10
        assert abs(cmd[2]) <= .5+1e-10
        assert math.hypot(cmd[0]-previous[0], cmd[1]-previous[1]) <= .3*.02+1e-10
        assert abs(cmd[2]-previous[2]) <= .8*.02+1e-10
        previous = cmd


def test_antiwindup_and_target_change():
    p = PositionPID(Config(ki_xy=.3, ki_yaw=.1))
    for _ in range(1000):
        p.step((0, 0, 0), (0, 0, 0), (100, 0, 2), .02)
    assert abs(p.integral[0]) < .01
    old = p.previous.copy()
    p.reset(keep_command=True)
    assert p.previous == old
    p.reset()
    assert p.previous == [0, 0, 0]


def test_closed_loop_planar_simulation():
    p = PositionPID()
    pose = [0., 0., 0.]
    velocity = [0., 0., 0.]
    goal = [1., -.5, 1.]
    for _ in range(2000):
        velocity = p.step(pose, velocity, goal, .02)
        wx, wy = rotate(*velocity[:2], pose[2])
        pose[0] += wx*.02; pose[1] += wy*.02; pose[2] += velocity[2]*.02
    assert math.hypot(pose[0]-goal[0], pose[1]-goal[1]) <= .021
    assert abs(angle_error(goal[2], pose[2])) <= .031
    assert p.reached
    assert math.hypot(*velocity[:2]) < 1e-6


def test_invalid_values_and_zero_gain():
    with pytest.raises(ValueError):
        PositionPID(Config(max_linear_speed=0))
    p = PositionPID()
    with pytest.raises(ValueError):
        p.step((math.nan, 0, 0), (0, 0, 0), (1, 0, 0), .02)
    assert p.step((0, 0, 0), (0, 0, 0), (0, 0, 0), .02) == (0, 0, 0)
