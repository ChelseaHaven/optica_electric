import math
import numpy as np
import pytest
from mpc.controller import Config,MPC


def test_direction_limits_and_short_yaw():
    f=MPC();previous=np.zeros(3)
    for _ in range(30):
        cmd=np.array(f.step([0,0,math.pi/2],[0,0,0],[2,0,math.pi/2],.05))
        assert abs(cmd[0])<1e-5 and cmd[1]<0
        assert np.linalg.norm(cmd[:2])<=.20001 and abs(cmd[2])<=.50001
        assert np.linalg.norm(cmd[:2]-previous[:2])<=.3*.05+1e-5
        assert abs(cmd[2]-previous[2])<=.8*.05+1e-5
        assert f.diagnostics['success']
        previous=cmd
    f.reset()
    assert f.step([0,0,math.pi-.05],[0,0,0],[0,0,-math.pi+.05],.05)[2]>0


def test_closed_loop_replanning_and_stationary_target():
    f=MPC();pose=np.zeros(3);velocity=np.zeros(3);goal=np.array([.3,-.15,.4])
    times=[]
    for _ in range(180):
        velocity=np.array(f.step(pose,velocity,goal,.05))
        c,s=np.cos(pose[2]),np.sin(pose[2]);vx,vy,w=velocity
        pose+=np.array([c*vx-s*vy,s*vx+c*vy,w])*.05
        times.append(f.diagnostics['solve_ms'])
    assert np.linalg.norm(pose[:2]-goal[:2])<.025
    assert abs(pose[2]-goal[2])<.035
    assert f.reached
    print('MPC solve time median/max ms:',np.median(times),max(times))


def test_mixed_heading_constraints_and_changed_goal():
    f=MPC();pose=np.array([0.,0.,.5]);velocity=np.zeros(3)
    for i in range(60):
        goal=[.5,.3,1.0] if i<30 else [-.3,.2,-.5]
        if i==30:f.reset(keep_command=True)
        cmd=np.array(f.step(pose,velocity,goal,.05))
        assert np.linalg.norm(cmd[:2])<=.20001
        assert f.diagnostics['constraint_violation']<=1e-5
        c,s=np.cos(pose[2]),np.sin(pose[2])
        pose+=np.array([c*cmd[0]-s*cmd[1],s*cmd[0]+c*cmd[1],cmd[2]])*.05
        velocity=cmd
    assert np.isfinite(pose).all()
    with pytest.raises(ValueError):f.step([math.nan,0,0],[0,0,0],[0,0,0],.05)
