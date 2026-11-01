"""Constrained finite-horizon holonomic MPC, solved as a convex quadratic program.
State: world x/y and unwrapped yaw. Decision: world vx/vy and yaw rate.
Only the first action is rotated into the current body frame and applied.
"""
from dataclasses import dataclass
import math
import time
import numpy as np
from scipy.optimize import minimize,LinearConstraint


@dataclass
class Config:
    horizon_steps: int = 12
    prediction_dt: float = .05
    weight_position: float = 20.0
    weight_yaw: float = 10.0
    terminal_weight_multiplier: float = 5.0
    weight_velocity: float = .1
    weight_yaw_rate: float = .1
    weight_velocity_change: float = .5
    weight_yaw_rate_change: float = .2
    max_linear_speed: float = .2
    max_angular_speed: float = .5
    max_linear_accel: float = .3
    max_angular_accel: float = .8
    position_tolerance: float = .02
    yaw_tolerance: float = .03
    settled_linear_speed: float = .03
    settled_angular_speed: float = .05
    solver_max_iterations: int = 60
    solver_ftol: float = 1e-6

    def validate(self):
        for name,value in vars(self).items():
            if not math.isfinite(value) or value<=0:
                raise ValueError(name+' must be positive and finite')
        if not isinstance(self.horizon_steps,int) or not 2<=self.horizon_steps<=40:
            raise ValueError('horizon_steps must be an integer between 2 and 40')
        if not isinstance(self.solver_max_iterations,int):
            raise ValueError('solver_max_iterations must be an integer')


class MPC:
    def __init__(self,config=None):
        self.cfg=config or Config();self.cfg.validate()
        n=self.cfg.horizon_steps
        self.s=np.kron(np.tril(np.ones((n,n))),np.eye(3)*self.cfg.prediction_dt)
        q=np.tile([self.cfg.weight_position,self.cfg.weight_position,self.cfg.weight_yaw],n)
        q[-3:]*=self.cfg.terminal_weight_multiplier
        self.q=q
        r=np.tile([self.cfg.weight_velocity,self.cfg.weight_velocity,self.cfg.weight_yaw_rate],n)
        self.rd=np.tile([self.cfg.weight_velocity_change,self.cfg.weight_velocity_change,self.cfg.weight_yaw_rate_change],n)
        self.d=np.kron(np.eye(n)-np.eye(n,k=-1),np.eye(3))
        self.h=2*(self.s.T @ (q[:,None]*self.s)+np.diag(r)+self.d.T @ (self.rd[:,None]*self.d))
        # Inscribed octagon: every feasible linear velocity lies inside the disk.
        angles=np.arange(8)*math.pi/4
        t=np.zeros((10,3));t[:8,0]=np.cos(angles);t[:8,1]=np.sin(angles)
        t[8,2]=1;t[9,2]=-1
        self.t=t
        velocity_constraints=np.kron(np.eye(n),t)
        self.constraints=np.vstack((velocity_constraints,velocity_constraints @ self.d))
        self.reset()

    def reset(self,keep_command=False):
        previous=self.previous.copy() if keep_command else [0.0]*3
        self.previous=previous;self.plan=None
        self.error=[0.0]*3;self.reached=False;self.diagnostics={}

    def step(self,pose,body_velocity,goal,dt):
        start=time.perf_counter()
        pose,body_velocity,goal=map(lambda v:np.asarray(v,dtype=float),(pose,body_velocity,goal))
        if any(v.shape!=(3,) or not np.isfinite(v).all() for v in (pose,body_velocity,goal)) or not math.isfinite(dt) or not 0<dt<=.1:
            raise ValueError('invalid MPC input or period')
        cfg=self.cfg;n=cfg.horizon_steps
        yaw_error=math.atan2(math.sin(goal[2]-pose[2]),math.cos(goal[2]-pose[2]))
        error=np.array([goal[0]-pose[0],goal[1]-pose[1],yaw_error])
        self.error=error.tolist()
        linear_inside=np.linalg.norm(error[:2])<=cfg.position_tolerance
        angular_inside=abs(error[2])<=cfg.yaw_tolerance
        target=error.copy()
        if linear_inside:target[:2]=0
        if angular_inside:target[2]=0
        c,s=math.cos(pose[2]),math.sin(pose[2])
        to_world=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        previous_world=to_world @ self.previous
        d0=np.zeros(n*3);d0[:3]=previous_world
        g=-2*(self.s.T @ (self.q*np.tile(target,n))+self.d.T @ (self.rd*d0))
        polygon_scale=math.cos(math.pi/8)
        speed_cap=np.array([cfg.max_linear_speed*polygon_scale]*8+[cfg.max_angular_speed]*2)
        accel_cap=np.array([cfg.max_linear_accel*polygon_scale]*8+[cfg.max_angular_accel]*2)
        slew=np.tile(accel_cap*cfg.prediction_dt,n);slew[:10]=accel_cap*dt
        offset=np.zeros(n*10);offset[:10]=self.t @ previous_world
        upper=np.r_[np.tile(speed_cap,n),slew+offset]
        if self.plan is None:
            initial=np.tile(previous_world,n)
        else:
            initial=np.r_[self.plan[3:],self.plan[-3:]]
        result=minimize(lambda u: .5*u @ self.h @ u+g @ u,initial,
                        jac=lambda u:self.h @ u+g,method='SLSQP',
                        constraints=[LinearConstraint(self.constraints,-np.inf,upper)],
                        options={'maxiter':cfg.solver_max_iterations,'ftol':cfg.solver_ftol,'disp':False})
        elapsed=(time.perf_counter()-start)*1000
        violation=float(np.max(self.constraints @ result.x-upper)) if np.isfinite(result.x).all() else float('inf')
        self.diagnostics={'success':bool(result.success),'iterations':int(result.nit),'solve_ms':elapsed,
                          'status':str(result.message),'constraint_violation':violation if math.isfinite(violation) else None,
                          'horizon_sec':n*cfg.prediction_dt}
        if not result.success or not np.isfinite(result.x).all() or violation>1e-5:
            raise RuntimeError('MPC optimization failed: '+str(result.message))
        self.plan=result.x.copy()
        command=to_world.T @ result.x[:3]
        self.previous=command.tolist()
        self.reached=bool(linear_inside and angular_inside and np.linalg.norm(body_velocity[:2])<=cfg.settled_linear_speed and
                          abs(body_velocity[2])<=cfg.settled_angular_speed and np.linalg.norm(command[:2])<=cfg.settled_linear_speed and
                          abs(command[2])<=cfg.settled_angular_speed)
        return tuple(self.previous)
