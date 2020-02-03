""" implements a very basic simulator, fully actuated and no contact ! """

import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import os, sys, time   
import matplotlib.pyplot as plt 




class CubeSimulator(object):
    def __init__(self, 
                urdf_path="/home/vagraval/blmc_ei/workspace/src/catkin/simulators/pybullet_fingers/test/basic_simulator/model/urdf/free_flyer.urdf", 
                mesh_path="/home/vagraval/blmc_ei/workspace/src/catkin/simulators/pybullet_fingers/test/basic_simulator/model/", 
                dt=1.e-3, 
                int_steps=1):
        self.urdf_path = urdf_path
        self.mesh_path = mesh_path
        self.robot = RobotWrapper.BuildFromURDF(
        urdf_path, [mesh_path], pin.JointModelFreeFlyer())
        self.rmodel = self.robot.model 
        self.rdata = self.rmodel.createData()
        self.dt = dt  # simulation step 
        self.int_steps = int_steps # itnegration steps 
        self.nq = self.rmodel.nq # position dimension 
        self.nv = self.rmodel.nv # velocity dimension 
        self.n = self.nq + self.nv # state vector dimesion 
        self.ndx = 2 * self.nv  # tangent vector dimension 
        self.m = self.nv # control dimension
        self.sub_dt = self.dt/self.int_steps #  integration dt 
        # 
        self.state = np.zeros([self.n,1])


    def reset_state(self, x):
        assert x.shape == (self.n,1), BaseException('state vector has wrong dimension')
        self.state[:,:] = x.copy() 
        

    def increment(self, x, dx):
        """ increments state vector by a tangent vector dx """
        assert x.shape == (self.n,1), BaseException('state vector has wrong dimension')
        assert dx.shape == (self.ndx,1), BaseException('tangent vector  has wrong dimension')
        xnext = np.zeros([self.n,1])
        xnext[:self.nq] = pin.integrate(self.rmodel, x[:self.nq], dx[:self.nv])
        xnext[self.nq:] = x[self.nq:] + dx[self.nv:]
        return xnext.copy() 

    def diff(self, x1, x2):
        """ returns x2-x1 """
        assert x1.shape == (self.n,1), BaseException('state vector 1 has wrong dimension')
        assert x2.shape == (self.n,1), BaseException('state vector 2 has wrong dimension')
        dx = np.zeros([self.ndx,1])
        dx[:self.nv] = pin.difference(self.rmodel, x1[:self.nq], x2[:self.nq])
        dx[self.nv:] = x2[self.nq:] - x1[self.nq:]
        return dx 


    def step(self, u):
        """ apply control u for one integration step """
        assert u.shape == (self.m,1), BaseException('control vector has wrong dimension')
        dx = np.zeros([self.ndx,1])
        x = self.state.copy()
        for _ in range(self.int_steps):
            pin.aba(self.rmodel, self.rdata, x[:self.nq],x[self.nq:],u) 
            dx[:self.nv] = self.sub_dt*(x[self.nq:] + .5 * self.sub_dt * self.rdata.ddq)
            dx[self.nv:] = self.sub_dt*self.rdata.ddq 
            x[:] = self.increment(x,dx)
        self.state = x.copy()
        return x 

    def random_state(self):
        """ returns a random state for the robot defined in the simulator """
        pos = pin.se3ToXYZQUAT(pin.SE3.Random())
        vel = np.random.rand(self.nv,1)
        return np.vstack([pos,vel])

        
            
    







if __name__=='__main__':
    urdf_path = os.path.abspath('model/urdf/free_flyer.urdf')
    mesh_path = os.path.abspath('model/')
    sim = CubeSimulator(urdf_path, mesh_path, dt=1.e-2, int_steps=10)
    print('position states dimension ', sim.nq)
    print('velocity states dimension ', sim.nv)
    print('control vector dimension ', sim.m)

    # create two random states 
    x1 = sim.random_state().copy()
    x2 = sim.random_state().copy()
    # compute their difference (both position and velocity stacked together)
    tangent_vect = sim.diff(x1,x2)
    # add/increment the tangent vector back 
    x2_new = sim.increment(x1, tangent_vect)
    print(x2-x2_new)
    print(np.linalg.norm(x2 - x2_new))
    # check if the results match 
    # assert  np.linalg.norm(x2 - x2_new) <= 1.e-10, BaseException("diff and increment don't match") 
    
    x_init = np.zeros([sim.n,1])
    x_init[2] = 1. # height is set to 1 
    x_init[6] = 1. # quaternion is now [0.,0.,0.,1.]
    # all velocities are zero --> free fall simulation 

    sim.reset_state(x_init)
    # simulation horizon 2 secs at 1.e-2 dt 
    N = 2000
    xt = np.zeros([N+1, sim.n, 1])
    xt[0] = x_init.copy() 

    u_0 = np.zeros([sim.m,1])
    # u_0[2] = 9.81

    # simulate for 2 seconds with no control (dt is 1.e-2)
    for t in range(N):
        xt[t+1] = sim.step(u_0).copy() 

    # simple check 
    # xf = x0 + dt* v0 + .5 * dt^2 * a_0 (for free fall case)
    print('error in final height',  xt[-1,2,0] - (1. -.5*9.81*(2.**2)))
    print('error in final vertical velocity ', xt[-1,sim.nq+2,0] - (-2.*9.81))

    # we can also display it in gepetto viewer 
    try:
        sim.robot.initViewer(loadModel=True)
    except:
        raise BaseException('Gepetto viewer not initialized ! ')


    sim.robot.display(xt[0,:sim.nq])

    for t in range(N+1):
        sim.robot.display(xt[t,:sim.nq])
        time.sleep(1.e-2)

    
    # plot the height as function of time (should look like -x^2 for a free fall )

    plt.figure()
    plt.plot(1.e-2*np.arange(N+1), xt[:,2, 0])
    plt.show() 







