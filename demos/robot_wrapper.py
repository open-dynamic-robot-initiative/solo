# from pinocchio.robot_wrapper import RobotWrapper
# # from pinocchio.visualize import *
# from os.path import join, dirname, exists
#
# import sys
# import pinocchio
# import numpy as np
# import crocoddyl
# import time
#
#
# robot_properties_path = "/home/vagrawal/blmc_ei/workspace/src/catkin/robots/robot_properties/robot_properties_manipulator"
# finger_urdf_path = join(robot_properties_path,"urdf", "finger.urdf")
# meshes_path = dirname(robot_properties_path)
# print(finger_urdf_path, meshes_path)
# robot = RobotWrapper.BuildFromURDF(finger_urdf_path, [meshes_path])
# robot_model = robot.model
#
# target = np.array([0.4, 0., .4])
#
# robot.initViewer(loadModel=True)
# robot.viewer.gui.addSphere('world/point', .05, [1., 0., 0., 1.])  # radius = .1, RGBA=1001
# robot.viewer.gui.applyConfiguration('world/point', target.tolist() + [0., 0., 0., 1.])  # xyz+quaternion
# robot.viewer.gui.refresh()


from __future__ import print_function

import argparse
# import ipdb
import traceback
import sys


from collections import OrderedDict


import numpy as np
import numpy.matlib
import time
import os

import math

import matplotlib

# import matplotlib.pylab as plt

import pinocchio

from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import *
from os.path import join, dirname

from scipy.ndimage import gaussian_filter1d

# import rospkg
import crocoddyl
import time


def to_matrix(array):
    matrix = np.matrix(array)
    if matrix.shape[0] < matrix.shape[1]:
        matrix = matrix.transpose()

    return matrix


class Robot(RobotWrapper):
    def __init__(self,
                 symplectic=True,
                 init='cad',
                 visualizer=None,
                 viscous_friction=0.0,
                 static_friction=0.0
                 ):
        self.load_urdf()
        print("Control Dim:", self.control_dim)
        self.viscous_friction = to_matrix(np.zeros(self.control_dim)) + viscous_friction
        self.static_friction = to_matrix(np.zeros(self.control_dim)) + static_friction
        self.symplectic = symplectic
        if visualizer == "meshcat":
            self.setVisualizer(MeshcatVisualizer())
        elif visualizer == "gepetto":
            self.setVisualizer(GepettoVisualizer())
        elif visualizer:
            raise NotImplementedError

        # Initialization of Parameters
        if init == 'cad':
            pass
        elif init == 'random':
            self.set_random_params()
        elif init == 'noisy':
            self.set_noisy_params()
        elif init == 'identity':
            self.set_identity_params()
        else:
            raise NotImplementedError

    # dynamics -----------------------------------------------------------------
    def simulate(self,
                 dt,
                 n_steps=None,
                 torque=None,
                 initial_angle=None,
                 initial_velocity=None,
                 mask=None,
                 verbose=False):
        """ Returns the sequence of angles, velocities and torques resulting
            from simulating the given torques."""

        if not mask:
            mask = np.ones(self.control_dim)

        zero = pinocchio.utils.zero(self.model.nv)
        print(n_steps, torque.shape)

        torque = np.array(zero) if torque is None else np.array(torque)
        torque = torque.reshape(-1, self.control_dim, 1)
        if torque.shape[0] == 1:
            assert (n_steps)
            torque = np.repeat(torque, repeats=n_steps, axis=0)
        elif n_steps:
            assert (n_steps == torque.shape[0])

        angle = zero if initial_angle is None else to_matrix(initial_angle)
        velocity = \
            zero if initial_velocity is None else to_matrix(initial_velocity)
        mask = to_matrix(mask)

        simulated_angles = []
        simulated_vels = []
        simulated_accelerations = []
        applied_torques = []
        for t in range(torque.shape[0]):
            acceleration = self.forward_dynamics(angle, velocity, torque[t])

            simulated_angles.append(np.ravel(angle))
            simulated_vels.append(np.ravel(velocity))
            simulated_accelerations.append(np.ravel(acceleration))
            applied_torques.append(np.ravel(torque[t]))
            if self.symplectic:
                velocity = velocity + np.multiply(mask, acceleration * dt)
                angle = angle + np.multiply(mask, velocity * dt)
            else:
                angle = angle + np.multiply(mask, velocity * dt)
                velocity = velocity + np.multiply(mask, acceleration * dt)
            if verbose:
                print('angle: ', np.array(angle).flatten(),
                      '\nvelocity: ', np.array(velocity).flatten())
        return np.array(simulated_angles), np.array(simulated_vels), \
            np.array(simulated_accelerations), np.array(applied_torques)

    def predict(self, angle, velocity, torque, dt):
        angle = to_matrix(angle)
        velocity = to_matrix(velocity)
        torque = to_matrix(torque)
        acceleration = self.forward_dynamics(angle, velocity, torque)
        if self.symplectic:
            velocity = velocity + acceleration * dt
            angle = angle + velocity * dt
        else:
            angle = angle + velocity * dt
            velocity = velocity + acceleration * dt
        return angle, velocity

    def friction_torque(self, velocity):
        velocity = to_matrix(velocity)
        return -(np.multiply(velocity, self.viscous_friction) +
                 np.multiply(np.sign(velocity), self.static_friction))

    def forward_dynamics(self, angle, velocity, actuator_torque):
        joint_torque = actuator_torque + self.friction_torque(velocity)

        return pinocchio.aba(self.model, self.data, angle, velocity,
                             joint_torque)

    def inverse_dynamics(self, angle, velocity, acceleration):

        joint_torque = pinocchio.rnea(self.model, self.data,
                                      to_matrix(angle),
                                      to_matrix(velocity),
                                      to_matrix(acceleration))
        actuator_torque = joint_torque - self.friction_torque(velocity)

        # TODO: Figure out why this fails some times.
        # just as a sanity check -----------------------------------------------
        Y = self.compute_regressor_matrix(angle, velocity, acceleration)
        actuator_torque_1 = Y * self.get_params()
        assert ((abs(actuator_torque - actuator_torque_1) <= 1e-6).all())
        # ----------------------------------------------------------------------

        return actuator_torque

    def compute_regressor_matrix(self, angle, velocity, acceleration):
        joint_torque_regressor = \
            pinocchio.computeJointTorqueRegressor(self.model, self.data,
                                                  to_matrix(angle),
                                                  to_matrix(velocity),
                                                  to_matrix(acceleration))

        viscous_friction_torque_regressor = to_diagonal_matrix(velocity)
        static_friction_torque_regressor = to_diagonal_matrix(
            np.sign(velocity))

        regressor_matrix = np.concatenate([
            joint_torque_regressor,
            viscous_friction_torque_regressor,
            static_friction_torque_regressor], axis=1)

        return regressor_matrix

    def params_to_inertia_about_origin(self, params, link_index):
        if isinstance(params, np.ndarray):
            params = np.array(params).flatten()

        inertia_about_origin = \
            np.array([[params[10 * link_index + 4], params[10 * link_index + 5], params[10 * link_index + 7]],
                      [params[10 * link_index + 5], params[10 *
                                                           link_index + 6], params[10 * link_index + 8]],
                      [params[10 * link_index + 7], params[10 * link_index + 8], params[10 * link_index + 9]]])
        return inertia_about_origin

    # see Wensing et al 2018 for details
    def params_to_second_moment(self, params, link_index):
        inertia_about_origin = self.params_to_inertia_about_origin(
            params, link_index)

        second_moment = np.diag([0.5 * np.trace(inertia_about_origin)
                                 for _ in range(3)]) - inertia_about_origin

        return second_moment

    # see Wensing et al 2018 for details
    def params_to_pseudo_inertia(self, params, link_index):
        second_moment = self.params_to_second_moment(params, link_index)
        mass_times_com = self.params_to_mass_times_com(params, link_index)
        mass = self.params_to_mass(params, link_index)

        pseudo_inertia = np.empty(shape=[4, 4], dtype=second_moment.dtype)

        pseudo_inertia[:3, :3] = second_moment
        pseudo_inertia[3, :3] = mass_times_com
        pseudo_inertia[:3, 3] = mass_times_com
        pseudo_inertia[3, 3] = mass

        return pseudo_inertia

    def params_to_mass_times_com(self, params, link_index):
        if isinstance(params, np.ndarray):
            params = np.array(params).flatten()

        mass_times_com = np.array([params[10 * link_index + 1],
                                   params[10 * link_index + 2], params[10 * link_index + 3]])
        return mass_times_com

    def params_to_mass(self, params, link_index):
        if isinstance(params, np.ndarray):
            params = np.array(params).flatten()

        mass = params[10 * link_index]
        return mass

    def params_to_viscous_friction(self, params, link_index):
        if isinstance(params, np.ndarray):
            params = np.array(params).flatten()

        return params[10 * self.count_degrees_of_freedom() + link_index]

    def params_to_static_friction(self, params, link_index):
        if isinstance(params, np.ndarray):
            params = np.array(params).flatten()

        return params[11 * self.count_degrees_of_freedom() + link_index]

    def count_degrees_of_freedom(self):
        return self.nv

    # getters and setters ------------------------------------------------------

    def get_params(self):
        theta = [self.model.inertias[i].toDynamicParameters()
                 for i in range(1, len(self.model.inertias))]

        theta = theta + [self.viscous_friction, self.static_friction]

        theta = np.concatenate(theta, axis=0)

        # some sanity checks
        for i in range(len(self.model.inertias) - 1):
            A = self.params_to_inertia_about_origin(theta, i)
            B = self.get_inertia_about_origin(i)
            assert(np.allclose(A, B))

            A = self.params_to_mass_times_com(theta, i)
            B = self.get_mass_times_com(i)
            assert(np.allclose(A, B))

            A = self.params_to_mass(theta, i)
            B = self.get_mass(i)
            assert(np.allclose(A, B))

            A = self.params_to_viscous_friction(theta, i)
            B = self.get_viscous_friction(i)
            assert(np.allclose(A, B))

            A = self.params_to_static_friction(theta, i)
            B = self.get_static_friction(i)
            assert(np.allclose(A, B))

            A = self.params_to_second_moment(theta, i)
            B = self.get_second_moment(i)
            assert(np.allclose(A, B))

        return theta

    def get_com(self, link_index):
        return np.array(self.model.inertias[link_index + 1].lever).flatten()

    def get_mass(self, link_index):
        return self.model.inertias[link_index + 1].mass

    def get_mass_times_com(self, link_index):
        return self.get_mass(link_index) * self.get_com(link_index)

    def get_inertia_about_com(self, link_index):
        return np.array(self.model.inertias[link_index + 1].inertia)

    def get_inertia_about_origin(self, link_index):
        inertia_matrix_com = self.get_inertia_about_com(link_index)
        com = self.get_com(link_index)
        mass = self.get_mass(link_index)

        # parallel axis theorem
        inertia_matrix_origin = inertia_matrix_com + mass * \
            (np.inner(com, com)*np.identity(3) - np.outer(com, com))
        return inertia_matrix_origin

    def get_viscous_friction(self, link_index):
        return self.viscous_friction[link_index]

    def get_static_friction(self, link_index):
        return self.static_friction[link_index]

    def get_second_moment(self, link_index):
        inertia_about_com = self.get_inertia_about_com(link_index)
        mass = self.get_mass(link_index)
        com = self.get_com(link_index)

        second_moment = 0.5 * np.trace(inertia_about_com) * \
            np.identity(3) - \
            inertia_about_com + mass * np.outer(com, com)

        return second_moment

    def set_params(self, theta):

        for dof in range(self.model.nv):
            theta_dof = theta[dof * 10: (dof + 1) * 10]

            self.model.inertias[dof + 1] = pinocchio.libpinocchio_pywrap.Inertia.FromDynamicParameters(
                theta_dof)

        n_inertial_params = self.model.nv * 10
        self.viscous_friction = theta[n_inertial_params: n_inertial_params + 3]
        self.static_friction = theta[
            n_inertial_params + 3: n_inertial_params + 6]

        assert (((self.get_params() - theta) < 1e-9).all())

    def set_random_params(self):
        for dof in range(self.model.nv):
            self.model.inertias[dof + 1].setRandom()

    def set_identity_params(self):
        for dof in range(self.model.nv):
            self.model.inertias[dof + 1].setIdentity()

    def set_noisy_params(self):
        sigma = 0.001
        for dof in range(self.model.nv):
            self.model.inertias[dof + 1].mass += sigma * np.random.randn()
            self.model.inertias[dof + 1].lever += sigma * np.random.randn(3, 1)
            self.model.inertias[dof + 1].inertia += np.abs(np.diag(
                sigma * np.random.randn(3)))

    # loading ------------------------------------------------------------------
    def load_urdf(self):
        try:
            model_path = "/home/vagraval/blmc_ei/workspace/src/catkin/robots/robot_properties/robot_properties_manipulator"
        except rospkg.ResourceNotFound:
            print('Warning: The URDF is not being loaded from a ROS package.')
            current_path = str(os.path.dirname(os.path.abspath(__file__)))
            model_path = str(os.path.abspath(os.path.join(current_path,
                                                          '../../robot_properties_manipulator')))
        urdf_path = join(model_path, "urdf", "trifinger_with_stage.urdf")
        if "tri" in urdf_path:
            self.control_dim = 9
        else:
            self.control_dim = 3
        meshes_path = dirname(model_path)
        print(urdf_path, meshes_path)
        self.initFromURDF(urdf_path, [meshes_path])

    def test_regressor_matrix(self):
        for _ in range(100):
            angle = pinocchio.randomConfiguration(self.model)
            velocity = pinocchio.utils.rand(self.model.nv)
            acceleration = pinocchio.utils.rand(self.model.nv)

            Y = self.compute_regressor_matrix(angle, velocity, acceleration)
            theta = self.get_params()
            other_tau = Y * theta

            torque = self.inverse_dynamics(angle, velocity, acceleration)

            assert ((abs(torque - other_tau) <= 1e-9).all())


def some_demo():
    dt = 0.001
    n_steps = 5000

    robot = Robot(visualizer='gepetto',
                  viscous_friction=0.0)
    robot.initViewer(loadModel=True)

    torque = np.array([0., 0., 0.5]*3)

    torques = np.empty([n_steps, 9])
    for t in range(n_steps):
        if t % 100 == 0 and t > 0:
            torque = -torque
        torques[t] = torque


    positions, _, _, _ = robot.simulate(dt=dt,
                                        n_steps=n_steps,
                                        torque=torques,
                                        initial_velocity=[0] * 9,
                                        initial_angle=[0] * 9)
    # ipdb.set_trace()
    # print(positions)
    # print(type(positions),type(positions[0]))
    start_time = time.time()
    robot.play(positions.transpose(), 0.01)
    elapsed_time = time.time() - start_time

    print('elapsed time:', elapsed_time)



def croc():
    robot = Robot(visualizer='meshcat',
                  viscous_friction=0.0)
    robot_model = robot.model

    DT = 1e-3
    T = 5
    target = np.array([0.3, 0.3, .2])
    robot.initViewer(loadModel=True)
    # robot.viewer.gui.addSphere('world/point', .05, [1., 0., 0., 1.])  # radius = .1, RGBA=1001
    # robot.viewer.gui.applyConfiguration('world/point', target.tolist() + [0., 0., 0., 1.])  # xyz+quaternion
    # robot.viewer.gui.refresh()

    # Create the cost functions
    Mref = crocoddyl.FrameTranslation(robot_model.getFrameId("finger_tip_link"), np.matrix(target).T)
    state = crocoddyl.StateMultibody(robot.model)
    goalTrackingCost = crocoddyl.CostModelFrameTranslation(state, Mref)
    xRegCost = crocoddyl.CostModelState(state)
    uRegCost = crocoddyl.CostModelControl(state)

    # Create cost model per each action model
    runningCostModel = crocoddyl.CostModelSum(state)
    terminalCostModel = crocoddyl.CostModelSum(state)

    # Then let's added the running and terminal cost functions
    runningCostModel.addCost("gripperPose", goalTrackingCost, 1.)
    runningCostModel.addCost("stateReg", xRegCost, 1e-4) # 1e-4
    runningCostModel.addCost("ctrlReg", uRegCost, 1e-7) # 1e-7
    terminalCostModel.addCost("gripperPose", goalTrackingCost, 1000.)
    terminalCostModel.addCost("stateReg", xRegCost, 1e-4)
    terminalCostModel.addCost("ctrlReg", uRegCost, 1e-7)

    # Create the actuation model
    actuationModel = crocoddyl.ActuationModelFull(state)

    # Create the action model
    runningModel = crocoddyl.IntegratedActionModelEuler(
        crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuationModel, runningCostModel), DT)
    terminalModel = crocoddyl.IntegratedActionModelEuler(
        crocoddyl.DifferentialActionModelFreeFwdDynamics(state, actuationModel, terminalCostModel))

    # Create the problem
    q0 = np.matrix([0., 0., 0.]).T
    x0 = np.concatenate([q0, pinocchio.utils.zero(state.nv)])
    problem = crocoddyl.ShootingProblem(x0, [runningModel] * T, terminalModel)

    # Creating the DDP solver for this OC problem, defining a logger
    ddp = crocoddyl.SolverDDP(problem)
    ddp.setCallbacks([crocoddyl.CallbackLogger(),
                      crocoddyl.CallbackVerbose()])

    # Solving it with the DDP algorithm
    ddp.solve()

    # Plotting the solution and the DDP convergence
    # %matplotlib inline

    log = ddp.getCallbacks()[0]
    crocoddyl.plotOCSolution(log.xs, log.us)
    crocoddyl.plotConvergence(log.costs,log.control_regs,
                              log.state_regs,log.gm_stops,
                              log.th_stops,log.steps)

    # Visualizing the solution in gepetto-viewer
    # crocoddyl.displayTrajectory(robot, ddp.xs, runningModel.dt)
    # ddp.getCallbacks()[2](ddp)
    # crocoddyl.CallbackDisplay(robot)(ddp)
    positions = []
    for t,xi in enumerate(log.xs):
        positions.append(np.asarray(xi).reshape(-1)[:state.nq])
    #    print(positions[:-1], type(positions[-1]))
        #robot.display(xi[:robot.nq, None])
        time.sleep(1.e-2)
    print(positions)
    print(log.us)
    input()
    robot.play(np.array(positions).transpose(), 1)

    positions = []
    for t,xi in enumerate(ddp.xs):
        positions.append(np.asarray(xi).reshape(-1)[:state.nq])
 #       print(xi, type(xi))
  #      #robot.display(xi[:robot.nq, None])
   #     time.sleep(1.e-2)
    print(positions)
    print(ddp.us)
    input()
    robot.play(np.array(positions).transpose(), 1)

    robot_data = robot_model.createData()
    xT = ddp.xs[-1]
    pinocchio.forwardKinematics(robot_model, robot_data, xT[:state.nq])
    pinocchio.updateFramePlacements(robot_model, robot_data)
    print('Finally reached = ', robot_data.oMf[robot_model.getFrameId("finger_tip_link")].translation.T)





if __name__ == '__main__':
    try:
        some_demo()
        # croc()
        input()
    except:
        extype, value, tb = sys.exc_info()
        # traceback.print_exc()
        # ipdb.post_mortem(tb)
