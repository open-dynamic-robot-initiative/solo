import math
import time
import pinocchio
import numpy as np

from robot_wrapper import Robot

import blmc_robots
import robot_interfaces


# Initialize pinocchio model
robot = Robot(visualizer='gepetto',
                  viscous_friction=0.0)
robot.initViewer(loadModel=True)

robot_model = robot.model
robot_data = robot_model.createData()

# Initialize real robot
finger = blmc_robots.Robot(robot_interfaces.trifinger,
                           blmc_robots.create_trifinger_backend,
                           "trifinger.yml")
finger.initialize()

# End effector Ids
finger_tip_links = ["finger_tip_link_0", "finger_tip_link_120", "finger_tip_link_240"]
finger_tip_ids = [robot_model.getFrameId(name) for name in finger_tip_links]


def next_target(target_type, i=None, t=None, idx=None):
	# defining step targets
	if target_type == "step":
		z = [0.05, 0.2]
		if t % 1001 == 0:
			idx = (idx + 1) % 2
		return z[idx], idx

	# defining continuous, more smoother trajectory
	if target_type == "continuous":
		return (1 + math.sin(math.radians(i))) * 0.12 + 0.06, idx


def get_torques(J, x_obs, x_des, v_obs, v_des, Kp, Kd):
	'''
	Torque = J.T * [ kp*(x-x_des) + kd*(x`-x`_des) ]
	'''
	torque = J.T * ( Kp * (x_des - x_obs) + Kd * (v_des - v_obs) )
	torque = np.array(torque).reshape(-1)
	return torque

# Setting initial values
i = 0
idx = 0
last_t = 0
skip_count = 0
torque = np.array([0]*9)
target_type = "step"
target_type = "continuous"
while True:
	# Applying torque
	t = finger.frontend.append_desired_action(finger.Action(torque=torque))

	# Getting robot's joint state
	obs = finger.frontend.get_observation(t)
	q = obs.position
	w = obs.velocity

	# forward kinematics
	pinocchio.computeJointJacobians(robot_model, robot_data, q)
	pinocchio.framesForwardKinematics(robot_model, robot_data, q)

	# Computing the Jacobians
	J = [pinocchio.getFrameJacobian(robot_model, robot_data, tip_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3] for tip_id in finger_tip_ids]
	J = np.vstack(J)

	# Calculating x_observed (end effector's position)
	x_obs = [robot_data.oMf[tip_id].translation for tip_id in finger_tip_ids]
	x_obs = np.vstack(x_obs)

	# x desired (end effector's position)
	z, idx = next_target(target_type, i=i, t=t, idx=idx)
	x_des = [np.matrix([0,0.02,z]).T, np.matrix([0.02,-0.02,z]).T, np.matrix([-0.005,-0.01,z]).T]
	x_des = np.vstack(x_des)

	# v = J(theta)*w (v_final = 0) (end effector velocity)
	v_obs = J * np.matrix(w).T
	v_des = [np.matrix([0,0,0]).T, np.matrix([0,0,0]).T, np.matrix([0,0,0]).T]
	v_des = np.vstack(v_des)

	# Kp and Kd
	Kp = np.diag(np.full(9,100))
	Kd = np.diag(np.full(9,0))

	torque = get_torques(J, x_obs, x_des, v_obs, v_des, Kp, Kd)

	i += 0.1

	# Checking skip ratio
	if t > last_t + 1:
		skip_count = skip_count + 1.0

	# Printing every 5 sec
	if t % 5000 == 1:
		print('skip ratio', skip_count / t)
	last_t = t
