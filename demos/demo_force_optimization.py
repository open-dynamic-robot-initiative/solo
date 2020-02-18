import math
import time
import pinocchio
import numpy as np

from robot_wrapper import Robot

import blmc_robots
import robot_interfaces

import matplotlib.pyplot as plt
from pybullet_fingers import sim_finger

import ipdb
from basic_simulator.basic_simulator import CubeSimulator
from quadprog import solve_qp


from object_tracker import ObjectTracker
import transformations as tf
from py_blmc_kinect_camera import KinectCamera

import pickle


class Robot_Control:

	def __init__(self, mode="real", block_info_from="real"):
		'''
		Intializes the robot to apply controls
		'''
		self.mode = mode
		self.block_info_from = block_info_from
		self.initialize_robot()
		self.reset_state()
		


	def initialize_robot(self):
		'''
		Loads the URDF in pinnochio and initializes the real robot
		'''
		# Initialize Robot model in pinocchio
		self.robot = Robot(visualizer='gepetto', viscous_friction=0.0)
		self.robot.initViewer(loadModel=True)

		self.robot_model = self.robot.model
		self.robot_data = self.robot_model.createData()


		# Initialize real robot
		if self.mode == "real":
			self.finger = blmc_robots.Robot(robot_interfaces.trifinger,
			                           blmc_robots.create_trifinger_backend,
			                           "trifinger.yml")
			self.finger.initialize()

		elif self.mode == "simulation" or self.mode == "pinocchio":
			self.finger = sim_finger.SimFinger(finger_type="trifinger", time_step=1.e-3, enable_visualization=True, action_bounds=None)

		# End effector Ids
		self.finger_tip_links = ["finger_tip_link_0", "finger_tip_link_120", "finger_tip_link_240"]
		self.finger_tip_ids = [self.robot_model.getFrameId(name) for name in self.finger_tip_links]



	def reset_state(self):
		'''Resets the state of the variables. 
		This is done to prepare the robot for the next task
		'''
		self.torque = np.array([0,0,0.]*3)
		self.is_grabbing = False
		self.block_com_position = None # initially unknown
		self.object_size = None # initially unknown
		self.landing_position = [0,-0.12,0.03]
		self.t = 0
		self.last_time_step = 0
		self.global_time_controller = 0


	



	def add_new_object(self, size, position, orientation=[0,0,0,1]):
		'''
		Adds a new object information to the system
		'''
		self.object_size = size
		self.block_com_position = position
		self.object_orientation = orientation

		if self.mode == "simulation": # adding object in pybullet
			self.finger.import_interaction_objects(size, position, orientation)
			self.cube = CubeSimulator(dt=1.e-3, int_steps=1)
			self.dt = 1.e-3
			# Initial State
			x_init = np.zeros([self.cube.n,1])
			x_init[:3] = np.resize(position,(3,1))
			x_init[3:7] = np.resize(orientation,(4,1)) # quaternion
			self.cube.reset_state(x_init)

			# we can also display it in gepetto viewer
			try:
				self.cube.robot.initViewer(loadModel=True)
			except:
				raise BaseException('Gepetto viewer not initialized ! ')

			self.cube.robot.display(x_init[:self.cube.nq])


		if self.mode == "real":
			self.cube = CubeSimulator(dt=1.e-3, int_steps=1)
			self.dt = 1.e-3

			# Initial State
			x_init = np.zeros([self.cube.n,1])
			x_init[:3] = np.resize(position,(3,1))
			x_init[3:7] = np.resize(orientation,(4,1)) # quaternion
			self.cube.reset_state(x_init)

			# Add tracking from depth camera
			# initial_object_in_base_link = tf.compose_matrix(angles=(0, 0, 0),
			#                                         translate=(0, 0, 0.05))
			# self.object_tracker = ObjectTracker()
			# self.object_tracker.initialize(initial_object_in_base_link)

		self.cube_q_last = self.get_cube_state()
		self.cube_q_next = self.get_cube_state()



	def get_cube_state(self):
		'''Returns the cube's current state (position and orientation --> 1x7) depending upon the requested source
		'''
		if self.block_info_from == "simulation":
			return self.finger.get_block_state()

		if self.block_info_from == "real":
			return self.box_state_from_tip_position()

		if self.block_info_from == "pinocchio":
			return self.cube.state.copy()[:7]


	def box_state_from_tip_position(self):
		'''Use the contact point location on the block to get the cube's com position
		'''
		end_effector_position = [self.robot_data.oMf[tip_id].translation for tip_id in self.finger_tip_ids]
		block_com_position = [(end_effector_position[0][i] +
								end_effector_position[1][i] +
								end_effector_position[2][i])/3.0 for i in range(3)]


		block_com_position = np.resize(block_com_position, (3,1))

		unit_vector_along_x = np.resize((end_effector_position[1] - end_effector_position[2]),3)
		unit_vector_along_x = unit_vector_along_x / max(np.linalg.norm(unit_vector_along_x), 1.e-9)

		a = np.resize((end_effector_position[2] - end_effector_position[0]),3)
		a = a /  max(np.linalg.norm(a), 1.e-9)

		b = np.resize((end_effector_position[1] - end_effector_position[0]),3)
		b = b / max(np.linalg.norm(b), 1.e-9)

		unit_vector_along_z = np.cross(a,b)
		unit_vector_along_z = unit_vector_along_z / max(np.linalg.norm(unit_vector_along_z), 1.e-9)

		unit_vector_along_y = np.cross(unit_vector_along_z, unit_vector_along_x)
		unit_vector_along_y = unit_vector_along_y / max(np.linalg.norm(unit_vector_along_y), 1.e-9)

		self.rotation_matrix = np.matrix([unit_vector_along_x, unit_vector_along_y, unit_vector_along_z]).T
		self.quat = pinocchio.Quaternion(self.rotation_matrix).coeffs()

		# Setting the position offset
		offset =  np.array([0,-0.01, 0]).reshape(-1,1)

		block_com_position = block_com_position + self.rotation_matrix.dot(offset)

		return np.concatenate([block_com_position, self.quat])




	def calculate_q_compensation(self):
		obs = self.obs
		q = obs.position.copy()
		tau = obs.torque

		if not self.is_grabbing:
			return q


		for i, (rad, torque) in enumerate(zip(q,tau)):
			if torque > 0.03:
				q[i] += math.radians(-1 * torque - 0.6)
			elif torque < -0.1:
				q[i] += math.radians(-1 * torque + 0.7)
			# else:
			# 	q[i] += math.radians(-11 * (torque - 0.03) - 0.63)

		return q



	def finger_tip_observed_state(self): 
		'''
		Apply the torque values to get to a new state of the robot.
		Also calculates the new Jacobians and the new end-effector position by doing Forward Kinematics.
		'''

		# Applying torque on real robot
		if self.mode == "real":
			time_stamp = self.finger.frontend.append_desired_action(self.finger.Action(torque=self.torque))
			self.time_stamp = time_stamp

			# Getting robot's joint state
			obs = self.finger.frontend.get_observation(time_stamp)
			self.obs = obs
			q = obs.position
			w = obs.velocity
			q = self.calculate_q_compensation() # for backlash and motorbelt timing error


		else:
			time_stamp = self.finger.append_desired_action(self.finger.Action(torque=self.torque))
			self.time_stamp = time_stamp

			# Getting robot's joint state
			obs = self.finger.get_observation(time_stamp)
			self.obs = obs
			q = np.array(obs.position)
			w = np.array(obs.velocity)

		# forward kinematics
		pinocchio.computeJointJacobians(self.robot_model, self.robot_data, q)
		pinocchio.framesForwardKinematics(self.robot_model, self.robot_data, q)

		# Computing the Jacobians
		J = [pinocchio.getFrameJacobian(self.robot_model, self.robot_data, tip_id,
										pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3] for tip_id in self.finger_tip_ids]
		self.J = np.vstack(J)

		tau = np.matrix(obs.torque).T
		try:
			J_inv = np.linalg.inv(self.J.T)
		except:
			J_inv = np.linalg.pinv(self.J.T)
		self.F_observed = J_inv * tau

		# Calculating x_observed (end effector's position)
		self.x_obs = [self.robot_data.oMf[tip_id].translation - self.block_com_position for tip_id in self.finger_tip_ids]

		if self.block_info_from == "pinocchio":
			self.x_obs = [np.array([[-0.00334082],[ 0.04888184],[-0.01672634]]),
						  np.array([[ 0.03910008],[ 0.00379204],[-0.01891771]]),
						  np.array([[-0.03915938],[-0.00046447],[-0.01780892]])]


		# x` = J * q` (x`_des = 0 --> end effector desired velocity = 0)
		self.v_obs = self.J * np.matrix(w).T



	def move(self, desired_position, offset=None, timeout=5, exit_before_timeout=True):
		'''
		Moves the finger tips to the desired final position.
		An offset is passed, so that the fingers don't disturb the object while reaching it.
		The forces are not optimised in a simple "move" task.
		'''
		if offset is not None:
			self.object_size += offset

		# Initial Values
		i = 0
		obj_start_pos = self.block_com_position

		while True:
			# Slowly changing (indirectly) the xyz-position for the end_effector by changing the position of the block
			x = (1 - math.cos(math.radians(i/10))) * (desired_position[0] - obj_start_pos[0])/2 + obj_start_pos[0]
			y = (1 - math.cos(math.radians(i/10))) * (desired_position[1] - obj_start_pos[1])/2 + obj_start_pos[1]
			z = (1 - math.cos(math.radians(i/10))) * (desired_position[2] - obj_start_pos[2])/2 + obj_start_pos[2]

			self.block_com_position = [x,y,z]
			self.quat = [0,0,0,1]
			self.cube_q_next = np.concatenate([self.block_com_position, self.quat, [0]*6])
			self.rotation_matrix = pinocchio.XYZQUATToSe3(self.cube_q_next[:7]).rotation

			# Calculating desired_external_force to apply at the contact points (to grasp the object)
			self.F = np.matrix(np.zeros(9)).T

			# Get robot's (finger tip's) observed state
			self.finger_tip_observed_state()

			# Calculates the final forces to apply at the end effectors and maps them to the finger motor torques
			self.finger_impedance_control(unoptimised_forces=True)

			# Setting timeout for "timeout seconds"
			i += 1
			if i % (timeout*1000) == 0:
				print("****************** Done *******************")
				break

			# The final position should be within a certain threshold
			if exit_before_timeout and all(np.abs(np.subtract(self.block_com_position, desired_position)) < 0.005):
				break

		# Undoing the offset which was initially set
		if offset is not None:
			self.object_size -= offset



	def grasp(self, f_applied=[1.1,1.1,0]):
		'''
		This signals that the fingers are now grasping the object.
		Hence, the external forces (optimised) to hold the object begin to be applied.
		For this, we assume that the fingers have already reached the object boundary.
		'''
		self.f_applied = f_applied
		self.cnt_array = [1,1,1]
		self.move(self.block_com_position, timeout=1, exit_before_timeout=False)
		self.is_grabbing = True


	def release(self):
		'''
		This signals that the fingers have now released the object.
		Hence, no external force should now be applied at the end effectors
		'''
		self.is_grabbing = False
		self.cnt_array = [0,0,0]
		


	def follow_predefined_trajectory(self):
		assert self.block_com_position is not None
		assert self.object_size is not None
		initial_position = self.block_com_position
		self.block_com_position = [0,0,0]
		self.move([0,0,0.25], offset=0.04, timeout=2, exit_before_timeout=False)
		if self.mode == "simulation":
			self.finger.set_block_state(initial_position, [0,0,0,1])
		self.move(initial_position, offset=0.04, timeout=2, exit_before_timeout=False) # reaching the object
		self.grasp()
		self.move(self.block_com_position[:2] + [0.21]) # picking up
		self.move(self.landing_position[:2] + [self.block_com_position[2]]) # moving forward
		self.move(self.landing_position) # moving down
		self.release()
		self.move(self.landing_position[:2] + [0.25], offset=0.02) # moving away from the object after placing it
		self.move([0,0,0.25], offset=0.02) # moving to the centre (just using it for a better ending, for now)


	def move_in_a_circle(self):
		alpha = math.radians(self.t/1000 * 60)
		r = 0.08
		x = r * math.cos(alpha)
		y = r * math.sin(alpha)

		return np.resize([x,y,0.05], (3,1))



	def next_state(self):
		'''Numerically computes the next state of the block (using interpolation between an initial and a final state)
		'''

		# the block is supposed to reach a final position within 1 second after which the final position changes
		seconds = 1
		if (self.t*self.dt) % seconds == 0:
			self.q1 = self.cube_q_last
			self.q2 = pinocchio.se3ToXYZQUAT(pinocchio.SE3.Random())
			self.q2[:3] = np.resize([0,0,0.17], (3,1))
			self.q2[3:] = np.resize([0,0,0,1], (4,1))

			# self.q2[:3] = self.move_in_a_circle()

			# if self.t == seconds * 1000:
			# 	self.q2 = self.cube_q_last


			self.last_time_step = self.t

			final_interpolation = pinocchio.interpolate(
						self.cube.rmodel,
						self.cube_q_last,
						self.q2,
						1)

			assert np.linalg.norm(self.q2-final_interpolation) <= 1.e-10

		# interpolating an intermediate state for every "1" millisecs ahead from the present state
		steps = 1
		if self.t % steps == 0:

			self.intermediate_state = pinocchio.interpolate(
							self.cube.rmodel,
							self.q1,
							self.q2,
							(steps/(1000*seconds))*(self.t - self.last_time_step))


			self.intermediate_velocity = pinocchio.difference(
							self.cube.rmodel,
							self.cube_q_next[:7],
							self.intermediate_state) / (steps/1000)

			self.cube_q_next = np.vstack([self.intermediate_state, self.intermediate_velocity])

		return np.vstack([self.intermediate_state, self.intermediate_velocity]) # 13x1


	def present_state(self):
		'''Numerically computes the present state of the block.
		'''
		steps = 1
		if self.t % steps == 0:
			prev_state = self.cube_q_last
			present_state = self.get_cube_state()
			self.cube_q_last = present_state

			present_velocity = pinocchio.difference(
								self.cube.rmodel,
								prev_state,
								present_state) / (steps/1000)

			self.state = np.vstack([present_state, present_velocity]) #  13x1

		return self.state


	def compute_block_wrench(self):
		'''
		Computes the block's wrench (i.e. the com linear forces and the andular torques) for each point along the trajectory.
		'''
		present_state = self.present_state() # 13x1
		next_state = self.next_state() # 13x1
		self.dx = self.cube.diff(present_state, next_state) # 12x1
		self.rotation_matrix = pinocchio.XYZQUATToSe3(self.cube_q_last).rotation # local to world

		# this is to account for the discrepancy between the actual control frequency and applied control frequency
		if self.global_time_controller % (self.dt*1000) == 0:
			self.t += 1

		self.global_time_controller += 1

		# feedback gains for the linear forces
		self._kc = np.matrix(np.diag(np.full(3,100))) # this already has mass multiplied
		self._dc = np.matrix(np.diag(np.full(3, .001))) # this already has mass multiplied 0.005

		# feedback gains for the angular torques
		self._kb = np.matrix(np.diag(np.full(3,.01))) # this already has mass multiplied
		self._db = np.matrix(np.diag(np.full(3,.0000))) # this already has mass multiplied

		# This is computed in the block's local frame
		self.block_com_wrench = np.vstack([
            self.rotation_matrix.T.dot(np.array([0,0,0.8]).reshape(-1,1)) + self._kc * self.dx[:3] + self._dc *self.dx[6:9],
            self._kb * self.dx[3:6] + self._db * self.dx[9:]
        	])

		self.block_com_position = present_state[:3]




	def quadprog_solve_qp(self, P, q, G=None, h=None, A=None, b=None, initvals=None):
		"""
		Solve a Quadratic Program defined as:
		    minimize
		        (1/2) * x.T * P * x + q.T * x
		    subject to
		        G * x <= h
		        A * x == b
		using quadprog <https://pypi.python.org/pypi/quadprog/>.
		Parameters
		----------
		P : numpy.array
		    Symmetric quadratic-cost matrix.
		q : numpy.array
		    Quadratic-cost vector.
		G : numpy.array
		    Linear inequality constraint matrix.
		h : numpy.array
		    Linear inequality constraint vector.
		A : numpy.array, optional
		    Linear equality constraint matrix.
		b : numpy.array, optional
		    Linear equality constraint vector.
		initvals : numpy.array, optional
		    Warm-start guess vector (not used).
		Returns
		-------
		x : numpy.array
		    Solution to the QP, if found, otherwise ``None``.
		Note
		----
		The quadprog solver only considers the lower entries of `P`, therefore it
		will use a wrong cost function if a non-symmetric matrix is provided.
		"""
		if initvals is not None:
		    print("quadprog: note that warm-start values ignored by wrapper")

		b = np.resize(b,6)

		qp_G = P
		qp_a = -q
		if A is not None:
		    qp_C = np.vstack([A, G]).T
		    qp_b = np.hstack([b, h])
		    meq = A.shape[0]
		else:  # no equality constraint
		    qp_C = G.T
		    qp_b = h
		    meq = 0
		return solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]



	def compute_force_qp(self):
		'''
		Optimize the end-effector forces subject to some linear and inequality constraints.
		'''

		self.finger_tip_observed_state()

		# Use the contact activation from the plan to determine which of the forces
		# should be active.
		N = (int)(np.sum(self.cnt_array))
		self._mu = 0.4

		# Setup the QP problem.
		Q = 2. * np.eye(3 * N + 6)
		Q[-6:,-6:] = 1e6 * np.eye(6)
		p = np.zeros(3 * N + 6)
		A = np.zeros((6, 3 * N + 6))
		b = self.block_com_wrench

		G = np.zeros((5 * N, 3 * N + 6))
		h = np.zeros((5 * N))

		j = 0
		for i in range(3):

		    if self.cnt_array[i] == 0:
		        continue

		    A[:3, 3 * j:3 * (j + 1)] = np.eye(3)
		    A[3:, 3 * j:3 * (j + 1)] = pinocchio.utils.skew(self.x_obs[i])


		    reduced_value = 1.47 # root 2

		    if i == 0:
		    	G[3*j + 0, 3 * j + 1] = -1      # -Fy >= 0
		    	G[3*j + 1, 3 * j + 0] = -reduced_value		# |mu Fy| - Fx >= 0
		    	G[3*j + 1, 3 * j + 1] = -self._mu
		    	G[4*j + 1, 3 * j + 0] = reduced_value		# |mu Fy| + Fx >= 0
		    	G[4*j + 1, 3 * j + 1] = -self._mu
		    	G[3*j + 2, 3 * j + 2] = -reduced_value		# |mu Fy| - Fz >= 0
		    	G[3*j + 2, 3 * j + 1] = -self._mu
		    	G[5*j + 2, 3 * j + 2] = reduced_value		# |mu Fy| + Fz >= 0
		    	G[5*j + 2, 3 * j + 1] = -self._mu


		    if i == 1:
		    	G[3*j + 0, 3 * j + 0] = -1      # Fx <= 0
		    	G[3*j + 1, 3 * j + 1] = -reduced_value		# |mu Fx| - Fy >= 0
		    	G[3*j + 1, 3 * j + 0] = -self._mu
		    	G[4*j + 1, 3 * j + 1] = reduced_value		# |mu Fx| + Fy >= 0
		    	G[4*j + 1, 3 * j + 0] = -self._mu
		    	G[3*j + 2, 3 * j + 2] = -reduced_value		# |mu Fx| - Fz >= 0
		    	G[3*j + 2, 3 * j + 0] = -self._mu
		    	G[5*j + 2, 3 * j + 2] = reduced_value		# |mu Fx| + Fz >= 0
		    	G[5*j + 2, 3 * j + 0] = -self._mu

		    if i == 2:
		    	G[3*j + 0, 3 * j + 0] = 1       # Fx >= 0
		    	G[3*j + 1, 3 * j + 1] = -reduced_value		# |mu Fx| - Fy >= 0
		    	G[3*j + 1, 3 * j + 0] = self._mu
		    	G[4*j + 1, 3 * j + 1] = reduced_value		# |mu Fx| + Fy >= 0
		    	G[4*j + 1, 3 * j + 0] = self._mu
		    	G[3*j + 2, 3 * j + 2] = -reduced_value		# |mu Fx| - Fz >= 0
		    	G[3*j + 2, 3 * j + 0] = self._mu
		    	G[5*j + 2, 3 * j + 2] = reduced_value		# |mu Fx| + Fz >= 0
		    	G[5*j + 2, 3 * j + 0] = self._mu


		    j += 1

		A[:, -6:] = np.eye(6)

		solx = self.quadprog_solve_qp(Q, p, G=G, h=h, A=A, b=b)

		F = np.zeros(9)
		j = 0
		for i in range(3):
		    if self.cnt_array[i] == 0:
		        continue
		    F[3*i: 3*(i + 1)] = self.rotation_matrix.dot(solx[3*j: 3*(j + 1)])
		    j += 1

		F = F.reshape(-1,1)


		F_observed_local = np.zeros([9]).reshape(-1,1)

		for i in range(3):
			F_observed_local[3*i: 3*(i + 1)] = self.rotation_matrix.T.dot(self.F_observed[3*i: 3*(i + 1)])

		self.observed_wrench = np.matrix(A[:,:-6]) * F_observed_local

		self.F = F # external forces in world frame
		self.finger_impedance_control()



	def finger_impedance_control(self, unoptimised_forces=False):
		'''
		Computes a relation between end-effector external force, position and velocity.
		This final force is mapped to the torque values using end-effector Jacobian transpose
		'''
		object_state = np.resize(self.cube_q_next, 13)
		self.end_eff_des_pos = [np.matrix([object_state[0], object_state[1], object_state[2]]).T,
				 np.matrix([object_state[0], object_state[1], object_state[2]]).T,
				 np.matrix([object_state[0], object_state[1], object_state[2]]).T]

		self.end_eff_des_pos = np.vstack(self.end_eff_des_pos)

		contact_point_offset = [np.array([0, self.object_size/2, 0]).reshape(-1,1),
						np.array([self.object_size/2, -self.object_size/8, 0]).reshape(-1,1),
						np.array([-self.object_size/2, -self.object_size/8, 0]).reshape(-1,1)]

		for i in range(3):
			self.end_eff_des_pos[3*i: 3*(i + 1)] += self.rotation_matrix.dot(contact_point_offset[i])

		self.end_eff_des_vel = np.array([object_state[7:10]] * 3).reshape(-1,1)

		# Finger 0
		self.end_eff_des_vel[0] += - np.linalg.norm(self.x_obs[0]) * object_state[12]
		self.end_eff_des_vel[1] += 0
		self.end_eff_des_vel[2] += np.linalg.norm(self.x_obs[0]) * object_state[10]

		# Finger 1
		self.end_eff_des_vel[3] += 0
		self.end_eff_des_vel[4] += np.linalg.norm(self.x_obs[1]) * object_state[12]
		self.end_eff_des_vel[5] += - np.linalg.norm(self.x_obs[1]) * object_state[11]

		# Finger 2
		self.end_eff_des_vel[6] += 0
		self.end_eff_des_vel[7] += - np.linalg.norm(self.x_obs[2]) * object_state[12]
		self.end_eff_des_vel[8] += np.linalg.norm(self.x_obs[2]) * object_state[11]

		for i in range(3):
			self.end_eff_des_vel[3*i: 3*(i + 1)] = self.rotation_matrix.dot(self.end_eff_des_vel[3*i: 3*(i + 1)])

		self.end_eff_obs_pos = [self.robot_data.oMf[tip_id].translation for tip_id in self.finger_tip_ids]
		self.end_eff_obs_pos = np.vstack(self.end_eff_obs_pos)
		self.end_eff_obs_vel = self.J * np.matrix(self.obs.velocity).T

		# Kp and Kd
		if unoptimised_forces:
			self.Kp = np.diag(np.full(9,81)) # np.diag(np.full(9,81))
			self.Kd = np.diag(np.full(9,0.11)) # np.diag(np.full(9,0.09))
		else:
			self.Kp = np.diag(np.full(9,63)) # np.diag(np.full(9,81)) 30 -- 53
			self.Kd = np.diag(np.full(9,0.001)) # np.diag(np.full(9,0.09)) 0.001

		force = self.Kp * (self.end_eff_des_pos - self.end_eff_obs_pos) + self.Kd * (self.end_eff_des_vel - self.end_eff_obs_vel)
		total_force = self.F + force

		torque = self.J.T * total_force
		self.torque = np.array(torque).reshape(-1)



	def apply_controls(self):
		'''Sequentially calls the funtion to calculate block's wrench at the com and then the optimised finger forces.
		'''
		self.compute_block_wrench()
		self.compute_force_qp()
		u = self.cube.step(self.block_com_wrench).copy()



	def optimised_trajectory(self):
		'''Divides the task of reaching the object, grasping it and then making it follow a desired trajectory.
		The end-effector forces are optimized while doing so.
		'''
		if self.mode == "simulation":
			self.finger.set_block_state([0,0,0.05], [0,0,0,1])

		self.block_com_position[2] = 0.15
		self.move([0,0,0.03], offset=0.03, timeout=3, exit_before_timeout=True) # reaching the object
		self.grasp()

		self.t = 0
		self.global_time_controller = 0


		self.cube_q_last = self.get_cube_state()
		self.cube_q_next = self.get_cube_state()

		# simulation horizon N*dt seconds. 
		N = 20000


		optimised_forces = np.zeros([N,9])
		observed_forces = np.zeros([N,9])

		desired_contact_points = np.zeros([N,9])
		observed_contact_points = np.zeros([N,9])

		desired_contact_velocities = np.zeros([N,9])
		observed_contact_velocities = np.zeros([N,9])

		desired_wrench = np.zeros([N,6])
		observed_wrench = np.zeros([N,6])

		desired_block_position = np.zeros([N,7])
		observed_block_position = np.zeros([N,7])

		differences = np.zeros([N,12])

		for t in range(N):
			self.apply_controls()

			desired_block_position[t,:] = np.resize(self.cube_q_next, 7)
			observed_block_position[t,:] = np.resize(self.cube_q_last, 7)

			differences[t,:] = np.resize(self.dx, 12)

			optimised_forces[t,:] = np.resize(self.F,9)
			observed_forces[t,:] = np.resize(self.F_observed,9)

			desired_contact_points[t,:] = np.resize(self.end_eff_des_pos, 9)
			observed_contact_points[t,:] = np.resize(self.end_eff_obs_pos, 9)

			desired_contact_velocities[t,:] = np.resize(self.end_eff_des_vel, 9)
			observed_contact_velocities[t,:] = np.resize(self.end_eff_obs_vel, 9)

			desired_wrench[t,:] = np.resize(self.block_com_wrench, 6)
			observed_wrench[t,:] = np.resize(self.observed_wrench, 6)


		# Dumping in a file
		data = {}

		data["optimised_forces"] = optimised_forces
		data["observed_forces"] = observed_forces

		data["desired_contact_points"] = desired_contact_points
		data["observed_contact_points"] = observed_contact_points

		data["desired_contact_velocities"] = desired_contact_velocities
		data["observed_contact_velocities"] = observed_contact_velocities

		data["desired_wrench"] = desired_wrench
		data["observed_wrench"] = observed_wrench

		data["desired_block_position"] = desired_block_position
		data["observed_block_position"] = observed_block_position

		data["differences"] = differences

		file = open("robot_data_dump.pkl", "wb")
		pickle.dump(data, file)
		file.close()







if __name__ == "__main__":

	control = Robot_Control(mode="simulation", block_info_from="simulation")

	# Move Up and Down
	object_size = 0.065
	block_com_position = [0.0, 0.0, 0.05]
	block_orientation = [0,0,0,1]
	control.add_new_object(object_size, block_com_position, block_orientation)
	control.optimised_trajectory()
	control.reset_state()
