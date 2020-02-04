import math
import time
import pinocchio
import numpy as np

from robot_wrapper import Robot

import blmc_robots
import robot_interfaces

import matplotlib.pyplot as plt
import sim_finger

import ipdb
from basic_simulator.basic_simulator import CubeSimulator
from quadprog import solve_qp


from object_tracker import ObjectTracker
import transformations as tf
from py_blmc_kinect_camera import KinectCamera


# plt.axis([0,10, -1.5,1.5])
TIME = 0


class Robot_Control:

	def __init__(self, mode="real"):
		self.mode = mode
		self.initialize_robot()
		self.reset_state()
		if self.mode == "simulation":
			self.finger.remove_block()

		

	def reset_state(self):
		self.torque = np.array([0,0,0.]*3)
		self.is_grabbing = False
		self.f_applied = [0,0,0] # desired external force in three directions
		self.desired_external_force = np.matrix(np.zeros(9)).T
		self.object_position = None # initially unknown
		self.object_size = None # initially unknown
		self.landing_position = [0,-0.12,0.03] 
		self.fixed_values = np.matrix(np.zeros(9)).T
		self.plot_dict = {'Fd':[], 'P':[], 'V':[], 'Fo':[]}
		self.plot_real_time = False
		self.change_force_dynamically = False
		self.t = 0
		self.q2 = np.resize([0,0,0.20,0,0,0,1],(7,1))
		self.last_time_step = 0
		self.global_time_controller = 0
		self.block_mass = 0.076


	def initialize_robot(self):
		'''
		Loads the URDF in pinnochio and initializes the real robot

		'''
		# Initialize pinocchio model
		self.robot = Robot(visualizer='gepetto',
		              viscous_friction=0.0)
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
			self.finger = sim_finger.Finger(finger_type="trifinger", time_step=1.e-3)

		# End effector Ids
		self.finger_tip_links = ["finger_tip_link_0", "finger_tip_link_120", "finger_tip_link_240"]
		self.finger_tip_ids = [self.robot_model.getFrameId(name) for name in self.finger_tip_links]


	def box_state_from_tip_position(self):
		end_effector_position = [self.robot_data.oMf[tip_id].translation for tip_id in self.finger_tip_ids]
		box_com_position = [(end_effector_position[0][i] + 
								end_effector_position[1][i] + 
								end_effector_position[2][i])/3.0 for i in range(3)]


		box_com_position = np.resize(box_com_position, (3,1))

		dir_x = np.resize((end_effector_position[1] - end_effector_position[2]),3)
		dir_x = dir_x / max(np.linalg.norm(dir_x),0.00000001)

		a = np.resize((end_effector_position[2] - end_effector_position[0]),3)
		a = a /  max(np.linalg.norm(a), 0.000000001)

		b = np.resize((end_effector_position[1] - end_effector_position[0]),3)
		b = b / max(np.linalg.norm(b), 0.000000001)

		dir_z = np.cross(a,b)
		dir_z = dir_z / max(np.linalg.norm(dir_z), 0.0000000001)

		dir_y = np.cross(dir_z, dir_x)
		dir_y = dir_y / max(np.linalg.norm(dir_y), 0.000000001)

		# print(dir_x, np.linalg.norm(dir_x))
		# print(dir_y, np.linalg.norm(dir_y))
		# print(dir_z, np.linalg.norm(dir_z))

		dir_x = dir_x
		dir_y = dir_y
		dir_z = dir_z

		self.rotation_matrix = np.matrix([dir_x, dir_y, dir_z]).T
		self.quat = pinocchio.Quaternion(self.rotation_matrix).coeffs()

		# Setting the position offset
		offset =  np.array([0,-0.0054, 0]).reshape(-1,1)

		box_com_position = box_com_position + self.rotation_matrix.dot(offset)

		# if self.t % 100 == 0:
		# 	print("Block Position:", box_com_position)

		# self.quat[:] = 0
		# self.quat[-1] = 1
		# ipdb.set_trace()
		# print("Box State:", np.concatenate([box_com_position, self.quat]))

		return np.concatenate([box_com_position, self.quat])


		ipdb.set_trace()



	def get_cube_state(self):
		if self.mode == "simulation":
			return self.finger.get_block_state()

		if self.mode == "real":
			quat, trans = self.object_tracker.get_object_pose()
			quat = np.concatenate([quat[1:],[quat[0]]], axis = 0)
			quat[:] = 0
			quat[3] = 1
			# trans[:2] = 0
			# trans[2] = 0.05 + 0.2 * (self.t/1000)
			return self.box_state_from_tip_position()
			
			# return np.concatenate([trans, quat], axis=0).reshape(-1,1)
			# return self.cube.state.copy()[:7]

		if self.mode == "pinocchio":
			return self.cube.state.copy()[:7]



	def add_new_object(self, size, position, orientation=[0,0,0,1]):
		'''
		Adds a new object to manipulate
		'''
		self.object_size = size
		self.object_position = position
		self.object_orientation = orientation

		if self.mode == "simulation" or self.mode == "pinocchio": # adding object in pybullet
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

			initial_object_in_base_link = tf.compose_matrix(angles=(0, 0, 0),
			                                        translate=(0, 0, 0.05))
			self.object_tracker = ObjectTracker()
			self.object_tracker.initialize(initial_object_in_base_link)

		self.cube_q1 = self.get_cube_state()
		self.cube_q_last = self.get_cube_state()


	def get_desired_state(self):
		# Calculating the desird position to reach
		self.x_des = [np.matrix([self.object_position[0], self.object_position[1] + self.object_size/2, self.object_position[2] + 0.004]).T,
				 np.matrix([self.object_position[0] + self.object_size/2, self.object_position[1] - self.object_size/8, self.object_position[2]]).T,
				 np.matrix([self.object_position[0] - self.object_size/2, self.object_position[1] - self.object_size/8, self.object_position[2]]).T]

		# self.x_des = [np.matrix([self.object_position[0], self.object_position[1] + self.object_size/2, self.object_position[2]]).T,
		# 		 np.matrix([self.object_position[0] + self.object_size/2, self.object_position[1], self.object_position[2]]).T,
		# 		 np.matrix([self.object_position[0] - self.object_size/2, self.object_position[1], self.object_position[2]]).T]

		self.x_des = np.vstack(self.x_des)

		self.v_des = [np.matrix([0,0,0]).T, np.matrix([0,0,0]).T, np.matrix([0,0,0]).T]
		self.v_des = np.vstack(self.v_des)

		# Kp and Kd
		self.Kp = np.diag(np.full(9,81)) # np.diag(np.full(9,81))
		self.Kd = np.diag(np.full(9,0.09)) # np.diag(np.full(9,0.09))



	def get_torques(self):
		'''
		Torque = J.T * (F_desired + [kp*(x-x_des) + kd*(x`-x`_des)] )
		'''
		# Calculating total force at the end effector
		# self.calculate_x_compensation()

		force = self.Kp * (self.x_des - self.x_obs) + self.Kd * (self.v_des - self.v_obs)
		# force = self.Kp * (self.x_des - self.fixed_values) + self.Kd * (self.v_des - self.v_obs)
		
		total_force = self.desired_external_force + force

		torque = self.J.T * total_force
		self.torque = np.array(torque).reshape(-1)


	def calculate_q_compensation(self):
		# obs = self.finger.frontend.get_observation(self.t)
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



	def get_next_state(self, debug=False):
		'''
		Returns the next state of the robot after applying the desired torque
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
			# self.t = t

			# Getting robot's joint state
			obs = self.finger.get_observation(time_stamp)
			q = np.array(obs.position)
			w = np.array(obs.velocity)




		# forward kinematics
		pinocchio.computeJointJacobians(self.robot_model, self.robot_data, q)
		pinocchio.framesForwardKinematics(self.robot_model, self.robot_data, q)

		# Computing the Jacobians
		J = [pinocchio.getFrameJacobian(self.robot_model, self.robot_data, tip_id, 
										pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3] for tip_id in self.finger_tip_ids]
		self.J = np.vstack(J)

		# Calculating x_observed (end effector's position)
		x_obs = [self.robot_data.oMf[tip_id].translation for tip_id in self.finger_tip_ids]
		self.x_obs = np.vstack(x_obs)

		# x` = J * q` (x`_des = 0) (end effector velocity)
		self.v_obs = self.J * np.matrix(w).T


		# ipdb.set_trace()

		if debug:
			tau = np.matrix(obs.torque).T
			try:
				J_inv = np.linalg.inv(self.J.T)
			except:
				J_inv = np.linalg.pinv(self.J.T)
			F = J_inv * tau

			direction = 'x'
			idx = {'x':[0,3,6],
				   'y':[1,4,7],
				   'z':[2,5,8]}

			if t%30==0:
				degrees = []
				for i in q:
					degrees.append(math.degrees(i))

				# print("Force applied along axis:",
				# 		F[idx[direction][0]],
				# 		F[idx[direction][1]],
				# 		F[idx[direction][2]],
				# 		"*****",
				# 		F[idx[direction][0]]/force_applied*100, 
				# 		F[idx[direction][1]]/force_applied*100, 
				# 		F[idx[direction][2]]/force_applied*100)
				# if self.is_grabbing:
				# 	print("Q position is \n\t", degrees[:3], "\n\t", degrees[3:6], "\n\t", degrees[6:])
				# 	q = self.calculate_q_compensation()
				# 	degrees = []
				# 	for i in q:
				# 		degrees.append(math.degrees(i))
				# 	print("Fixed Q position is \n\t", degrees[:3], "\n\t", degrees[3:6], "\n\t", degrees[6:])

				# else:
				# 	print("Q position is \n", degrees[:3], "\n", degrees[3:6], "\n", degrees[6:])
				print("Q position is \n", degrees[:3], "\n", degrees[3:6], "\n", degrees[6:])

				# print(force_applied)
			
				print("X_obsered:\n\t\t\t\t\t\t\t\t\t\t",
						self.x_obs[:3].T, "\n\t\t\t\t\t\t\t\t\t\t",
						self.x_obs[3:6].T, "\n\t\t\t\t\t\t\t\t\t\t",
						self.x_obs[6:].T)

				print("Torque obsered:\n",
						tau[:3].T, "\n",
						tau[3:6].T, "\n",
						tau[6:].T)

				# # # print("Percentage of force actually applied:",
				# # # 		F[idx[direction][0]]/force_applied*100, 
				# # # 		F[idx[direction][1]]/force_applied*100, 
				# # # 		F[idx[direction][2]]/force_applied*100)

				# print("Force applied along axis:",
				# 			F[idx[direction][0]],
				# 			F[idx[direction][1]],
				# 			F[idx[direction][2]])


				# # Plotting real-time values for the different terms of the total force
				if self.is_grabbing:
					position = self.Kp * (self.x_des - self.x_obs)
					velocity = self.Kd * (self.v_des - self.v_obs)
					global TIME

					if self.plot_real_time:
						plt.plot(TIME, F.data.tolist()[3][0], 'bo')
						plt.plot(TIME, position.data.tolist()[3][0], 'y+')
						plt.plot(TIME, velocity.data.tolist()[3][0], 'g+')
						plt.plot(TIME, self.desired_external_force.data.tolist()[3][0], 'ro')
						plt.pause(0.0001)

					else:
						self.plot_dict['Fo'].append(F.data.tolist()[3][0])
						self.plot_dict['P'].append(position.data.tolist()[3][0])
						self.plot_dict['V'].append(velocity.data.tolist()[3][0])
						self.plot_dict['Fd'].append(self.desired_external_force.data.tolist()[3][0])

					TIME += 1


	def generate_plots(self):
		print("Generating plots")
		for time in range(len(self.plot_dict['Fo'])):
			plt.plot(time, self.plot_dict['Fo'][time], 'bo')
			plt.plot(time, self.plot_dict['P'][time], 'y+')
			plt.plot(time, self.plot_dict['V'][time], 'g+')
			plt.plot(time, self.plot_dict['Fd'][time], 'ro')
			plt.pause(0.0001)
			# plt.show()


	


	def get_desired_external_force(self):
		# Calculating desired_external_force to apply to grasp the object
		desired_external_force = np.matrix(np.zeros(9)).T

		# if self.change_force_dynamically and self.is_grabbing and self.t % 1000 == 0:
		# 	global TIME
		# 	# self.f_applied[0] = (1 - math.cos(TIME/10)) * 1.5
		# 	# self.f_applied[1] = (1 - math.cos(TIME/10)) * 1.5
		# 	self.f_applied[0] += 0.1
		# 	self.f_applied[1] += 0.1

		# # Force along x axis
		# # desired_external_force[0] = f_x
		# desired_external_force[3] = -self.f_applied[0]
		# desired_external_force[6] = self.f_applied[0]

		# # Force along y axis
		# desired_external_force[1] = -self.f_applied[1]
		# # desired_external_force[4] = self.f_applied[]
		# # desired_external_force[7] = self.f_applied[]

		# # Force along z axis
		# desired_external_force[2] = self.f_applied[2]
		# desired_external_force[5] = self.f_applied[2]
		# desired_external_force[8] = self.f_applied[2]

		self.desired_external_force = self.F




	def move(self, desired_position, offset=None, timeout=5, exit_before_timeout=True):
		''' 
		Moves the arms to the desired final position
		'''
		if offset is not None:
			self.object_size += offset

		# Initial Values
		i = 0
		obj_start_pos = self.object_position

		while True:
			# Slowly changing the y-position for the end_effector
			x = (1 - math.cos(math.radians(i/10))) * (desired_position[0] - obj_start_pos[0])/2 + obj_start_pos[0]
			y = (1 - math.cos(math.radians(i/10))) * (desired_position[1] - obj_start_pos[1])/2 + obj_start_pos[1]
			z = (1 - math.cos(math.radians(i/10))) * (desired_position[2] - obj_start_pos[2])/2 + obj_start_pos[2]

			self.object_position = [x,y,z]

			# Calculating desired_external_force to apply to grasp the object
			if self.is_grabbing:
				desired_external_force = self.get_desired_external_force()
			
			# Getting final state for end_effector
			self.get_desired_state()
			
			# Get robot's observed state
			self.get_next_state(debug=False)

			# Calculating required torque for total force
			self.get_torques()

			# Setting timeout for "timeout seconds"
			i += 1
			if i % (timeout*1000) == 0:
				print("****************** Done *******************")
				break

			if exit_before_timeout and all(np.abs(np.subtract(self.object_position, desired_position)) < 0.005):
				break

		if offset is not None:
			self.object_size -= offset



	def grasp(self, f_applied=[1.1,1.1,0]):
		self.f_applied = f_applied
		# self.is_grabbing = True
		self.cnt_array = [1,1,1]
		self.move(self.object_position, timeout=1, exit_before_timeout=False)
		self.is_grabbing = True


	def release(self):
		self.is_grabbing = False
		self.cnt_array = [0,0,0]
		self.desired_external_force = np.matrix(np.zeros(9)).T


	def follow_predefined_trajectory(self):
		assert self.object_position is not None
		assert self.object_size is not None
		initial_position = self.object_position
		self.object_position = [0,0,0]
		self.move([0,0,0.25], offset=0.04, timeout=2, exit_before_timeout=False)
		if self.mode == "simulation":
			self.finger.set_block_state(initial_position, [0,0,0,1])
		self.move(initial_position, offset=0.04, timeout=2, exit_before_timeout=False) # reaching the object
		self.grasp()
		self.move(self.object_position[:2] + [0.21]) # picking up 
		self.move(self.landing_position[:2] + [self.object_position[2]]) # moving forward
		self.move(self.landing_position) # moving down
		self.release()
		self.move(self.landing_position[:2] + [0.25], offset=0.02) # moving away from the object after placing it
		self.move([0,0,0.25], offset=0.02) # moving to the centre (just using it for a better ending, for now)


	def trajectory2(self):
		self.move(self.object_position, offset=0.02, timeout=1, exit_before_timeout=False) # reaching the object
		self.grasp()
		for _ in range(5):
			self.move(self.object_position[:2] + [0.21]) # picking up 
			self.move(self.object_position[:2] + [0.045]) # picking up 


	def next_state(self):
		seconds = 6
		if (self.t*self.dt) % seconds == 0:
			self.q1 = self.cube_q1
			self.q2 = pinocchio.se3ToXYZQUAT(pinocchio.SE3.Random())
			self.q2[:3] = np.resize([0,0,0.20], (3,1))
			if self.t == 2000:
				self.q2[1] = -0.1
			self.q2[3:] = np.resize([0,0,0,1], (4,1))


			# print("Reach Q2:", self.q2)

			self.last_time_step = self.t

			final_interpolation = pinocchio.interpolate(
						self.cube.rmodel, 
						self.cube_q1, 
						self.q2, 
						1)

			# input()

			assert np.linalg.norm(self.q2-final_interpolation) <= 1.e-10




		steps = 1
		if self.t % steps == 0:
			# print("Time:", self.t, self.last_time_step)
			# state = self.get_cube_state()

			self.intermediate_state = pinocchio.interpolate(
							self.cube.rmodel, 
							self.cube_q1, 
							self.q2, 
							(steps/(1000*seconds))*(self.t - self.last_time_step))

			# else:
			self.intermediate_velocity = pinocchio.difference(
							self.cube.rmodel,
							self.cube_q1,
							self.intermediate_state) / (steps/1000)

			self.cube_q_last = np.vstack([self.intermediate_state, self.intermediate_velocity])
		# ipdb.set_trace()
		return np.vstack([self.intermediate_state, self.intermediate_velocity])


	def present_state(self):
		steps = 1
		if self.t % steps == 0:
			prev_state = self.cube_q1
			present_state = self.get_cube_state()
			self.cube_q1 = present_state

			# print(present_state)

			present_velocity = pinocchio.difference(
								self.cube.rmodel,
								prev_state,
								present_state) / (steps/1000)

			self.state = np.vstack([present_state, present_velocity])

		return self.state


	def compute_com_block(self):
		present_state = self.present_state()
		next_state = self.next_state()
		self.dx = self.cube.diff(present_state, next_state)
		self.rotation_matrix = pinocchio.XYZQUATToSe3(self.cube_q1).rotation # local to world


		# self.dx[6:] = 0

		if self.global_time_controller % (self.dt*1000) == 0:
			self.t += 1
		# print(self.cnt_array)

		self.global_time_controller += 1

		self._kc = np.matrix(np.diag(np.full(3,0.8))) # this already has mass multiplied
		# self._kc[2] = 10
		self._dc = np.matrix(np.diag(np.full(3, .1))) # this already has mass multiplied

		self._kb = np.matrix(np.diag(np.full(3,.01))) # this already has mass multiplied 0.4
		self._db = np.matrix(np.diag(np.full(3,.0000))) # this already has mass multiplied 0.0063

		# This is computed in the block's local frame
		self.block_com_forces = np.vstack([
            self.rotation_matrix.T.dot(np.array([0,0,0.74]).reshape(-1,1)) + self._kc * self.dx[:3] + self._dc *self.dx[6:9],
            self._kb * self.dx[3:6] + self._db * self.dx[9:]
        	])

		# ipdb.set_trace()

		# self.block_com_position = self.rotation_matrix.T.dot(present_state[:3])
		self.block_com_position = present_state[:3]

		# print("Centre of Mass Forces:", self.block_com_forces)
		# print(next_state)




	def compute_finger_tip_positions(self):
		if self.mode == "real":
			time_stamp = self.finger.frontend.append_desired_action(self.finger.Action(torque=self.torque))

			# Getting robot's joint state
			obs = self.finger.frontend.get_observation(time_stamp)
			self.obs = obs
			q = obs.position
			w = obs.velocity
			q = self.calculate_q_compensation() # for backlash and motorbelt timing error
			tau = np.matrix(obs.torque).T
			J_inv = np.linalg.inv(self.J.T)
			self.F_observed = J_inv * tau

		elif self.mode == "simulation" or self.mode == "pinocchio":
			time_stamp = self.finger.append_desired_action(self.finger.Action(torque=self.torque))

			# Getting robot's joint state
			obs = self.finger.get_observation(time_stamp)
			q = np.array(obs.position)
			w = np.array(obs.velocity)

		# forward kinematics
		pinocchio.computeJointJacobians(self.robot_model, self.robot_data, q)
		pinocchio.framesForwardKinematics(self.robot_model, self.robot_data, q)

		# Computing the Jacobians
		J = [pinocchio.getFrameJacobian(self.robot_model, self.robot_data, tip_id, 
										pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3] for tip_id in self.finger_tip_ids]
		self.J = np.vstack(J)

		# Calculating x_observed (end effector's position)
		self.x_obs = [self.robot_data.oMf[tip_id].translation - self.block_com_position for tip_id in self.finger_tip_ids]
		self.v_obs = self.J * np.matrix(w).T
		
		if self.mode == "pinocchio":
			self.x_obs = [np.array([[-0.00334082],[ 0.04888184],[-0.01672634]]),
						  np.array([[ 0.03910008],[ 0.00379204],[-0.01891771]]),
						  np.array([[-0.03915938],[-0.00046447],[-0.01780892]])]

		# print(self.x_obs)
		# self.x_obs = [self.rotation_matrix.T.dot(self.robot_data.oMf[tip_id].translation) - self.block_com_position for tip_id in self.finger_tip_ids]
		# self.x_obs = np.vstack(x_obs)

		# print("Finger tip position:", self.x_obs)



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
		    qp_C = -G.T
		    qp_b = -h
		    meq = 0
		return solve_qp(qp_G, qp_a, qp_C, qp_b, meq)[0]



	def compute_force_qp(self):
		self.compute_finger_tip_positions() # in the local frame to the block

		# Use the contact activation from the plan to determine which of the forces
		# should be active.
		N = (int)(np.sum(self.cnt_array))
		self._mu = 0.9

		# Setup the QP problem.
		Q = 2. * np.eye(3 * N + 6)
		Q[-6:,-6:] = 1e6 * np.eye(6)
		p = np.zeros(3 * N + 6)
		A = np.zeros((6, 3 * N + 6))
		b = self.block_com_forces

		G = np.zeros((5 * N, 3 * N + 6))
		h = np.zeros((5 * N))

		j = 0
		for i in range(3):

		    if self.cnt_array[i] == 0:
		        continue

		    A[:3, 3 * j:3 * (j + 1)] = np.eye(3)
		    A[3:, 3 * j:3 * (j + 1)] = pinocchio.utils.skew(self.x_obs[i])


		    reduced_value = 0.7

		    if i == 0:
		    	G[3*j + 0, 3 * j + 1] = -1      # -Fy >= 0
		    	G[3*j + 1, 3 * j + 0] = -reduced_value		# |mu Fy| - Fx >= 0
		    	G[3*j + 1, 3 * j + 1] = -self._mu 
		    	G[3*j + 2, 3 * j + 2] = -reduced_value		# |mu Fy| - Fz >= 0
		    	G[3*j + 2, 3 * j + 1] = -self._mu
		    	G[4*j + 1, 3 * j + 0] = reduced_value		# |mu Fy| + Fx >= 0
		    	G[4*j + 1, 3 * j + 1] = -self._mu 
		    	G[5*j + 2, 3 * j + 2] = reduced_value		# |mu Fy| + Fz >= 0
		    	G[5*j + 2, 3 * j + 1] = -self._mu 


		    if i == 1:
		    	G[3*j + 0, 3 * j + 0] = -1      # Fx <= 0
		    	G[3*j + 1, 3 * j + 1] = -reduced_value		# |mu Fx| - Fy >= 0
		    	G[3*j + 1, 3 * j + 0] = -self._mu 
		    	G[3*j + 2, 3 * j + 2] = -reduced_value		# |mu Fx| - Fz >= 0
		    	G[3*j + 2, 3 * j + 0] = -self._mu 
		    	G[4*j + 1, 3 * j + 1] = reduced_value		# |mu Fx| + Fy >= 0
		    	G[4*j + 1, 3 * j + 0] = -self._mu 
		    	G[5*j + 2, 3 * j + 2] = reduced_value		# |mu Fx| + Fz >= 0
		    	G[5*j + 2, 3 * j + 0] = -self._mu

		    if i == 2:
		    	G[3*j + 0, 3 * j + 0] = 1       # Fx >= 0
		    	G[3*j + 1, 3 * j + 1] = -reduced_value		# |mu Fx| - Fy >= 0
		    	G[3*j + 1, 3 * j + 0] = self._mu 
		    	G[3*j + 2, 3 * j + 2] = -reduced_value		# |mu Fx| - Fz >= 0
		    	G[3*j + 2, 3 * j + 0] = self._mu 
		    	G[4*j + 1, 3 * j + 1] = reduced_value		# |mu Fx| + Fy >= 0
		    	G[4*j + 1, 3 * j + 0] = self._mu 
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

		# F[0] = 0
		# F[1] = 0
		# F[2] = 1

		# F[3] = 0
		# F[4] = 0
		# F[5] = 1

		# F[6] = 0
		# F[7] = 0
		# F[8] = 1
		
		torque = self.J.T * F
		self.F = F
		if self.t % 100 == 0:
			# print("Forces before Conversion:", solx)
			# print("Present Position:", self.cube_q1)
			# print("Forces On 3 fingers:",F)
			# print("Required Block Force: ", self.block_com_forces)
			pass

		self.impedance_control_finger()

		# self.torque = np.array(torque).reshape(-1)

		# print("Troques:", self.torque)




	def impedance_control_finger(self):	
		object_state = np.resize(self.cube_q_last, 13)
		self.end_eff_des_pos = [np.matrix([object_state[0], object_state[1] + self.object_size/2, object_state[2] + 0.004]).T,
				 np.matrix([object_state[0] + self.object_size/2, object_state[1] - self.object_size/8, object_state[2]]).T,
				 np.matrix([object_state[0] - self.object_size/2, object_state[1] - self.object_size/8, object_state[2]]).T]


		self.end_eff_des_pos = [self.rotation_matrix.dot(i) for i in self.end_eff_des_pos]
		self.end_eff_des_pos = np.vstack(self.end_eff_des_pos)

		self.end_eff_des_vel = np.array([object_state[7:10]] * 3).reshape(-1,1)
		# self.end_eff_des_vel = np.array([0]*9).reshape(-1,1)
		for i in range(3):
			self.end_eff_des_vel[i] += np.linalg.norm(self.x_obs[0]) * object_state[10+i]
			self.end_eff_des_vel[i+3] += np.linalg.norm(self.x_obs[1]) * object_state[10+i]
			self.end_eff_des_vel[i+6] += np.linalg.norm(self.x_obs[2]) * object_state[10+i]


		self.end_eff_obs_pos = [self.robot_data.oMf[tip_id].translation for tip_id in self.finger_tip_ids]
		self.end_eff_obs_pos = np.vstack(self.end_eff_obs_pos)
		self.end_eff_obs_vel = self.J * np.matrix(self.obs.velocity).T
		

		# ipdb.set_trace()

		

		# Kp and Kd
		self.Kp = np.diag(np.full(9,10)) # np.diag(np.full(9,81))
		self.Kd = np.diag(np.full(9,0.09)) # np.diag(np.full(9,0.09))

		force = self.Kp * (self.end_eff_des_pos - self.end_eff_obs_pos) + self.Kd * (self.end_eff_des_vel - self.end_eff_obs_vel)
		total_force = self.F + force

		torque = self.J.T * total_force
		self.torque = np.array(torque).reshape(-1)




	def check_contacts(self):

		for i in range(3):
			# print(np.linalg.norm(self.x_obs[i]))
			if np.linalg.norm(self.x_obs[i]) < 0.06:
				self.cnt_array[i] = 1
			else:
				self.cnt_array[i] = 0





	def apply_controls(self):
		self.compute_com_block()
		self.compute_force_qp()
		# ipdb.set_trace()
		# self.check_contacts()
		u = self.cube.step(self.block_com_forces).copy()
		# time.sleep(0.001)




	def optimised_trajectory(self):
		# self.move([0,0,0.25], offset=0.02, timeout=1, exit_before_timeout=False) # reaching the object
		if self.mode == "simulation":
			self.finger.set_block_state([0,0,0.05], [0,0,0,1])
		self.move([0,0,0.03], offset=0.02, timeout=3, exit_before_timeout=True) # reaching the object
		self.grasp()
		self.t = 0
		self.global_time_controller = 0
		

		self.cube_q1 = self.get_cube_state()
		self.cube_q_last = self.get_cube_state()

		# simulation horizon 20 secs at 1.e-2 dt 
		N = 6000

		
		optimised_forces = np.zeros([N,9])
		observed_forces = np.zeros([N,9])

		desired_contact_points = np.zeros([N,9])
		observed_contact_points = np.zeros([N,9])

		desired_contact_velocities = np.zeros([N,9])
		observed_contact_velocities = np.zeros([N,9])

		desired_wrench = np.zeros([N,6])
		observed_wrench = np.zeros([N,6])



		# torques = np.zeros([N,9])
		# desired_block_forces = np.zeros([N,6])
		# observed_block_forces = np.zeros([N,6])

		desired_block_position = np.zeros([N,7])
		observed_block_position = np.zeros([N,7])

		differences = np.zeros([N,12])
		
		for t in range(N):
			self.apply_controls()

			desired_block_position[t,:] = np.resize(self.cube_q_last, 7)
			observed_block_position[t,:] = np.resize(self.cube_q1, 7)

			differences[t,:] = np.resize(self.dx, 12)

			optimised_forces[t,:] = np.resize(self.F,9)
			observed_forces[t,:] = np.resize(self.F_observed,9)

			desired_contact_points[t,:] = np.resize(self.end_eff_des_pos, 9)
			observed_contact_points[t,:] = np.resize(self.end_eff_obs_pos, 9)

			desired_contact_velocities[t,:] = np.resize(self.end_eff_des_vel, 9)
			observed_contact_velocities[t,:] = np.resize(self.end_eff_obs_vel, 9)

			desired_wrench[t,:] = np.resize(self.block_com_forces, 6)
			observed_wrench[t,:] = np.resize(self.observed_wrench, 6)


			# torques[t,:] = np.resize(self.torque, 9)
			# desired_block_forces[t,:] = np.resize(self.block_com_forces, 6)
			# observed_block_forces[t,:] = np.resize(self.observed_forces, 6)
			# self.cube.robot.display(self.cube.state[:self.cube.nq])


		fig1, ax1 = plt.subplots(9,1)
		fig1.suptitle('Contact Forces')
		for i in range(3):
				# Optimised Contact Forces (Desired)
				ax1[i].plot(1.e-3*np.arange(N), optimised_forces[:,i], '--')
				ax1[i+3].plot(1.e-3*np.arange(N), optimised_forces[:,i+3],'--')
				ax1[i+6].plot(1.e-3*np.arange(N), optimised_forces[:,i+6],'--')

				# Observed Contact Forces
				ax1[i].plot(1.e-3*np.arange(N), observed_forces[:,i])
				ax1[i+3].plot(1.e-3*np.arange(N), observed_forces[:,i+3])
				ax1[i+6].plot(1.e-3*np.arange(N), observed_forces[:,i+6])




		fig2, ax2 = plt.subplots(9,1)
		fig2.suptitle('Contact Points')
		for i in range(3):
				# Optimised Contact Forces (Desired)
				ax2[i].plot(1.e-3*np.arange(N), optimised_forces[:,i], '--')
				ax2[i+3].plot(1.e-3*np.arange(N), optimised_forces[:,i+3],'--')
				ax2[i+6].plot(1.e-3*np.arange(N), optimised_forces[:,i+6],'--')

				# Observed Contact Forces
				ax2[i].plot(1.e-3*np.arange(N), observed_forces[:,i])
				ax2[i+3].plot(1.e-3*np.arange(N), observed_forces[:,i+3])
				ax2[i+6].plot(1.e-3*np.arange(N), observed_forces[:,i+6])




		fig2, ax2 = plt.subplots(9,1)
		fig2.suptitle('Contact Velocities')
		for i in range(3):
				# Optimised Contact Forces (Desired)
				ax2[i].plot(1.e-3*np.arange(N), optimised_forces[:,i], '--')
				ax2[i+3].plot(1.e-3*np.arange(N), optimised_forces[:,i+3],'--')
				ax2[i+6].plot(1.e-3*np.arange(N), optimised_forces[:,i+6],'--')

				# Observed Contact Forces
				ax2[i].plot(1.e-3*np.arange(N), observed_forces[:,i])
				ax2[i+3].plot(1.e-3*np.arange(N), observed_forces[:,i+3])
				ax2[i+6].plot(1.e-3*np.arange(N), observed_forces[:,i+6])




		fig4, ax4 = plt.subplots(6,1)
		fig4.suptitle('Wrench')
		for i in range(3):
				# Block COM Forces
				ax4[i].plot(1.e-3*np.arange(N), desired_wrench[:,i],'--')
				ax4[i].plot(1.e-3*np.arange(N), observed_wrench[:,i])

				# Block COM Moment
				ax4[i+3].plot(1.e-3*np.arange(N), desired_wrench[:,i+3],'--')
				ax4[i+3].plot(1.e-3*np.arange(N), observed_wrench[:,i+3])





		# Block COM position 
		fig5, ax5 = plt.subplots(7,1)
		fig5.suptitle('Block Position')
		for i in range(4):
			if i < 3:
				# Block Position
				ax5[i].plot(1.e-3*np.arange(N), desired_block_position[:,i],'--')
				ax5[i].plot(1.e-3*np.arange(N), observed_block_position[:,i])

			# Block Orientation
			ax5[i+3].plot(1.e-3*np.arange(N), desired_block_position[:,i+3],'--')
			ax5[i+3].plot(1.e-3*np.arange(N), observed_block_position[:,i+3])





		# Differences
		fig, ax = plt.subplots(12,1)
		fig.suptitle('Differences')
		for i in range(3):
			# Block Position
			ax[i].plot(1.e-2*np.arange(N), differences[:,i],'--')
			# ax[i].plot(1.e-2*np.arange(N), observed_block_position[:,i])

			# Block Orientation
			ax[i+3].plot(1.e-2*np.arange(N), differences[:,i+3])
			# ax[i+3].plot(1.e-2*np.arange(N), observed_block_position[:,i+3])

			ax[i+6].plot(1.e-2*np.arange(N), differences[:,i+6])

			ax[i+9].plot(1.e-2*np.arange(N), differences[:,i+9])

		print(self.t)

		plt.show()













if __name__ == "__main__":

	control = Robot_Control(mode="real")

	# Move Up and Down
	object_size = 0.065      
	object_position = [0.0, 0.0, 0.15]
	block_orientation = [0,0,0,1]
	control.add_new_object(object_size, object_position, block_orientation)
	control.optimised_trajectory()
	control.reset_state()

	# input()

	# # First Object
	# object_size = 0.065      
	# object_position = [0.0, 0.08, object_size/2 + 0.008]

	# control.add_new_object(object_size, object_position)
	# control.follow_predefined_trajectory()
	# control.reset_state()


	# # Second Object
	# object_size = 0.065      
	# object_position = [0.08, 0.0, object_size/2 + 0.008]

	# control.add_new_object(object_size, object_position)
	# control.landing_position = [0,-0.12,0.11] 
	# control.follow_predefined_trajectory()
	# control.reset_state()


	# # Third Object
	# object_size = 0.045      
	# object_position = [-0.08, 0.0, object_size/2 + 0.008]

	# control.add_new_object(object_size, object_position)
	# control.landing_position = [0,-0.12,0.16] 
	# control.follow_predefined_trajectory()
	# control.reset_state()



