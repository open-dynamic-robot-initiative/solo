import math
import time
import pinocchio
import numpy as np

from robot_wrapper import Robot

import blmc_robots
import robot_interfaces

import matplotlib.pyplot as plt

import ipdb


plt.axis([0,10, -1.5,1.5])
TIME = 0


class Robot_Control:

	def __init__(self):
		self.initialize_robot()
		self.reset_state()
		

	def reset_state(self):
		self.torque = np.array([0,-0.3,0.1]*3)
		self.is_grabbing = False
		self.f_applied = [0,0,0] # desired external force in three directions
		self.desired_external_force = np.matrix(np.zeros(9)).T
		self.object_position = None # initially unknown
		self.object_size = None # initially unknown
		self.landing_position = [0,-0.12,0.03] 
		self.fixed_values = np.matrix(np.zeros(9)).T
		self.plot_dict = {'Fd':[], 'P':[], 'V':[], 'Fo':[]}
		self.plot_real_time = True
		self.change_force_dynamically = True


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
		self.finger = blmc_robots.Robot(robot_interfaces.trifinger,
		                           blmc_robots.create_trifinger_backend,
		                           "trifinger.yml")
		self.finger.initialize()

		# End effector Ids
		self.finger_tip_links = ["finger_tip_link_0", "finger_tip_link_120", "finger_tip_link_240"]
		self.finger_tip_ids = [self.robot_model.getFrameId(name) for name in self.finger_tip_links]



	def add_new_object(self, object_size, object_position):
		'''
		Adds a new object to manipulate
		'''
		self.object_size = object_size
		self.object_position = object_position



	def get_desired_state(self):
		# Calculating the desird position to reach
		self.x_des = [np.matrix([self.object_position[0], self.object_position[1] + self.object_size/2, self.object_position[2]]).T,
				 np.matrix([self.object_position[0] + self.object_size/2, self.object_position[1] - self.object_size/4, self.object_position[2]]).T,
				 np.matrix([self.object_position[0] - self.object_size/2, self.object_position[1] - self.object_size/4, self.object_position[2]]).T]

		self.x_des = np.vstack(self.x_des)

		self.v_des = [np.matrix([0,0,0]).T, np.matrix([0,0,0]).T, np.matrix([0,0,0]).T]
		self.v_des = np.vstack(self.v_des)

		# Kp and Kd
		self.Kp = np.diag(np.full(9,81)) # np.diag(np.full(9,81))
		self.Kd = np.diag(np.full(9,0.09)) # np.diag(np.full(9,0.09))


	def calculate_x_compensation(self):
		'''
		Compensation along x,y,z-direction
		'''
		position_error = np.matrix(np.zeros(9)).T

		if self.is_grabbing:
			# first arm
			position_error[0] = self.x_des.data.tolist()[0][0] - self.x_obs.data.tolist()[0][0]
			position_error[1] = self.x_des.data.tolist()[1][0] - self.x_obs.data.tolist()[1][0]
			position_error[2] = self.x_des.data.tolist()[2][0] - self.x_obs.data.tolist()[2][0]
			# second arm
			position_error[3] = self.x_des.data.tolist()[3][0] - self.x_obs.data.tolist()[3][0]
			position_error[4] = self.x_des.data.tolist()[4][0] - self.x_obs.data.tolist()[4][0]
			position_error[5] = self.x_des.data.tolist()[5][0] - self.x_obs.data.tolist()[5][0]
			# third
			position_error[6] = self.x_des.data.tolist()[6][0] - self.x_obs.data.tolist()[6][0]
			position_error[7] = self.x_des.data.tolist()[7][0] - self.x_obs.data.tolist()[7][0]
			position_error[8] = self.x_des.data.tolist()[8][0] - self.x_obs.data.tolist()[8][0]

		self.compensation = self.Kp * position_error

		self.fixed_values =  self.x_obs + position_error



	def get_torques(self):
		'''
		Torque = J.T * (F_desired + [kp*(x-x_des) + kd*(x`-x`_des)] )
		'''
		# Calculating total force at the end effector
		self.calculate_x_compensation()

		force = self.Kp * (self.x_des - self.x_obs) + self.Kd * (self.v_des - self.v_obs)
		# force = self.Kp * (self.x_des - self.fixed_values) + self.Kd * (self.v_des - self.v_obs)
		
		total_force = self.desired_external_force + force

		torque = self.J.T * total_force
		self.torque = np.array(torque).reshape(-1)




	def calculate_q_compensation(self):
		obs = self.finger.frontend.get_observation(self.t)
		q = obs.position.copy()
		tau = obs.torque

		if not self.is_grabbing:
			return q


		for i, (rad, torque) in enumerate(zip(q,tau)):
			if torque > 0.03:
				q[i] += math.radians(-1 * torque - 0.7)
			if torque < -0.1:
				q[i] += math.radians(-1 * torque + 0.7)  

		return q





	def get_next_state(self, debug=False):
		'''
		Returns the next state of the robot after applying the desired torque
		'''
		# Applying torque
		t = self.finger.frontend.append_desired_action(self.finger.Action(torque=self.torque))
		self.t = t

		# Getting robot's joint state
		obs = self.finger.frontend.get_observation(t)
		q = obs.position
		w = obs.velocity

		q = self.calculate_q_compensation()

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
			J_inv = np.linalg.inv(self.J.T)
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
				if self.is_grabbing:
					print("Q position is \n\t", degrees[:3], "\n\t", degrees[3:6], "\n\t", degrees[6:])
					q = self.calculate_q_compensation()
					degrees = []
					for i in q:
						degrees.append(math.degrees(i))
					print("Fixed Q position is \n\t", degrees[:3], "\n\t", degrees[3:6], "\n\t", degrees[6:])

				else:
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

		if self.change_force_dynamically and self.is_grabbing and self.t % 1000 == 0:
			global TIME
			# self.f_applied[0] = (1 - math.cos(TIME/10)) * 1.5
			# self.f_applied[1] = (1 - math.cos(TIME/10)) * 1.5
			self.f_applied[0] += 0.1
			self.f_applied[1] += 0.1

		# Force along x axis
		# desired_external_force[0] = f_x
		desired_external_force[3] = -self.f_applied[0]
		desired_external_force[6] = self.f_applied[0]

		# Force along y axis
		desired_external_force[1] = -self.f_applied[1]
		# desired_external_force[4] = self.f_applied[]
		# desired_external_force[7] = self.f_applied[]

		# Force along z axis
		desired_external_force[2] = self.f_applied[2]
		desired_external_force[5] = self.f_applied[2]
		desired_external_force[8] = self.f_applied[2]

		self.desired_external_force = desired_external_force




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
			self.get_next_state(debug=True)

			# Calculating required torque for total force
			self.get_torques()

			# Setting timeout for "timeout seconds"
			i += 1
			if i % (timeout*1000) == 0:
				print("Done **************************************************************************")
				break

			if exit_before_timeout and all(np.abs(np.subtract(self.object_position, desired_position)) < 0.005):
				break

		if offset is not None:
			self.object_size -= offset



	def grasp(self, f_applied=[-0.3,-0.3,0]):
		self.f_applied = f_applied
		self.is_grabbing = True
		self.move(self.object_position, timeout=100, exit_before_timeout=False)


	def release(self):
		self.is_grabbing = False
		self.desired_external_force = np.matrix(np.zeros(9)).T


	def follow_predefined_trajectory(self):
		assert self.object_position is not None
		assert self.object_size is not None
		self.move(self.object_position, offset=0.0, timeout=2, exit_before_timeout=False) # reaching the object
		self.grasp()
		self.generate_plots()
		self.move(self.object_position[:2] + [0.21]) # picking up 
		self.move(self.landing_position[:2] + [self.object_position[2]]) # moving forward
		self.move(self.landing_position) # moving down
		self.release()
		self.move(self.landing_position[:2] + [0.25], offset=0.02) # moving away from the object after placing it
		self.move([0,0,0.25], offset=0.02) # moving to the centre (just using it for a better ending, for now)
		self.generate_plots()







if __name__ == "__main__":

	control = Robot_Control()

	# First Object
	object_size = 0.065      
	object_position = [0.0, 0.0, object_size/2 + 0.008]

	control.add_new_object(object_size, object_position)
	control.follow_predefined_trajectory()
	control.reset_state()


	# Second Object
	object_size = 0.065      
	object_position = [0.08, 0.0, object_size/2 + 0.008]

	control.add_new_object(object_size, object_position)
	control.landing_position = [0,-0.12,0.11] 
	control.follow_predefined_trajectory()
	control.reset_state()


	# Third Object
	object_size = 0.045      
	object_position = [-0.08, 0.0, object_size/2 + 0.008]

	control.add_new_object(object_size, object_position)
	control.landing_position = [0,-0.12,0.16] 
	control.follow_predefined_trajectory()
	control.reset_state()






















































































# # Initialize pinocchio model
# robot = Robot(visualizer='gepetto',
#                   viscous_friction=0.0)
# robot.initViewer(loadModel=True)

# robot_model = robot.model
# robot_data = robot_model.createData()

# # Initialize real robot
# finger = blmc_robots.Robot(robot_interfaces.trifinger,
#                            blmc_robots.create_trifinger_backend,
#                            "trifinger.yml")
# finger.initialize()

# # End effector Ids
# finger_tip_links = ["finger_tip_link_0", "finger_tip_link_120", "finger_tip_link_240"]
# finger_tip_ids = [robot_model.getFrameId(name) for name in finger_tip_links]

# plt.axis([0,10, -1.5,1.5])

# TIME = 0







# def get_desired_state(object_size, object_position):

# 	# Calculating the desird position to reach
# 	x_des = [np.matrix([object_position[0], object_position[1] + object_size/2, object_position[2]]).T,
# 			 np.matrix([object_position[0] + object_size/2, object_position[1] - object_size/4, object_position[2]]).T,
# 			 np.matrix([object_position[0] - object_size/2, object_position[1] - object_size/4, object_position[2]]).T]

# 	# x_des = [np.matrix([object_position[0], object_position[1] + object_size/2, object_position[2]]).T,
# 	# 		 np.matrix([object_position[0], object_position[1] - object_size/2, object_position[2]]).T,
# 	# 		 np.matrix([object_position[0], object_position[1] - object_size/2, object_position[2]]).T]
	
# 	x_des = np.vstack(x_des)

# 	v_des = [np.matrix([0,0,0]).T, np.matrix([0,0,0]).T, np.matrix([0,0,0]).T]
# 	v_des = np.vstack(v_des)

# 	# Kp and Kd
# 	Kp = np.diag(np.full(9,81)) # np.diag(np.full(9,81))
# 	Kd = np.diag(np.full(9,0.09)) # np.diag(np.full(9,0.09))

# 	return x_des, v_des, Kp, Kd


# def get_torques(J, x_obs, x_des, v_obs, v_des, Kp, Kd, desired_external_force=np.matrix(np.zeros(9)).T, tau=None):
# 	'''
# 	Torque = J.T * (F_desired + [kp*(x-x_des) + kd*(x`-x`_des)] )
# 	'''
# 	# Calculating total force at the end effector
# 	force = Kp * (x_des - x_obs) + Kd * (v_des - v_obs)
# 	total_force = desired_external_force + force

# 	torque = J.T * total_force
# 	torque = np.array(torque).reshape(-1)

# 	# if tau is not None:
# 	# 	position = Kp * (x_des - x_obs)
# 	# 	velocity = Kd * (v_des -v_obs)
# 	# 	global TIME
# 	# 	J_inv = np.linalg.inv(J.T)
# 	# 	F = J_inv * tau
		
# 	# 	plt.plot(TIME, F.data.tolist()[3][0], 'bo')
# 	# 	plt.plot(TIME, position.data.tolist()[3][0], 'y+')
# 	# 	plt.plot(TIME, velocity.data.tolist()[3][0], 'g+')
# 	# 	plt.plot(TIME, desired_external_force.data.tolist()[3][0], 'ro')
# 	# 	if TIME % 20 == 0:
# 	# 		plt.pause(0.0001)
# 	# 	TIME += 1

	
# 	return torque


# def get_next_state(torque, force_applied=None, return_tau=False):
# 	'''
# 	Returns the next state of the robot after applying the desired torque
# 	'''
# 	# Applying torque
# 	t = finger.frontend.append_desired_action(finger.Action(torque=torque))

# 	# Getting robot's joint state
# 	obs = finger.frontend.get_observation(t)
# 	q = obs.position
# 	w = obs.velocity

# 	# forward kinematics
# 	pinocchio.computeJointJacobians(robot_model, robot_data, q)
# 	pinocchio.framesForwardKinematics(robot_model, robot_data, q)

# 	# Computing the Jacobians
# 	J = [pinocchio.getFrameJacobian(robot_model, robot_data, tip_id, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3] for tip_id in finger_tip_ids]
# 	J = np.vstack(J)

# 	# Calculating x_observed (end effector's position)
# 	x_obs = [robot_data.oMf[tip_id].translation for tip_id in finger_tip_ids]
# 	x_obs = np.vstack(x_obs)

# 	# x` = J * q` (x`_des = 0) (end effector velocity)
# 	v_obs = J * np.matrix(w).T


# 	############## TESTING
# 	if force_applied :
# 		tau = np.matrix(obs.torque).T
# 		J_inv = np.linalg.inv(J.T)
# 		F = J_inv * tau

# 		direction = 'x'
# 		idx = {'x':[0,3,6],
# 			   'y':[1,4,7],
# 			   'z':[2,5,8]}
# 		if t%20==0:
# 			degrees = []
# 			for i in q:
# 				degrees.append(math.degrees(i))

# 			# print("Force applied along axis:",
# 			# 		F[idx[direction][0]],
# 			# 		F[idx[direction][1]],
# 			# 		F[idx[direction][2]],
# 			# 		"*****",
# 			# 		F[idx[direction][0]]/force_applied*100, 
# 			# 		F[idx[direction][1]]/force_applied*100, 
# 			# 		F[idx[direction][2]]/force_applied*100)
# 			print("Q position is \n", degrees[:3], "\n", degrees[3:6], "\n", degrees[6:])
# 			print(force_applied)
		
# 			print("X_obsered:\n",
# 					x_obs[:3].T, "\n",
# 					x_obs[3:6].T, "\n",
# 					x_obs[6:].T)

# 		# print("Percentage of force actually applied:",
# 		# 		F[idx[direction][0]]/force_applied*100, 
# 		# 		F[idx[direction][1]]/force_applied*100, 
# 		# 		F[idx[direction][2]]/force_applied*100)

# 		# print("Force applied along axis:",
# 		# 			F[idx[direction][0]],
# 		# 			F[idx[direction][1]],
# 		# 			F[idx[direction][2]])
# 	# ##############
# 	if return_tau:
# 		return x_obs, v_obs, J, tau

# 	return x_obs, v_obs, J




# def get_desired_external_force(f_x=0, f_y=0, f_z=0):
# 	# Calculating desired_external_force to apply to grasp the object
# 	desired_external_force = np.matrix(np.zeros(9)).T

# 	# Force along x axis
# 	# desired_external_force[0] = f_x
# 	desired_external_force[3] = -f_x
# 	desired_external_force[6] = f_x

# 	# Force along y axis
# 	desired_external_force[1] = -f_y
# 	# desired_external_force[4] = f_y
# 	# desired_external_force[7] = f_y

# 	# Force along z axis
# 	desired_external_force[2] = f_z
# 	desired_external_force[5] = f_z
# 	desired_external_force[8] = f_z

# 	return desired_external_force



# def reach_target(object_size, object_position, timeout=10, offset=0.02):
# 	# Add offset to the object size
# 	# object_size += offset
# 	# Getting final state for end_effector
# 	x_des, v_des, Kp, Kd = get_desired_state(object_size, object_position)

# 	# Initial Values
# 	torque = np.array([0]*9)
# 	# torque = np.array([0,-0.3,0.1]*3)
# 	i = 0
# 	while True:
# 		# Get robot's observed state
# 		x_obs, v_obs, J, tau = get_next_state(torque,1, True)

# 		# Calculating torque values needed for desired end effector position
# 		torque = get_torques(J, x_obs, x_des, v_obs, v_des, Kp, Kd)

# 		# Setting timeout for "timeout seconds"
# 		i += 1
# 		if i % (timeout*1000) == 0:
# 			print("Reached Target *******************************************************************")
# 			break





# def grasp_target(object_size, object_position, timeout=10):
# 	# Getting final state for end_effector
# 	# object_size -= 0.020
# 	x_des, v_des, Kp, Kd = get_desired_state(object_size, object_position)
	
# 	# Calculating desired_external_force to apply to grasp the object
# 	f_x = 1
# 	f_y = 1.5
# 	desired_external_force = get_desired_external_force(2,0)#(f_x,f_y)

# 	# Initial Values
# 	torque = np.array([0]*9)
# 	i = 0
# 	while True:
# 		# Get robot's observed state
# 		x_obs, v_obs, J, tau = get_next_state(torque,5, True)

# 		# Calculating torque values needed for grasping object at desired end effector position
# 		torque = get_torques(J, x_obs, x_des, v_obs, v_des, Kp, Kd, desired_external_force, tau)

# 		# Setting timeout for "timeout seconds"
# 		i += 1
# 		if i % (timeout*1000) == 0:
# 			print("Grasped Object")
# 			break
# 		# time.sleep(0.01)




# def move_down(object_size, object_position, torque, z_desired=0.03, timeout=15):
# 	# Initial Values
# 	i = 1
# 	z_starting = object_position[2]
	
# 	while True:
# 		# Slowly changing the y-position for the end_effector
# 		z = (1 - math.cos(math.radians(i/10))) * (z_desired - z_starting)/2 + z_starting
# 		object_position[2] = z

# 		# Calculating desired_external_force to apply to grasp the object
# 		desired_external_force = get_desired_external_force()

# 		# Getting final state for end_effector
# 		x_des, v_des, Kp, Kd = get_desired_state(object_size, object_position)
		
# 		# Get robot's observed state
# 		x_obs, v_obs, J = get_next_state(torque)

# 		# Calculating required torque for total force
# 		torque = get_torques(J, x_obs, x_des, v_obs, v_des, Kp, Kd, desired_external_force)

# 		# Setting timeout for "timeout seconds"
# 		i += 1
# 		if i % (timeout*1000) == 0:
# 			print("Done")
# 			break


# 		if np.abs(z-z_desired) < 0.005:
# 			object_size += 0.02
# 			reach_target(object_size, object_position)
# 			z_starting = object_position[2]
# 			z_desired = 0.25
# 			i = 0
# 			while True:
# 				z = (1 - math.cos(math.radians(i/10))) * (z_desired - z_starting)/2 + z_starting
# 				object_position[2] = z
# 				# Getting final state for end_effector
# 				x_des, v_des, Kp, Kd = get_desired_state(object_size, object_position)
				
# 				# Get robot's observed state
# 				x_obs, v_obs, J = get_next_state(torque)

# 				# Calculating required torque for total force
# 				torque = get_torques(J, x_obs, x_des, v_obs, v_des, Kp, Kd)
# 				i += 1
# 				if i % 1500 == 0:
# 					print("Done")
# 					break
# 			break








# def move_forward(object_size, object_position, torque, timeout=15):
# 	# Initial Values
# 	i = 1
# 	y_starting = object_position[1]
# 	y_desired = -0.12

# 	x_starting = object_position[0]
# 	x_desired = 0
# 	while True:
# 		# Slowly changing the y-position for the end_effector
# 		y = (1 - math.cos(math.radians(i/10))) * (y_desired - y_starting)/2 + y_starting
# 		object_position[1] = y

# 		x = (1 - math.cos(math.radians(i/10))) * (x_desired - x_starting)/2 + x_starting
# 		object_position[0] = x


# 		# Calculating desired_external_force to apply to grasp the object
# 		desired_external_force = get_desired_external_force()

# 		# Getting final state for end_effector
# 		x_des, v_des, Kp, Kd = get_desired_state(object_size, object_position)
		
# 		# Get robot's observed state
# 		x_obs, v_obs, J = get_next_state(torque, 1)

# 		# Calculating required torque for total force
# 		torque = get_torques(J, x_obs, x_des, v_obs, v_des, Kp, Kd, desired_external_force)

# 		# Setting timeout for "timeout seconds"
# 		i += 1
# 		if i % (timeout*1000) == 0:
# 			print("Done")
# 			break

# 		if np.abs(y-y_desired) < 0.005:
# 			i -= 1
# 			print("Moving Down")
# 			return torque, object_position






# def pickup_target(object_size, object_position, torque=np.array([0]*9), timeout=100, hold_in_air=False):
# 	# Getting final state for end_effector
# 	# object_size -= 0.015
# 	x_des, v_des, Kp, Kd = get_desired_state(object_size, object_position)
	
# 	# Initial Values
# 	i = 1
# 	while True:
# 		# Slowly changing the z-position for the end_effector
# 		z = (1 - math.cos(math.radians(i/10))) * 0.10 + 0.05

# 		if hold_in_air:
# 			if z > 0.21:
# 				z = 0.21
# 				i -= 1
# 				return torque, object_position

# 		object_position[2] = z

# 		######### TESTING ######
# 		# if i%200 == 0:
# 		# 	x = y = np.random.uniform(-0.01, 0.01)
# 		# 	z = np.random.uniform(-0.01, 0.01)
# 		# 	object_position = [object_position[0]+x, object_position[1]+y, object_position[2]+z]
# 		########################

# 		# Slowly changing the desired_force in z
# 		f_z = math.sin(math.radians(i)) * 0.6
# 		f_z = 0
# 		desired_external_force = get_desired_external_force()

# 		# Getting final state for end_effector
# 		x_des, v_des, Kp, Kd = get_desired_state(object_size, object_position)
		
# 		# Get robot's observed state
# 		x_obs, v_obs, J, tau = get_next_state(torque, 1, True)

# 		# Calculating required torque for total force
# 		torque = get_torques(J, x_obs, x_des, v_obs, v_des, Kp, Kd, desired_external_force, tau=tau)

# 		# Setting timeout for "timeout seconds"
# 		i += 1
# 		if i % (timeout*1000) == 0:
# 			print("Done")
# 			break


# def reach_random_positions():
# 	while True:
# 		x = np.random.uniform(-0.1, 0.1)
# 		y = np.random.uniform(-0.1, 0.1)
# 		z = np.random.uniform(0.04, 0.2)
# 		object_position = [x,y,z]
# 		reach_target(0.0,object_position,timeout=0.05)



# def position_tune():
# 	position = np.zeros(9)
# 	for _ in range(1000): 
# 		t = finger.frontend.append_desired_action(
# 	            finger.Action(position=position))
# 	blmc_robots.demo_print_position(finger)



# def move_around_with_object(object_size, object_position, timeout=100):

# 	# Initial Values
# 	torque = np.array([0]*9)
# 	i = 0
# 	while True:
# 		# Get robot's observed state
# 		x_obs, v_obs, J = get_next_state(torque, 1)

# 		if i%10 == 0:
# 			x = y = np.random.uniform(-0.05, 0.05)
# 			z = np.random.uniform(-0.04, 0.1)
# 			i = 0

# 		xx = math.sin(math.radians(i)) * (x-x_obs[0].data.tolist()[0][0])  				+ x_obs[0].data.tolist()[0][0]
# 		yy = math.sin(math.radians(i)) * (y-x_obs[1].data.tolist()[0][0]+object_size/2) + x_obs[1].data.tolist()[0][0] - object_size/2
# 		zz = math.sin(math.radians(i)) * (z-x_obs[2].data.tolist()[0][0]) 				+ x_obs[2].data.tolist()[0][0]

# 		object_position = [xx,yy,zz]

# 		# Getting final state for end_effector
# 		x_des, v_des, Kp, Kd = get_desired_state(object_size, object_position)

# 		# Extrenal force
# 		desired_external_force = get_desired_external_force(0,0)

# 		# Calculating required torque for total force
# 		torque = get_torques(J, x_obs, x_des, v_obs, v_des, Kp, Kd, desired_external_force)

# 		# Setting timeout for "timeout seconds"
# 		i += 1/10
# 		if i % (timeout*1000) == 0:
# 			print("Done")
# 			break
































# # position_tune()
# # reach_random_positions()
# # reach_target(object_size, object_position[:2]+[0.25])
# for _ in range(100):
# 	reach_target(object_size, object_position)
# 	# blmc_robots.demo_print_position(finger)
# 	# input()
# 	grasp_target(object_size, object_position)
# 	# input()

# last_torque, last_object_position = pickup_target(object_size, object_position, hold_in_air=True) 
# last_torque, last_object_position = move_forward(object_size, last_object_position, last_torque)
# move_down(object_size, last_object_position, last_torque)




# object_position = [0.08, 0.0, object_size/2 + 0.008]

# reach_target(object_size, object_position[:2]+[0.25])
# reach_target(object_size, object_position)
# grasp_target(object_size, object_position)
# last_torque, object_position = pickup_target(object_size, object_position, hold_in_air=True) 
# last_torque, object_position = move_forward(object_size, object_position, last_torque)
# move_down(object_size, object_position, last_torque, 0.11)





# object_size = 0.045
# object_position = [-0.08, 0.0, object_size/2 + 0.008]

# reach_target(object_size, object_position[:2]+[0.25])
# reach_target(object_size, object_position)
# grasp_target(object_size, object_position)
# last_torque, object_position = pickup_target(object_size, object_position, hold_in_air=True) 
# last_torque, object_position = move_forward(object_size, object_position, last_torque)
# move_down(object_size, object_position, last_torque, 0.16)

# # move_around_with_object(object_size, object_position)

# object_position = [0.0, 0.06, object_size/2 + 0.008]

# reach_target(object_size, object_position[:2]+[0.25])