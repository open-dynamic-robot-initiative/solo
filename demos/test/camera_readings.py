import numpy as np
from object_tracker import ObjectTracker
import transformations as tf
from py_blmc_kinect_camera import KinectCamera


class Camera:
	def __init__(self):
		initial_object_in_base_link = tf.compose_matrix(angles=(0, 0, 0),
                                                    translate=(0, 0, 0.05))

		object_tracker = ObjectTracker()
		object_tracker.initialize(initial_object_in_base_link)

		self.object_tracker = object_tracker
	
	def get_live_feed(self, cameraQueue):
		while True:
			quat, trans = self.object_tracker.get_object_pose()
			quat = np.concatenate([quat[1:],[quat[0]]], axis = 0)
			
			cameraQueue.put(np.concatenate([trans, quat], axis=0).reshape(-1,1))
