from camera_readings import Camera
from force_optimization import Robot_Control
from multiprocessing import Process, Queue


if __name__ == '__main__':

	control = Robot_Control(mode="real")

	camera = Camera()
	
	cameraQueue = Queue()

	process_camera = Process(target=camera.get_live_feed, args=(cameraQueue))
	process_camera.daemon = True
	process_camera.start()

	control.start(cameraQueue)
