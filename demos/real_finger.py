import time
import numpy as np

import threading
import ipdb
from py_real_finger import RealFinger


finger = RealFinger.create('can0', 'can1')
finger.current_time_index()

desired_position = np.ones(3)
kp = 5
kd = 0

def control_loop():
    desired_torque = np.zeros(3)
    while True:
        desired_position = np.random.rand(3) * 6 - 1
        for _ in range(300):
            t = finger.append_desired_action(desired_torque)
            desired_torque = kp * (desired_position - finger.get_observation(t).angle) - \
                kd * finger.get_observation(t).velocity

thread = threading.Thread(target=control_loop)

thread.start()
thread.join()
