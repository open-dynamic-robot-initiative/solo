import time
import numpy as np

import threading
import ipdb
import py_real_finger


finger = py_real_finger.create('can0', 'can1')

kp = 5
kd = 0

def control_loop():
    desired_torque = np.zeros(3)
    while True:

        # desired_torque = np.random.rand(3)  - 0.5
        # for _ in range(300):
        #     finger.append_desired_action(desired_torque)

        desired_position = np.random.rand(3) * 6 - 1
        for _ in range(300):
            t = finger.append_desired_action(desired_torque)
            desired_torque = kp * (desired_position - finger.get_observation(t).angle) - \
                kd * finger.get_observation(t).velocity

thread = threading.Thread(target=control_loop)
thread.setDaemon(True)
thread.start()
# time.sleep(1.0)

ipdb.set_trace()
