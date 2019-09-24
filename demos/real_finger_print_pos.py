#!/usr/bin/env python3
import time
import numpy as np

import py_finger
import py_real_finger


finger_data = py_finger.Data()
finger_backend = py_real_finger.create_real_finger_backend("can0", "can1",
                                                           finger_data)
finger = py_finger.Frontend(finger_data)


finger_backend.calibrate()

# FIXME this makes the application hang
#t = finger.get_current_time_index()
#print("t = %d" % t)

kp = 5
kd = 0
desired_torque = np.zeros(3)

while True:

    desired_position = np.random.rand(3) * 6 - 1
    for _ in range(300):
        t = finger.append_desired_action(desired_torque)
        #trq = finger.get_observation(t).torque
        trq = finger.get_observation(t).angle
        print("\r%6.3f %6.3f %6.3f" % (trq[0], trq[1], trq[2]), end="")
