#!/usr/bin/env python3
"""Send zero-torque commands to the robot and print joint angles."""
import time
import numpy as np

import py_finger
import py_real_finger


finger_data = py_finger.Data()
finger_backend = py_real_finger.create_real_finger_backend("can0", "can1",
                                                           finger_data)
finger = py_finger.Frontend(finger_data)
finger_backend.calibrate()

desired_torque = np.zeros(3)

while True:
    for _ in range(300):
        t = finger.append_desired_action(desired_torque)
        pos = finger.get_observation(t).angle
        print("\r%6.3f %6.3f %6.3f" % (pos[0], pos[1], pos[2]), end="")
