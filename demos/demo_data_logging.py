#!/usr/bin/env python3
"""Send zero-torque commands to the robot and create log"""

import time
import numpy as np

import py_finger
import py_real_finger

finger_data = py_finger.Data()
finger_backend = py_real_finger.create_random_finger_backend(finger_data)
finger = py_finger.Frontend(finger_data)

desired_torque = np.zeros(3)

block_size = 100
filename = "log.csv";

finger_logger = py_finger.FingerLogger(finger_data, block_size, filename)
finger_logger.run()

while True:
    for _ in range(1000):
        t = finger.append_desired_action(desired_torque)

        pos = finger.get_observation(t).angle
