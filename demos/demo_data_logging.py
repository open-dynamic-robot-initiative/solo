#!/usr/bin/env python3
"""Send zero-torque commands to the robot and create a log"""

import time
import numpy as np

import robot_interfaces
import blmc_robots

def main():

    finger_data = robot_interfaces.finger.Data()
    finger_backend = blmc_robots.create_fake_finger_backend(finger_data)
    finger = robot_interfaces.finger.Frontend(finger_data)

    desired_torque = np.zeros(3)

    block_size = 100
    filename = "log.csv";

    finger_logger = robot_interfaces.finger.FingerLogger(finger_data, block_size, filename)
    finger_logger.run()

    while True:
        for _ in range(1000):
            t = finger.append_desired_action(desired_torque)

            pos = finger.get_observation(t).angle

if __name__ == "__main__":
    main()
