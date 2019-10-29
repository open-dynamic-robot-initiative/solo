#!/usr/bin/env python3
"""Send zero-torque commands to the robot and create a log"""

import time
import numpy as np

import blmc_robots
import robot_interfaces

def main():

    finger_data = robot_interfaces.finger.Data()
    finger_backend = blmc_robots.create_fake_finger_backend(finger_data)
    finger_frontend = robot_interfaces.finger.Frontend(finger_data)

    finger_backend.initialize()

    kp = 5
    kd = 0

    Action = robot_interfaces.finger.Action
    desired_torque = np.zeros(3)

    block_size = 100
    filename = "log.csv";
    finger_logger = robot_interfaces.finger.FingerLogger(finger_data, block_size, filename)
    finger_logger.start()

    while True:

        desired_position = np.random.rand(3) * 6 - 1

        for _ in range(1000):

            # print("hello")

            t = finger_frontend.append_desired_action(Action(torque=desired_torque))

            pos = finger_frontend.get_observation(t).position
            vel = finger_frontend.get_observation(t).velocity

            position_error = desired_position - pos
            desired_torque = kp * position_error - kd * vel


if __name__ == "__main__":
    main()
