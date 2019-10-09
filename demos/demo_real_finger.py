#!/usr/bin/env python3
import time
import numpy as np

import threading
import ipdb
import robot_interfaces
import blmc_robots


def main():
    finger_data = robot_interfaces.finger.Data()
    finger_backend = blmc_robots.create_real_finger_backend("can0", "can1",
                                                               finger_data)
    finger = robot_interfaces.finger.Frontend(finger_data)

    finger_backend.initialize()

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
                position_error = (desired_position -
                                  finger.get_observation(t).angle)
                desired_torque = (kp * position_error -
                                  kd * finger.get_observation(t).velocity)

    thread = threading.Thread(target=control_loop)
    thread.setDaemon(True)
    thread.start()
    # time.sleep(1.0)

    ipdb.set_trace()


if __name__ == "__main__":
    main()
