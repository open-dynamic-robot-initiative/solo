#!/usr/bin/env python3
"""Send zero-torque commands to Finger robot and print joint positions."""
import numpy as np

import robot_interfaces
import blmc_robots


finger_data = robot_interfaces.finger.Data()
finger_backend = blmc_robots.create_real_finger_backend("can0", "can1",
                                                        finger_data)
finger = robot_interfaces.finger.Frontend(finger_data)


finger_backend.initialize()

# TODO this makes the application hang
#t = finger.get_current_time_index()
#print("t = %d" % t)

while True:
    t = finger.append_desired_action(robot_interfaces.finger.Action())
    pos = finger.get_observation(t).angle
    print("\r%6.3f %6.3f %6.3f" % (pos[0], pos[1], pos[2]), end="")
