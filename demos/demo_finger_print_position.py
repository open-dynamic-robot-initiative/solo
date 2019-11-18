#!/usr/bin/env python3
"""Send zero-torque commands to Finger robot and print joint positions."""
import os
import numpy as np
import rospkg

import robot_interfaces
import blmc_robots

# load the default config file
config_file_path = os.path.join(
    rospkg.RosPack().get_path("blmc_robots"), "config", "finger.yml")

finger_data = robot_interfaces.finger.Data()
finger_backend = blmc_robots.create_real_finger_backend(finger_data,
                                                        config_file_path)
finger = robot_interfaces.finger.Frontend(finger_data)


finger_backend.initialize()

while True:
    t = finger.append_desired_action(robot_interfaces.finger.Action())
    pos = finger.get_observation(t).position
    print("\r%6.3f %6.3f %6.3f" % (pos[0], pos[1], pos[2]), end="")
