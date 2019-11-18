#!/usr/bin/env python3
"""Send zero-torque commands to Finger robot and print joint positions."""
import os
import numpy as np
import rospkg

from robot_interfaces import one_joint
import blmc_robots

# load the default config file
config_file_path = os.path.join(
    rospkg.RosPack().get_path("blmc_robots"), "config", "onejoint.yml")

robot_data = one_joint.Data()
robot_backend = blmc_robots.create_one_joint_backend(robot_data,
                                                     config_file_path)
robot = one_joint.Frontend(robot_data)


robot_backend.initialize()

while True:
    t = robot.append_desired_action(one_joint.Action())
    pos = robot.get_observation(t).position
    #print("\r%6.3f %6.3f %6.3f" % (pos[0], pos[1], pos[2]), end="")
    print(pos)
