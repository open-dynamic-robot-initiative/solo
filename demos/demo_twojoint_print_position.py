#!/usr/bin/env python3
"""Send zero-torque commands to Finger robot and print joint positions."""
import numpy as np

from robot_interfaces import two_joint
import blmc_robots

home_offset = np.array([2.256, 2.2209])
robot_data = two_joint.Data()
robot_backend = blmc_robots.create_two_joint_backend("can6",
						     home_offset,
                                                     robot_data)

robot = two_joint.Frontend(robot_data)

robot_backend.initialize()

while True:
    t = robot.append_desired_action(two_joint.Action())
    pos = robot.get_observation(t).position
    #print("\r%6.3f %6.3f %6.3f" % (pos[0], pos[1], pos[2]), end="")
    print(pos)
