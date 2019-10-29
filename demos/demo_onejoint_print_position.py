#!/usr/bin/env python3
"""Send zero-torque commands to Finger robot and print joint positions."""
import numpy as np

from robot_interfaces import one_joint
import blmc_robots

home_offset = 2.241742
robot_data = one_joint.Data()
robot_backend = blmc_robots.create_one_joint_backend("can6",
                                                     home_offset,
                                                     robot_data)
robot = one_joint.Frontend(robot_data)


robot_backend.initialize()

# TODO this makes the application hang
#t = finger.get_current_time_index()
#print("t = %d" % t)

while True:
    t = robot.append_desired_action(one_joint.Action())
    pos = robot.get_observation(t).position
    #print("\r%6.3f %6.3f %6.3f" % (pos[0], pos[1], pos[2]), end="")
    print(pos)
