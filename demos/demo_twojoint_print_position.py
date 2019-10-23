#!/usr/bin/env python3
"""Send zero-torque commands to Finger robot and print joint positions."""
import os
import numpy as np
import rospkg

from robot_interfaces import two_joint
import blmc_robots

def main():
    # load the default config file
    config_file_path = os.path.join(
        rospkg.RosPack().get_path("blmc_robots"), "config", "twojoint.yml")

    robot_data = two_joint.Data()
    robot_backend = blmc_robots.create_two_joint_backend(robot_data,
                                                         config_file_path)

    robot = two_joint.Frontend(robot_data)

    robot_backend.initialize()

    while True:
        t = robot.append_desired_action(two_joint.Action())
        pos = robot.get_observation(t).position
        print(pos)


if __name__ == "__main__":
    main()
