#!/usr/bin/env python3
"""Simple demo moving two joints back and forth."""
import os
import numpy as np
import copy
import rospkg

from robot_interfaces import two_joint
import blmc_robots

import time


def main():

    # Configuration
    # ========================================

    # Limit of the range in which the joint can move (i.e. should be a little
    # bit before hitting the end stop).
    position_limit = 2.7

    # Gains for the position controller
    kp = 5
    kd = 0.04

    # ========================================

    # load the default config file
    config_file_path = os.path.join(
        rospkg.RosPack().get_path("blmc_robots"), "config", "twojoint.yml")

    robot_data = two_joint.Data()
    robot_backend = blmc_robots.create_two_joint_backend(robot_data,
                                                         config_file_path)
    robot = two_joint.Frontend(robot_data)


    N_JOINTS = 2

    def go_to(goal_position, steps, hold, verbose=False):
        """Move with linear position profile.

        Move from the current position to the goal_position using a linear
        position profile.  Then hold the robot at the goal position for the
        specified "hold" time.

        :param goal_position: Goal position of the robot [joint radian].
        :type goal_position: array-like, type float, length = number of joints.
        :param steps: Number of steps to take between start and goal position.
            Determines the velocity of the movement.
        :type steps: int
        :param hold: Number of steps to hold the motor after reaching the goal
            position.
        :type hold: int
        """
        desired_torque = np.zeros(N_JOINTS)

        action = two_joint.Action(torque=desired_torque)
        t = robot.append_desired_action(action)
        # Use current position as start position.
        # Copy the current position, so value in the variable can be modified
        # later.

        desired_step_position = copy.copy(robot.get_observation(t).position)

        stepsize = (goal_position - desired_step_position) / steps

        # move from start to goal
        for step in range(steps):
            desired_step_position += stepsize
            action = two_joint.Action(torque=desired_torque)
            # print("pass1")
            t = robot.append_desired_action(action)
            # print("pass2")
            position_error = (desired_step_position -
                              robot.get_observation(t).position)
            desired_torque = (kp * position_error -
                              kd * robot.get_observation(t).velocity)*1.4
            # time.sleep(0.01)
            if verbose and step % 50 == 0 and step < steps:
                print_on_terminal(desired_torque, t)


        # hold at goal position
        for step in range(hold):
            action = two_joint.Action(torque=desired_torque)
            t = robot.append_desired_action(action)
            position_error = goal_position - robot.get_observation(t).position
            desired_torque = (kp * position_error -
                              kd * robot.get_observation(t).velocity)

    def print_on_terminal(desired_torque, t):
        try:
            print("DT: {}, T: {}".format(desired_torque, robot.get_observation(t).torque))
            # print("P: ", robot.get_observation(t).position)
            # print("V: ", robot.get_observation(t).velocity)
            # print("T: ", robot.get_observation(t).torque)
        except:
            print("Error")
            pass


    def go_to_zero(steps, hold):
        go_to(np.zeros(N_JOINTS), steps, hold)


    robot_backend.initialize()
    print("homing finished")
    go_to_zero(1000, 2000)
    # print("start")
    goal_position = 1 #position_limit
    while True:
        goal_position *= -1
        # move to goal position within 2000 ms and wait there for 100 ms
        go_to(goal_position, 100, 100)


if __name__ == "__main__":
    main()
