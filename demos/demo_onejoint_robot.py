#!/usr/bin/env python3
"""Simple demo moving a single joint back and forth."""
import numpy as np
import copy

import py_finger
import py_real_finger


def main():

    # Configuration
    # ========================================

    # Offset between encoder index and zero-position (in radian).
    # Set this such that the zero position is in the center between left and
    # right end stop.
    home_offset = 2.241742

    # Limit of the range in which the joint can move (i.e. should be a little
    # bit before hitting the end stop).
    position_limit = 2.7

    # Gains for the position controller
    kp = 5
    kd = 0.04


    # ========================================


    finger_data = py_finger.OJData()
    finger_backend = py_real_finger.create_one_joint_backend("can7",
                                                             home_offset,
                                                             finger_data)
    finger = py_finger.OJFrontend(finger_data)


    N_JOINTS = 1

    def go_to(goal_position, steps, hold):
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

        t = finger.append_desired_action(desired_torque)
        # Use current position as start position.
        # Copy the current position, so value in the variable can be modified
        # later.
        desired_step_position = copy.copy(finger.get_observation(t).angle)

        stepsize = (goal_position - desired_step_position) / steps

        # move from start to goal
        for step in range(steps):
            desired_step_position += stepsize
            t = finger.append_desired_action(desired_torque)
            position_error = (desired_step_position -
                              finger.get_observation(t).angle)
            desired_torque = (kp * position_error -
                              kd * finger.get_observation(t).velocity)

        # hold at goal position
        for step in range(hold):
            t = finger.append_desired_action(desired_torque)
            position_error = goal_position - finger.get_observation(t).angle
            desired_torque = (kp * position_error -
                              kd * finger.get_observation(t).velocity)


    def go_to_zero(steps, hold):
        go_to(np.zeros(N_JOINTS), steps, hold)


    finger_backend.initialize()
    print("homing finished")
    go_to_zero(1000, 2000)

    finger.append_desired_action(np.ones(N_JOINTS) * 0.15)

    goal_position = position_limit
    while True:
        goal_position *= -1
        # move to goal position within 2000 ms and wait there for 100 ms
        go_to(goal_position, 2000, 100)



if __name__ == "__main__":
    main()
