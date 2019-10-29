#!/usr/bin/env python3
"""Simple demo moving a single joint back and forth."""
import numpy as np
import copy

from robot_interfaces import two_joint
import blmc_robots


def main():

    # Configuration
    # ========================================

    # Offset between encoder index and zero-position (in radian).
    # Set this such that the zero position is in the center between left and
    # right end stop.
    home_offset = np.array([2.256, 2.2209])

    # Limit of the range in which the joint can move (i.e. should be a little
    # bit before hitting the end stop).
    position_limit = 2.7

    # Gains for the position controller
    kp = 5
    kd = 0.04


    # ========================================


    robot_data = two_joint.Data()
    robot_backend = blmc_robots.create_two_joint_backend("can6",
							 home_offset,
                                                         robot_data)
    robot = two_joint.Frontend(robot_data)


    N_JOINTS = 2

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
            t = robot.append_desired_action(action)
            position_error = (desired_step_position -
                              robot.get_observation(t).position)
            desired_torque = (kp * position_error -
                              kd * robot.get_observation(t).velocity)

        # hold at goal position
        for step in range(hold):
            action = two_joint.Action(torque=desired_torque)
            t = robot.append_desired_action(action)
            position_error = goal_position - robot.get_observation(t).position
            desired_torque = (kp * position_error -
                              kd * robot.get_observation(t).velocity)


    def go_to_zero(steps, hold):
        go_to(np.zeros(N_JOINTS), steps, hold)


    robot_backend.initialize()
    print("homing finished")
    go_to_zero(1000, 2000)

    goal_position = position_limit
    while True:
        goal_position *= -1
        # move to goal position within 2000 ms and wait there for 100 ms
        go_to(goal_position, 2000, 100)



if __name__ == "__main__":
    main()
