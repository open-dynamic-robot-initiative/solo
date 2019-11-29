#!/usr/bin/env python3
"""Demo script for the TriFinger robot

Moves the TriFinger robot with a hard-coded choreography for show-casing and
testing.
"""
import time
import numpy as np

import robot_interfaces
import blmc_robots


N_JOINTS = 9


def run_choreography(robot):
    """Move the legs in some hard-coded choreography."""

    def perform_step(position):
        t = robot.frontend.append_desired_action(robot.Action(position=position))
        time.sleep(1)

    deg45 = np.pi / 4

    while True:
        perform_step([
            0, -deg45, -deg45,
            0, -deg45, -deg45,
            0, -deg45, -deg45,
        ])

        perform_step([
            0, +deg45, +deg45,
            0, -deg45, -deg45,
            0, -deg45, -deg45,
        ])

        perform_step([
            0, -deg45, -deg45,
            0, +deg45, +deg45,
            0, -deg45, -deg45,
        ])

        perform_step([
            0, -deg45, -deg45,
            0, -deg45, -deg45,
            0, +deg45, +deg45,
        ])

        perform_step([
            0, -deg45, -deg45,
            0, -deg45, -deg45,
            0, -deg45, -deg45,
        ])

        perform_step([
            -deg45, -deg45, -deg45,
            0, -deg45, -deg45,
            0, -deg45, -deg45,
        ])

        perform_step([
            0, -deg45, -deg45,
            -deg45, -deg45, -deg45,
            0, -deg45, -deg45,
        ])

        perform_step([
            0, -deg45, -deg45,
            0, -deg45, -deg45,
            -deg45, -deg45, -deg45,
        ])


def main():
    robot = blmc_robots.Robot(robot_interfaces.trifinger,
                              blmc_robots.create_trifinger_backend,
                              "trifinger.yml")
    robot.initialize()

    # move around
    run_choreography(robot)


if __name__ == "__main__":
    main()

