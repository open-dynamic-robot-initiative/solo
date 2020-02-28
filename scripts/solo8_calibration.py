import numpy as np
from blmc_robots.py_solo8 import Solo8

import signal
import time

pressed_ctrl_c = False

def handler(signum, frame):
    global pressed_ctrl_c
    pressed_ctrl_c = True

signal.signal(signal.SIGINT, handler)

pos = np.zeros(8)
vel = np.zeros(8)
enc = np.zeros(8)
tau = np.zeros(8)

pd_time = 0.
target_time = 2.
target_angle = 1.0
target_pos = np.array([1., -2., 1., -2., -1., 2., -1., 2.])

t = 0.
dt = 0.001

robot = Solo8()
robot.initialize("eno1")

while !pressed_ctrl_c:
    # Acquire and read sensor values.
    robot.acquire_sensors()
    robot.fill_joint_positions(pos)
    robot.fill_joint_velocities(vel)
    robot.fill_joint_encoder_indices(enc)

    pd_time = min(t, target_time)



    robot.send_target_joint_torque(tau)

    t += dt
    time.sleep(dt)