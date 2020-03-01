import numpy as np
from blmc_robots import Solo8

np.set_printoptions(4, suppress=True, linewidth=120)

import signal
import time

pressed_ctrl_c = False


def handler(signum, frame):
    global pressed_ctrl_c
    pressed_ctrl_c = True

signal.signal(signal.SIGINT, handler)


###

abs_enc_indices = np.array([
    0.48793072, -0.39736822,  0.2617748 , -0.37641401,
    -0.46944608, 0.64938238, -0.51563708,  0.41886759
])

zero_pos = np.array([
  0.5, -0.5, 0.5, -0.5,
  -0.5, 0.5, -0.5, -0.5
])

###

pos = np.zeros(8)
vel = np.zeros(8)
enc = np.zeros(8)
tau = np.zeros(8)

pd_time = 0.
target_time = 0.5
target_angle = 1.0
target_pos = np.array([1., -2., 1., -2., -1., 2., -1., 2.])
target_pos_final = np.array([1., -2., 1., -2., -1., 2., -1., 2.])
target_real = np.zeros_like(target_pos)

t = 0.
dt = 0.001

robot = Solo8()
robot.initialize("eno1")

P = 1.
D = 0.01

T = 10000
log_pos = np.zeros((T, 8))
log_enc = np.zeros((T, 8))


class GotoPdController(object):
    def __init__(self, P, D, pos_from, pos_to, duration):
        self.P = P
        self.D = D
        self.pos_from = pos_from
        self.pos_to = pos_to
        self.pos_del = pos_to - pos_from
        self.duration = duration
        self.tau = np.zeros(8)

    def run(self, rel_t, pos, vel, enc):
        target = self.pos_from + min(rel_t / self.duration, 1.) * self.pos_del
        self.tau[:] = self.P * (target - pos) - self.D * vel
        return self.tau

class SetZeroPos(object):
    def __init__(self):
        self.first_call = True

    def run(self, rel_t, pos, vel, enc):
        if self.first_call:
            self.first_call = False

            robot.set_zero_angles(enc - abs_enc_indices - zero_pos)

controllers = [
    {'from': 0, 'till': 1., 'ctrl': GotoPdController(P, D, np.zeros(8), target_pos, 1)},
    {'from': 1, 'till': 2., 'ctrl': GotoPdController(P, D, target_pos, np.zeros(8), 1)}
    {'from': 2, 'till': 2.05, 'ctrl': SetZeroPos()}
    {'from': 2.05, 'till': T/dt, 'ctrl': GotoPdController(P, D, zero_pos, np.zeros(8), 2)}
]

it = 0
for it in range(T):
    # Acquire and read sensor values.
    robot.acquire_sensors()
    robot.fill_joint_positions(pos)
    robot.fill_joint_velocities(vel)
    robot.fill_joint_encoder_indices(enc)

    log_pos[it] = pos
    log_enc[it] = enc

    tau[:] = 0.
    for ctrl_info in controllers:
        if t < ctrl_info['till']:
            tau = ctrl_info['ctrl'].run(t - ctrl_info['from'], pos, vel, enc)
            break

    robot.send_target_joint_torque(tau)

    t += dt
    time.sleep(dt)

np.savetxt('log_pos.out', log_pos, fmt='%.3f')
np.savetxt('log_enc.out', log_enc, fmt='%.3f')

print('Enc: ', enc)
print('Pos: ', pos)
print("Done")

