import time
import numpy as np
from blmc_robots import SingleLeg

np.set_printoptions(suppress=True, precision=3)

P = 1.0
D = 0.1

# Connect to the leg.
leg = SingleLeg()
leg.initialize()
leg.disable_can_recv_timeout()

# Simple PD controller setting the joint to the desired slider position.
try:
    while True:
        leg.acquire_sensors()
        des_pos = leg.get_slider_positions() - 0.5
        pos = leg.get_joint_positions()
        vel = leg.get_joint_velocities()
         
        tau = P * (des_pos - pos) - D * vel
                
        leg.send_target_joint_torque(tau)
        time.sleep(0.001)
except KeyboardInterrupt:
    print('interrupted!')

# Need to send zero torques. Otherwise the safety mode on the card kicks in
# and the card goes into error mode.
leg.send_target_joint_torque(np.array([0., 0.]))

