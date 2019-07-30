import time
import numpy as np

import ipdb
from py_real_finger import RealFinger

np.set_printoptions(suppress=True, precision=3)


# Connect to the leg.
finger = RealFinger.create('can0', 'can1')
# finger.pause()
# observation = finger.get_observation(finger.current_time_index())
finger.current_time_index()

ipdb.set_trace()

time.sleep(10)