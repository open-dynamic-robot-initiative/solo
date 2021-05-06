
#### Viconclient object for center of mass control
class ViconClientEntityBullet(object):
    def __init__(self, clientName):
        self.name = clientName
        self.clientName = clientName

    def connect_to_vicon(self, host_name):
        self.host_name = host_name

    def displaySignals(self):
        print("signals are :")

    def add_object_to_track(self, name):
        pass

    def robot_wrapper(self, robot_wrapper, robot_vicon_name):
        self.robot = robot_wrapper
        self.robot_vicon_name = robot_vicon_name

    def signal(self, signal_name):
        if signal_name == self.robot_vicon_name + "_position":
            value = self.robot.signal_base_pos_.sout
            print(value)
        elif signal_name == self.robot_vicon_name + "_velocity_body":
		        value = self.robot.signal_base_vel_.sout
        elif signal_name == self.robot_vicon_name + "_velocity_world":
		        value = self.robot.signal_base_vel_world_.sout
        else:
            raise ValueError('Signal not defined: ' + signal_name)

        return value


