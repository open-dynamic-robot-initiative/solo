from os import path
import numpy as np
import time
import pinocchio
import pybullet
from pinocchio.utils import zero
from matplotlib import pyplot as plt

from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph_manager.dynamic_graph.device import Device
import dynamic_graph_manager.robot
from bullet_utils.env import BulletEnvWithGround


class DgBulletSoloBaseRobot(dynamic_graph_manager.robot.Robot):
    """
    Base implementation for solo8 and solo12 robot.
    """

    def __init__(
        self,
        SoloRobotClass,
        SoloConfigClass,
        use_fixed_base=False,
        record_video=False,
        init_sliders_pose=4 * [0.5],
    ):
        self.SoloRobotClass = SoloRobotClass
        self.SoloConfigClass = SoloConfigClass
        self.record_video = record_video

        # create the robot configuration.
        self.config = SoloConfigClass()

        # Create the bullet environment.
        self.bullet_env = BulletEnvWithGround()

        robotStartPos = [0.0, 0, 0.7]
        robotStartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])

        # Create the robot and add it to the env.
        self.robot_bullet = SoloRobotClass(
            pos=robotStartPos,
            orn=robotStartOrientation,
            useFixedBase=use_fixed_base,
            init_sliders_pose=init_sliders_pose,
        )

        self.robot_bullet = self.bullet_env.add_robot(self.robot_bullet)

        self.q0 = zero(self.robot_bullet.pin_robot.nq)

        # Initialize the device.
        self.device = Device(self.config.robot_name)
        assert path.exists(self.config.dgm_yaml_path)
        self.device.initialize(self.config.dgm_yaml_path)

        # Create signals for the base.
        self.signal_base_pos_ = VectorConstant("bullet_quadruped_base_pos")
        self.signal_base_vel_ = VectorConstant("bullet_quadruped_base_vel")
        self.signal_base_vel_world_ = VectorConstant(
            "bullet_quadruped_base_vel_world"
        )
        self.signal_base_pos_.sout.value = np.hstack(
            [robotStartPos, robotStartOrientation]
        )
        self.signal_base_vel_.sout.value = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        self.signal_base_vel_world_.sout.value = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )

        # Initialize signals that are not filled in sim2signals.
        if self.device.hasSignal("contact_sensors"):
            self.device.contact_sensors.value = np.array(4 * [0.0])

        # Sync the current robot state to the graph input signals.
        self._sim2signal()

        super(DgBulletSoloBaseRobot, self).__init__(
            self.config.robot_name, self.device
        )

    def base_signals(self):
        return self.signal_base_pos_.sout, self.signal_base_vel_.sout

    def _sim2signal(self):
        """Reads the state from the simulator and fills
        the corresponding signals."""

        # TODO: Handle the following signals:
        # - joint_target_torques
        # - joint_torques

        q, dq = self.robot_bullet.get_state()

        device = self.device
        device.joint_positions.value = q[7:]
        device.joint_velocities.value = dq[6:]

        self.signal_base_pos_.sout.value = q[0:7]
        self.signal_base_vel_.sout.value = dq[0:6]
        self.signal_base_vel_world_.sout.value = (
            self.robot_bullet.get_base_velocity_world()
        )

        device.slider_positions.value = np.array(
            [
                self.robot_bullet.get_slider_position("a"),
                self.robot_bullet.get_slider_position("b"),
                self.robot_bullet.get_slider_position("c"),
                self.robot_bullet.get_slider_position("d"),
            ]
        )

    def run(self, steps=1, sleep=False):
        """ Get input from Device, Step the simulation, feed the Device. """

        for _ in range(steps):
            self.device.execute_graph()
            self.robot_bullet.send_joint_command(
                np.matrix(self.device.ctrl_joint_torques.value).T
            )
            self.bullet_env.step(sleep=sleep)
            self._sim2signal()

    def reset_state(self, q, dq):
        """Sets the bullet simulator and the signals to
        the provided state values."""
        self.robot_bullet.reset_state(q, dq)
        self._sim2signal()

    def add_ros_and_trace(
        self, client_name, signal_name, topic_name=None, topic_type=None
    ):
        ## for vicon entity
        self.signal_name = signal_name

    def start_video_recording(self, file_name):
        if self.record_video:
            self.bullet_env.start_video_recording(file_name)

    def stop_video_recording(self):
        if self.record_video:
            self.bullet_env.stop_video_recording()
