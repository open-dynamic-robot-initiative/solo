"""Interface between Bullet and the Dynamic Graph for solo12 """


from robot_properties_solo.solo8wrapper import Solo8Robot, Solo8Config
from solo.dg_bullet_solo import DgBulletSoloBaseRobot


class QuadrupedBulletRobot(DgBulletSoloBaseRobot):
    def __init__(
        self,
        use_fixed_base=False,
        record_video=False,
        init_sliders_pose=4 * [0.5],
    ):

        super(QuadrupedBulletRobot, self).__init__(
            Solo8Robot,
            Solo8Config,
            use_fixed_base,
            record_video,
            init_sliders_pose,
        )

        self.q0[0] = 0.2
        self.q0[1] = 0.0
        self.q0[2] = 0.22
        self.q0[6] = 1.0
        self.q0[7] = 0.8
        self.q0[8] = -1.6
        self.q0[9] = 0.8
        self.q0[10] = -1.6
        self.q0[11] = -0.8
        self.q0[12] = 1.6
        self.q0[13] = -0.8
        self.q0[14] = 1.6

        # Sync the current robot state to the graph input signals.
        self._sim2signal()


def get_robot(
    use_fixed_base=False,
    record_video=False,
    init_sliders_pose=4 * [0.5],
    with_gui=True,
):
    return QuadrupedBulletRobot(
        use_fixed_base, record_video, init_sliders_pose
    )


# Alias to new solo8 name.
Solo8BulletRobot = QuadrupedBulletRobot
get_solo8_robot = get_robot
