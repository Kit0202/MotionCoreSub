from motioncore.pose import PoseManager


class FakeRobot:
    def __init__(self):
        self.commands = []
        self.keyframes = []

    def move_joint(self, name, value):
        self.commands.append((name, value))

    def reset_to_keyframe(self, name):
        self.keyframes.append(name)


def test_stand_uses_stable_keyframe():
    robot = FakeRobot()
    PoseManager(robot).stand()
    assert robot.keyframes == ["stand"]


def test_t_pose_commands_both_shoulders():
    robot = FakeRobot()
    PoseManager(robot).t_pose()
    assert ("left_shoulder_roll_joint", 1.57) in robot.commands
    assert ("right_shoulder_roll_joint", -1.57) in robot.commands
