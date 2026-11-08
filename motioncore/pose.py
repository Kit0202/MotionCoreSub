class PoseManager:

    def __init__(self, robot):
        self.robot = robot

    def stand(self):
        """Reset the robot to the configured stable standing keyframe."""
        self.robot.reset_to_keyframe("stand")

    def raise_left_arm(self):
        """Raise the left arm."""
        self.robot.move_joint(
            "left_shoulder_pitch_joint",
            0.5
        )

    def raise_right_arm(self):
        """Raise the right arm."""
        self.robot.move_joint(
            "right_shoulder_pitch_joint",
            0.5
        )

    def t_pose(self):
        """Simple T-pose."""

        self.robot.move_joint("left_shoulder_roll_joint", 1.57)
        self.robot.move_joint("right_shoulder_roll_joint", -1.57)