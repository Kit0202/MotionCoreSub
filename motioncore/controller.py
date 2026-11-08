class JointController:

    def __init__(self, robot):
        self.robot = robot

    def move(self, joint_name, position):

        self.robot.move_joint(joint_name, position)