import numpy as np


class TrajectoryEngine:

    def __init__(self, robot):
        self.robot = robot

    @staticmethod
    def _validate_steps(steps):
        if not isinstance(steps, int) or steps < 1:
            raise ValueError("steps must be an integer greater than or equal to 1")
        return steps

    def linear(self, joint_name, target, steps=200):
        """Move one joint smoothly to a target position."""
        steps = self._validate_steps(steps)
        actuator = self.robot.actuators[joint_name]
        actuator_id = actuator["id"]
        start = float(self.robot.data.ctrl[actuator_id])

        if steps == 1:
            self.robot.move_joint(joint_name, float(target))
            yield
            return

        for value in np.linspace(start, target, steps):
            self.robot.move_joint(joint_name, float(value))
            yield

    def multi_joint_linear(self, targets, steps=200):
        """Move multiple joints smoothly at the same time."""
        steps = self._validate_steps(steps)
        if not targets:
            return

        starts = {}
        for joint_name in targets:
            actuator = self.robot.actuators[joint_name]
            actuator_id = actuator["id"]
            starts[joint_name] = float(self.robot.data.ctrl[actuator_id])

        if steps == 1:
            for joint_name, target in targets.items():
                self.robot.move_joint(joint_name, float(target))
            yield
            return

        for step in range(steps):
            ratio = step / (steps - 1)
            for joint_name, target in targets.items():
                start = starts[joint_name]
                value = start + (float(target) - start) * ratio
                self.robot.move_joint(joint_name, value)
            yield
