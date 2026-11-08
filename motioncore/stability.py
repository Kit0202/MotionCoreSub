import math


class StabilityMonitor:
    """
    Monitors robot posture stability using body height and orientation.
    """

    def __init__(
        self,
        robot,
        body_name="pelvis",
        min_height=0.45,
        max_tilt_rad=0.8
    ):
        self.robot = robot
        self.body_name = body_name
        self.min_height = min_height
        self.max_tilt_rad = max_tilt_rad

    def get_roll_pitch(self):
        """
        Convert quaternion to roll and pitch.
        MuJoCo quaternion format: [w, x, y, z]
        """

        q = self.robot.get_body_quaternion(self.body_name)

        w = q[0]
        x = q[1]
        y = q[2]
        z = q[3]

        roll = math.atan2(
            2.0 * (w * x + y * z),
            1.0 - 2.0 * (x * x + y * y)
        )

        pitch_value = 2.0 * (w * y - z * x)
        pitch_value = max(-1.0, min(1.0, pitch_value))

        pitch = math.asin(pitch_value)

        return roll, pitch

    def is_fallen(self):
        height = self.robot.get_body_height(self.body_name)
        roll, pitch = self.get_roll_pitch()

        if height < self.min_height:
            return True

        if abs(roll) > self.max_tilt_rad:
            return True

        if abs(pitch) > self.max_tilt_rad:
            return True

        return False

    def get_stability_status(self):
        height = self.robot.get_body_height(self.body_name)
        roll, pitch = self.get_roll_pitch()

        return {
            "height": round(height, 3),
            "roll": round(roll, 3),
            "pitch": round(pitch, 3),
            "fallen": self.is_fallen()
        }