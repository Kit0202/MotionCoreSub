class MotionManager:

    def __init__(self, robot):
        self.robot = robot
        self.walk_speed = 0.5
        self.active_walk = None

    def stand(self, keyframe_name="stand"):
        """Reset robot to a stable standing keyframe."""

        self.robot.reset_to_keyframe(keyframe_name)
        yield from self.robot.sequencer.wait(100)

    def raise_arm(self, side):
        """
        Raise an arm.

        side: "left" or "right"
        """

        if side not in ["left", "right"]:
            raise ValueError("side must be 'left' or 'right'")

        config = self.robot.config["raise_arm"]
        side_config = config[side]

        targets = {
            f"{side}_shoulder_pitch_joint": side_config["shoulder_pitch"],
            f"{side}_shoulder_roll_joint": side_config["shoulder_roll"],
            f"{side}_elbow_joint": side_config["elbow"]
        }

        yield from self.robot.trajectory.multi_joint_linear(
            targets,
            steps=config["steps"]
        )

    def lower_arm(self, side):
        """
        Lower an arm.

        side: "left" or "right"
        """

        if side not in ["left", "right"]:
            raise ValueError("side must be 'left' or 'right'")

        targets = {
            f"{side}_shoulder_pitch_joint": 0.0,
            f"{side}_shoulder_roll_joint": 0.0,
            f"{side}_elbow_joint": 0.0,
            f"{side}_wrist_yaw_joint": 0.0
        }

        yield from self.robot.trajectory.multi_joint_linear(
            targets,
            steps=200
        )

    def wave_hand(self, side):
        """
        Wave one hand.

        side: "left" or "right"
        """

        if side not in ["left", "right"]:
            raise ValueError("side must be 'left' or 'right'")

        config = self.robot.config["wave"]

        yield from self.raise_arm(side)

        wrist_joint = f"{side}_wrist_yaw_joint"

        for _ in range(config["repetitions"]):
            yield from self.robot.trajectory.linear(
                wrist_joint,
                config["wrist_yaw_high"],
                steps=config["steps"]
            )

            yield from self.robot.trajectory.linear(
                wrist_joint,
                config["wrist_yaw_low"],
                steps=config["steps"]
            )

        yield from self.robot.trajectory.linear(
            wrist_joint,
            0.0,
            steps=config["steps"]
        )

        yield from self.lower_arm(side)

    # Wrapper methods for readability

    def raise_left_arm(self):
        yield from self.raise_arm("left")

    def raise_right_arm(self):
        yield from self.raise_arm("right")

    def lower_left_arm(self):
        yield from self.lower_arm("left")

    def lower_right_arm(self):
        yield from self.lower_arm("right")

    def wave_left_hand(self):
        yield from self.wave_hand("left")

    def wave_right_hand(self):
        yield from self.wave_hand("right")
        
    def salute(self, side="right"):
        """
        Perform a simple salute motion.

        side: "left" or "right"
        """

        if side not in ["left", "right"]:
            raise ValueError("side must be 'left' or 'right'")

        config = self.robot.config["salute"]
        side_config = config[side]

        targets = {
            f"{side}_shoulder_pitch_joint": side_config["shoulder_pitch"],
            f"{side}_shoulder_roll_joint": side_config["shoulder_roll"],
            f"{side}_elbow_joint": side_config["elbow"],
            f"{side}_wrist_yaw_joint": side_config["wrist_yaw"]
        }

        yield from self.robot.trajectory.multi_joint_linear(
            targets,
            steps=config["steps"]
        )

        yield from self.robot.sequencer.wait(config["hold_steps"])

        yield from self.lower_arm(side)
        
    def salute_right(self):
        yield from self.salute("right")


    def salute_left(self):
        yield from self.salute("left")
        
    def weight_shift_test(self):
        """
        Test simple left-right weight shifting while both feet stay on the ground.
        This is safer than lifting the legs.
        """

        yield from self.stand()

        config = self.robot.config["walk"]

        # Save current standing values
        neutral = {}

        joints = [
            "waist_roll_joint",
            "left_ankle_roll_joint",
            "right_ankle_roll_joint"
        ]

        for joint_name in joints:
            actuator_id = self.robot.actuators[joint_name]["id"]
            neutral[joint_name] = float(self.robot.data.ctrl[actuator_id])

        for cycle in range(3):

            print(f"Weight shift cycle {cycle + 1}: shift left")

            shift_left = {
                "waist_roll_joint": neutral["waist_roll_joint"] + 0.04,
                "left_ankle_roll_joint": neutral["left_ankle_roll_joint"] + 0.03,
                "right_ankle_roll_joint": neutral["right_ankle_roll_joint"] + 0.03
            }

            yield from self.robot.trajectory.multi_joint_linear(
                shift_left,
                steps=300
            )

            yield from self.robot.sequencer.wait(100)

            print(f"Weight shift cycle {cycle + 1}: shift right")

            shift_right = {
                "waist_roll_joint": neutral["waist_roll_joint"] - 0.04,
                "left_ankle_roll_joint": neutral["left_ankle_roll_joint"] - 0.03,
                "right_ankle_roll_joint": neutral["right_ankle_roll_joint"] - 0.03
            }

            yield from self.robot.trajectory.multi_joint_linear(
                shift_right,
                steps=300
            )

            yield from self.robot.sequencer.wait(100)

        print("Return to center")

        center = {
            "waist_roll_joint": neutral["waist_roll_joint"],
            "left_ankle_roll_joint": neutral["left_ankle_roll_joint"],
            "right_ankle_roll_joint": neutral["right_ankle_roll_joint"]
        }

        yield from self.robot.trajectory.multi_joint_linear(
            center,
            steps=300
        )
        
    def knee_bend_test(self):
        """
        Test small knee bending while keeping both feet on the ground.
        This prepares the robot for marching and walking.
        """

        yield from self.stand()

        # Save current standing values
        neutral = {}

        joints = [
            "left_hip_pitch_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "right_hip_pitch_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint"
        ]

        for joint_name in joints:
            actuator_id = self.robot.actuators[joint_name]["id"]
            neutral[joint_name] = float(self.robot.data.ctrl[actuator_id])

        for cycle in range(3):

            print(f"Knee bend cycle {cycle + 1}: bend knees")

            bend_pose = {
                "left_hip_pitch_joint": neutral["left_hip_pitch_joint"] - 0.08,
                "left_knee_joint": neutral["left_knee_joint"] + 0.15,
                "left_ankle_pitch_joint": neutral["left_ankle_pitch_joint"] - 0.06,

                "right_hip_pitch_joint": neutral["right_hip_pitch_joint"] - 0.08,
                "right_knee_joint": neutral["right_knee_joint"] + 0.15,
                "right_ankle_pitch_joint": neutral["right_ankle_pitch_joint"] - 0.06
            }

            yield from self.robot.trajectory.multi_joint_linear(
                bend_pose,
                steps=300
            )

            yield from self.robot.sequencer.wait(100)

            print(f"Knee bend cycle {cycle + 1}: return to stand")

            yield from self.robot.trajectory.multi_joint_linear(
                neutral,
                steps=300
            )

            yield from self.robot.sequencer.wait(100)
            
    def tiny_foot_lift_test(self):
        """
        Test very small foot lifting after weight shifting.
        This is the preparation stage before marching.
        """

        yield from self.stand()

        neutral = {}

        joints = [
            "waist_roll_joint",
            "left_hip_pitch_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint"
        ]

        for joint_name in joints:
            actuator_id = self.robot.actuators[joint_name]["id"]
            neutral[joint_name] = float(self.robot.data.ctrl[actuator_id])

        for cycle in range(3):

            print(f"Foot lift cycle {cycle + 1}: shift weight right")

            shift_right = {
                "waist_roll_joint": neutral["waist_roll_joint"] - 0.03,
                "right_ankle_roll_joint": neutral["right_ankle_roll_joint"] - 0.02
            }

            yield from self.robot.trajectory.multi_joint_linear(
                shift_right,
                steps=300
            )

            yield from self.robot.sequencer.wait(100)

            print(f"Foot lift cycle {cycle + 1}: tiny left foot lift")

            left_lift = {
                "left_hip_pitch_joint": neutral["left_hip_pitch_joint"] - 0.05,
                "left_knee_joint": neutral["left_knee_joint"] + 0.10,
                "left_ankle_pitch_joint": neutral["left_ankle_pitch_joint"] - 0.04
            }

            yield from self.robot.trajectory.multi_joint_linear(
                left_lift,
                steps=300
            )

            yield from self.robot.sequencer.wait(100)

            print(f"Foot lift cycle {cycle + 1}: lower left foot")

            left_down = {
                "left_hip_pitch_joint": neutral["left_hip_pitch_joint"],
                "left_knee_joint": neutral["left_knee_joint"],
                "left_ankle_pitch_joint": neutral["left_ankle_pitch_joint"]
            }

            yield from self.robot.trajectory.multi_joint_linear(
                left_down,
                steps=300
            )

            yield from self.robot.sequencer.wait(100)

            print(f"Foot lift cycle {cycle + 1}: return center")

            center_right = {
                "waist_roll_joint": neutral["waist_roll_joint"],
                "right_ankle_roll_joint": neutral["right_ankle_roll_joint"]
            }

            yield from self.robot.trajectory.multi_joint_linear(
                center_right,
                steps=300
            )

            yield from self.robot.sequencer.wait(100)

            print(f"Foot lift cycle {cycle + 1}: shift weight left")

            shift_left = {
                "waist_roll_joint": neutral["waist_roll_joint"] + 0.03,
                "left_ankle_roll_joint": neutral["left_ankle_roll_joint"] + 0.02
            }

            yield from self.robot.trajectory.multi_joint_linear(
                shift_left,
                steps=300
            )

            yield from self.robot.sequencer.wait(100)

            print(f"Foot lift cycle {cycle + 1}: tiny right foot lift")

            right_lift = {
                "right_hip_pitch_joint": neutral["right_hip_pitch_joint"] - 0.05,
                "right_knee_joint": neutral["right_knee_joint"] + 0.10,
                "right_ankle_pitch_joint": neutral["right_ankle_pitch_joint"] - 0.04
            }

            yield from self.robot.trajectory.multi_joint_linear(
                right_lift,
                steps=300
            )

            yield from self.robot.sequencer.wait(100)

            print(f"Foot lift cycle {cycle + 1}: lower right foot")

            right_down = {
                "right_hip_pitch_joint": neutral["right_hip_pitch_joint"],
                "right_knee_joint": neutral["right_knee_joint"],
                "right_ankle_pitch_joint": neutral["right_ankle_pitch_joint"]
            }

            yield from self.robot.trajectory.multi_joint_linear(
                right_down,
                steps=300
            )

            yield from self.robot.sequencer.wait(100)

            print(f"Foot lift cycle {cycle + 1}: return center")

            center_left = {
                "waist_roll_joint": neutral["waist_roll_joint"],
                "left_ankle_roll_joint": neutral["left_ankle_roll_joint"]
            }

            yield from self.robot.trajectory.multi_joint_linear(
                center_left,
                steps=300
            )

            yield from self.robot.sequencer.wait(100)
        
    def march_in_place(self):
        """
        Safer basic marching motion with simple weight shifting.
        This is still open-loop and may need tuning.
        """

        print("Starting safer march in place motion")

        config = self.robot.config["walk"]

        # Start from stable keyframe
        yield from self.stand()

        # Save the current standing control values as neutral
        neutral = {}

        leg_joints = [
            "left_hip_pitch_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "waist_roll_joint"
        ]

        for joint_name in leg_joints:
            actuator_id = self.robot.actuators[joint_name]["id"]
            neutral[joint_name] = float(self.robot.data.ctrl[actuator_id])

        for cycle in range(config["cycles"]):

            print(f"March cycle {cycle + 1}: shift weight right")

            # Shift weight to right side before lifting left leg
            shift_right = {
                "waist_roll_joint": config["weight_shift"]["waist_roll_right"],
                "right_ankle_roll_joint": config["weight_shift"]["right_ankle_roll"]
            }

            yield from self.robot.trajectory.multi_joint_linear(
                shift_right,
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

            print(f"March cycle {cycle + 1}: lift left leg")

            left_lift = {
                "left_hip_pitch_joint": neutral["left_hip_pitch_joint"] + config["left_lift"]["left_hip_pitch"],
                "left_knee_joint": neutral["left_knee_joint"] + config["left_lift"]["left_knee"],
                "left_ankle_pitch_joint": neutral["left_ankle_pitch_joint"] + config["left_lift"]["left_ankle_pitch"]
            }

            yield from self.robot.trajectory.multi_joint_linear(
                left_lift,
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

            print(f"March cycle {cycle + 1}: lower left leg")

            left_neutral = {
                "left_hip_pitch_joint": neutral["left_hip_pitch_joint"],
                "left_knee_joint": neutral["left_knee_joint"],
                "left_ankle_pitch_joint": neutral["left_ankle_pitch_joint"]
            }

            yield from self.robot.trajectory.multi_joint_linear(
                left_neutral,
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

            print(f"March cycle {cycle + 1}: return center")

            center = {
                "waist_roll_joint": neutral["waist_roll_joint"],
                "right_ankle_roll_joint": neutral["right_ankle_roll_joint"]
            }

            yield from self.robot.trajectory.multi_joint_linear(
                center,
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

            print(f"March cycle {cycle + 1}: shift weight left")

            # Shift weight to left side before lifting right leg
            shift_left = {
                "waist_roll_joint": config["weight_shift"]["waist_roll_left"],
                "left_ankle_roll_joint": config["weight_shift"]["left_ankle_roll"]
            }

            yield from self.robot.trajectory.multi_joint_linear(
                shift_left,
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

            print(f"March cycle {cycle + 1}: lift right leg")

            right_lift = {
                "right_hip_pitch_joint": neutral["right_hip_pitch_joint"] + config["right_lift"]["right_hip_pitch"],
                "right_knee_joint": neutral["right_knee_joint"] + config["right_lift"]["right_knee"],
                "right_ankle_pitch_joint": neutral["right_ankle_pitch_joint"] + config["right_lift"]["right_ankle_pitch"]
            }

            yield from self.robot.trajectory.multi_joint_linear(
                right_lift,
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

            print(f"March cycle {cycle + 1}: lower right leg")

            right_neutral = {
                "right_hip_pitch_joint": neutral["right_hip_pitch_joint"],
                "right_knee_joint": neutral["right_knee_joint"],
                "right_ankle_pitch_joint": neutral["right_ankle_pitch_joint"]
            }

            yield from self.robot.trajectory.multi_joint_linear(
                right_neutral,
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

            print(f"March cycle {cycle + 1}: return center")

            center = {
                "waist_roll_joint": neutral["waist_roll_joint"],
                "left_ankle_roll_joint": neutral["left_ankle_roll_joint"]
            }

            yield from self.robot.trajectory.multi_joint_linear(
                center,
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])
            
    def march_in_place_v2(self):
        """
        Safer marching motion based on stable weight shift and tiny foot lift.
        """

        yield from self.stand()

        neutral = {}

        joints = [
            "waist_roll_joint",
            "left_hip_pitch_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint"
        ]

        for joint_name in joints:
            actuator_id = self.robot.actuators[joint_name]["id"]
            neutral[joint_name] = float(self.robot.data.ctrl[actuator_id])

        for cycle in range(5):

            print(f"March V2 cycle {cycle + 1}: left step")

            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"] - 0.03,
                    "right_ankle_roll_joint": neutral["right_ankle_roll_joint"] - 0.02
                },
                steps=250
            )

            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "left_hip_pitch_joint": neutral["left_hip_pitch_joint"] - 0.05,
                    "left_knee_joint": neutral["left_knee_joint"] + 0.10,
                    "left_ankle_pitch_joint": neutral["left_ankle_pitch_joint"] - 0.04
                },
                steps=250
            )

            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "left_hip_pitch_joint": neutral["left_hip_pitch_joint"],
                    "left_knee_joint": neutral["left_knee_joint"],
                    "left_ankle_pitch_joint": neutral["left_ankle_pitch_joint"]
                },
                steps=250
            )

            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"],
                    "right_ankle_roll_joint": neutral["right_ankle_roll_joint"]
                },
                steps=250
            )

            print(f"March V2 cycle {cycle + 1}: right step")

            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"] + 0.03,
                    "left_ankle_roll_joint": neutral["left_ankle_roll_joint"] + 0.02
                },
                steps=250
            )

            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "right_hip_pitch_joint": neutral["right_hip_pitch_joint"] - 0.05,
                    "right_knee_joint": neutral["right_knee_joint"] + 0.10,
                    "right_ankle_pitch_joint": neutral["right_ankle_pitch_joint"] - 0.04
                },
                steps=250
            )

            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "right_hip_pitch_joint": neutral["right_hip_pitch_joint"],
                    "right_knee_joint": neutral["right_knee_joint"],
                    "right_ankle_pitch_joint": neutral["right_ankle_pitch_joint"]
                },
                steps=250
            )

            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"],
                    "left_ankle_roll_joint": neutral["left_ankle_roll_joint"]
                },
                steps=250
            )
            
    def forward_step_test(self):
        """
        Very small forward step test.
        This is the first stage before real forward walking.
        """

        yield from self.stand()

        neutral = {}

        joints = [
            "waist_roll_joint",

            "left_hip_pitch_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",

            "right_hip_pitch_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint"
        ]

        for joint_name in joints:
            actuator_id = self.robot.actuators[joint_name]["id"]
            neutral[joint_name] = float(self.robot.data.ctrl[actuator_id])

        for cycle in range(3):

            print(f"Forward step cycle {cycle + 1}: shift weight right")

            # Shift weight to right support leg
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"] - 0.03,
                    "right_ankle_roll_joint": neutral["right_ankle_roll_joint"] - 0.02
                },
                steps=300
            )

            print(f"Forward step cycle {cycle + 1}: move left leg forward")

            # Small left forward step
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "left_hip_pitch_joint": neutral["left_hip_pitch_joint"] - 0.10,
                    "left_knee_joint": neutral["left_knee_joint"] + 0.12,
                    "left_ankle_pitch_joint": neutral["left_ankle_pitch_joint"] - 0.05
                },
                steps=300
            )

            yield from self.robot.sequencer.wait(80)

            print(f"Forward step cycle {cycle + 1}: return left leg")

            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "left_hip_pitch_joint": neutral["left_hip_pitch_joint"],
                    "left_knee_joint": neutral["left_knee_joint"],
                    "left_ankle_pitch_joint": neutral["left_ankle_pitch_joint"]
                },
                steps=300
            )

            print(f"Forward step cycle {cycle + 1}: return center")

            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"],
                    "right_ankle_roll_joint": neutral["right_ankle_roll_joint"]
                },
                steps=300
            )

            yield from self.robot.sequencer.wait(80)

            print(f"Forward step cycle {cycle + 1}: shift weight left")

            # Shift weight to left support leg
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"] + 0.03,
                    "left_ankle_roll_joint": neutral["left_ankle_roll_joint"] + 0.02
                },
                steps=300
            )

            print(f"Forward step cycle {cycle + 1}: move right leg forward")

            # Small right forward step
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "right_hip_pitch_joint": neutral["right_hip_pitch_joint"] - 0.10,
                    "right_knee_joint": neutral["right_knee_joint"] + 0.12,
                    "right_ankle_pitch_joint": neutral["right_ankle_pitch_joint"] - 0.05
                },
                steps=300
            )

            yield from self.robot.sequencer.wait(80)

            print(f"Forward step cycle {cycle + 1}: return right leg")

            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "right_hip_pitch_joint": neutral["right_hip_pitch_joint"],
                    "right_knee_joint": neutral["right_knee_joint"],
                    "right_ankle_pitch_joint": neutral["right_ankle_pitch_joint"]
                },
                steps=300
            )

            print(f"Forward step cycle {cycle + 1}: return center")

            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"],
                    "left_ankle_roll_joint": neutral["left_ankle_roll_joint"]
                },
                steps=300
            )

            yield from self.robot.sequencer.wait(80)
            
    def slow_walk_v1(self):
        """
        Conservative slow walking prototype.
        This is an early open-loop walking attempt using:
        - stable standing pose
        - small forward lean
        - weight shifting
        - tiny alternating foot lift
        """

        print("Starting SLOW_WALK_V1")

        yield from self.stand()

        config = self.robot.config["slow_walk"]

        neutral = {}

        joints = [
            "waist_roll_joint",
            "waist_pitch_joint",

            "left_hip_pitch_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",

            "right_hip_pitch_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint"
        ]

        for joint_name in joints:
            actuator_id = self.robot.actuators[joint_name]["id"]
            neutral[joint_name] = float(self.robot.data.ctrl[actuator_id])

        # Small forward lean
        yield from self.robot.trajectory.multi_joint_linear(
            {
                "waist_pitch_joint": neutral["waist_pitch_joint"]
                + config["forward_lean"]["waist_pitch"]
            },
            steps=config["steps"]
        )

        for cycle in range(config["cycles"]):

            print(f"Slow walk cycle {cycle + 1}: left step")

            # Shift weight to right leg
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"]
                    - config["weight_shift"]["waist_roll"],

                    "right_ankle_roll_joint": neutral["right_ankle_roll_joint"]
                    - config["weight_shift"]["ankle_roll"]
                },
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

            # Move left leg slightly forward
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "left_hip_pitch_joint": neutral["left_hip_pitch_joint"]
                    + config["leg_lift"]["hip_pitch"],

                    "left_knee_joint": neutral["left_knee_joint"]
                    + config["leg_lift"]["knee"],

                    "left_ankle_pitch_joint": neutral["left_ankle_pitch_joint"]
                    + config["leg_lift"]["ankle_pitch"]
                },
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

            # Lower left leg
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "left_hip_pitch_joint": neutral["left_hip_pitch_joint"],
                    "left_knee_joint": neutral["left_knee_joint"],
                    "left_ankle_pitch_joint": neutral["left_ankle_pitch_joint"]
                },
                steps=config["steps"]
            )

            # Return center
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"],
                    "right_ankle_roll_joint": neutral["right_ankle_roll_joint"]
                },
                steps=config["steps"]
            )

            print(f"Slow walk cycle {cycle + 1}: right step")

            # Shift weight to left leg
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"]
                    + config["weight_shift"]["waist_roll"],

                    "left_ankle_roll_joint": neutral["left_ankle_roll_joint"]
                    + config["weight_shift"]["ankle_roll"]
                },
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

            # Move right leg slightly forward
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "right_hip_pitch_joint": neutral["right_hip_pitch_joint"]
                    + config["leg_lift"]["hip_pitch"],

                    "right_knee_joint": neutral["right_knee_joint"]
                    + config["leg_lift"]["knee"],

                    "right_ankle_pitch_joint": neutral["right_ankle_pitch_joint"]
                    + config["leg_lift"]["ankle_pitch"]
                },
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

            # Lower right leg
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "right_hip_pitch_joint": neutral["right_hip_pitch_joint"],
                    "right_knee_joint": neutral["right_knee_joint"],
                    "right_ankle_pitch_joint": neutral["right_ankle_pitch_joint"]
                },
                steps=config["steps"]
            )

            # Return center
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"],
                    "left_ankle_roll_joint": neutral["left_ankle_roll_joint"]
                },
                steps=config["steps"]
            )

        # Return full neutral posture
        yield from self.robot.trajectory.multi_joint_linear(
            neutral,
            steps=config["steps"]
        )
        
    def slow_walk_v2(self):
        """
        Slow walk prototype with opposite arm swing.
        This improves visual humanoid walking behavior.
        """

        print("Starting SLOW_WALK_V2 with arm swing")

        yield from self.stand()

        config = self.robot.config["slow_walk"]

        neutral = {}

        joints = [
            "waist_roll_joint",
            "waist_pitch_joint",

            "left_hip_pitch_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",

            "right_hip_pitch_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",

            "left_shoulder_pitch_joint",
            "left_elbow_joint",
            "right_shoulder_pitch_joint",
            "right_elbow_joint"
        ]

        for joint_name in joints:
            actuator_id = self.robot.actuators[joint_name]["id"]
            neutral[joint_name] = float(self.robot.data.ctrl[actuator_id])

        # Small forward lean
        yield from self.robot.trajectory.multi_joint_linear(
            {
                "waist_pitch_joint": neutral["waist_pitch_joint"]
                + config["forward_lean"]["waist_pitch"]
            },
            steps=config["steps"]
        )

        for cycle in range(config["cycles"]):

            print(f"Slow walk V2 cycle {cycle + 1}: left step with right arm swing")

            # Shift weight to right leg
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"]
                    - config["weight_shift"]["waist_roll"],

                    "right_ankle_roll_joint": neutral["right_ankle_roll_joint"]
                    - config["weight_shift"]["ankle_roll"]
                },
                steps=config["steps"]
            )

            # Left leg forward + right arm swing
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "left_hip_pitch_joint": neutral["left_hip_pitch_joint"]
                    + config["leg_lift"]["hip_pitch"],

                    "left_knee_joint": neutral["left_knee_joint"]
                    + config["leg_lift"]["knee"],

                    "left_ankle_pitch_joint": neutral["left_ankle_pitch_joint"]
                    + config["leg_lift"]["ankle_pitch"],

                    "right_shoulder_pitch_joint": neutral["right_shoulder_pitch_joint"]
                    + config["arm_swing"]["shoulder_pitch"],

                    "right_elbow_joint": neutral["right_elbow_joint"]
                    + config["arm_swing"]["elbow"],

                    "left_shoulder_pitch_joint": neutral["left_shoulder_pitch_joint"]
                    - config["arm_swing"]["shoulder_pitch"]
                },
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

            # Return left leg and arms
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "left_hip_pitch_joint": neutral["left_hip_pitch_joint"],
                    "left_knee_joint": neutral["left_knee_joint"],
                    "left_ankle_pitch_joint": neutral["left_ankle_pitch_joint"],

                    "left_shoulder_pitch_joint": neutral["left_shoulder_pitch_joint"],
                    "left_elbow_joint": neutral["left_elbow_joint"],
                    "right_shoulder_pitch_joint": neutral["right_shoulder_pitch_joint"],
                    "right_elbow_joint": neutral["right_elbow_joint"]
                },
                steps=config["steps"]
            )

            # Return center
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"],
                    "right_ankle_roll_joint": neutral["right_ankle_roll_joint"]
                },
                steps=config["steps"]
            )

            print(f"Slow walk V2 cycle {cycle + 1}: right step with left arm swing")

            # Shift weight to left leg
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"]
                    + config["weight_shift"]["waist_roll"],

                    "left_ankle_roll_joint": neutral["left_ankle_roll_joint"]
                    + config["weight_shift"]["ankle_roll"]
                },
                steps=config["steps"]
            )

            # Right leg forward + left arm swing
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "right_hip_pitch_joint": neutral["right_hip_pitch_joint"]
                    + config["leg_lift"]["hip_pitch"],

                    "right_knee_joint": neutral["right_knee_joint"]
                    + config["leg_lift"]["knee"],

                    "right_ankle_pitch_joint": neutral["right_ankle_pitch_joint"]
                    + config["leg_lift"]["ankle_pitch"],

                    "left_shoulder_pitch_joint": neutral["left_shoulder_pitch_joint"]
                    + config["arm_swing"]["shoulder_pitch"],

                    "left_elbow_joint": neutral["left_elbow_joint"]
                    + config["arm_swing"]["elbow"],

                    "right_shoulder_pitch_joint": neutral["right_shoulder_pitch_joint"]
                    - config["arm_swing"]["shoulder_pitch"]
                },
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

            # Return right leg and arms
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "right_hip_pitch_joint": neutral["right_hip_pitch_joint"],
                    "right_knee_joint": neutral["right_knee_joint"],
                    "right_ankle_pitch_joint": neutral["right_ankle_pitch_joint"],

                    "left_shoulder_pitch_joint": neutral["left_shoulder_pitch_joint"],
                    "left_elbow_joint": neutral["left_elbow_joint"],
                    "right_shoulder_pitch_joint": neutral["right_shoulder_pitch_joint"],
                    "right_elbow_joint": neutral["right_elbow_joint"]
                },
                steps=config["steps"]
            )

            # Return center
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"],
                    "left_ankle_roll_joint": neutral["left_ankle_roll_joint"]
                },
                steps=config["steps"]
            )

        # Return full neutral posture
        yield from self.robot.trajectory.multi_joint_linear(
            neutral,
            steps=config["steps"]
        )
        
    def arm_swing_test(self):
        """
        Test arm swing while both feet stay on the ground.
        This checks whether arm motion affects balance.
        """

        yield from self.stand()

        config = self.robot.config["slow_walk"]

        neutral = {}

        joints = [
            "left_shoulder_pitch_joint",
            "left_elbow_joint",
            "right_shoulder_pitch_joint",
            "right_elbow_joint"
        ]

        for joint_name in joints:
            actuator_id = self.robot.actuators[joint_name]["id"]
            neutral[joint_name] = float(self.robot.data.ctrl[actuator_id])

        for cycle in range(4):

            print(f"Arm swing cycle {cycle + 1}: right arm forward")

            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "right_shoulder_pitch_joint": neutral["right_shoulder_pitch_joint"]
                    + config["arm_swing"]["shoulder_pitch"],

                    "right_elbow_joint": neutral["right_elbow_joint"]
                    + config["arm_swing"]["elbow"],

                    "left_shoulder_pitch_joint": neutral["left_shoulder_pitch_joint"]
                    - config["arm_swing"]["shoulder_pitch"]
                },
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

            print(f"Arm swing cycle {cycle + 1}: left arm forward")

            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "left_shoulder_pitch_joint": neutral["left_shoulder_pitch_joint"]
                    + config["arm_swing"]["shoulder_pitch"],

                    "left_elbow_joint": neutral["left_elbow_joint"]
                    + config["arm_swing"]["elbow"],

                    "right_shoulder_pitch_joint": neutral["right_shoulder_pitch_joint"]
                    - config["arm_swing"]["shoulder_pitch"]
                },
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

        yield from self.robot.trajectory.multi_joint_linear(
            neutral,
            steps=config["steps"]
        )
        
    def slow_walk_v2_safe(self):
        """
        Safer slow walk with arm swing.
        Arm swing is applied before each leg movement to reduce instability.
        """

        print("Starting SLOW_WALK_V2_SAFE")

        yield from self.stand()

        config = self.robot.config["slow_walk"]

        neutral = {}

        joints = [
            "waist_roll_joint",
            "waist_pitch_joint",

            "left_hip_pitch_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",

            "right_hip_pitch_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",

            "left_shoulder_pitch_joint",
            "left_elbow_joint",
            "right_shoulder_pitch_joint",
            "right_elbow_joint"
        ]

        for joint_name in joints:
            actuator_id = self.robot.actuators[joint_name]["id"]
            neutral[joint_name] = float(self.robot.data.ctrl[actuator_id])

        for cycle in range(config["cycles"]):

            print(f"Safe walk cycle {cycle + 1}: prepare left step")

            # Right arm forward, left arm back
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "right_shoulder_pitch_joint": neutral["right_shoulder_pitch_joint"] + config["arm_swing"]["shoulder_pitch"],
                    "right_elbow_joint": neutral["right_elbow_joint"] + config["arm_swing"]["elbow"],
                    "left_shoulder_pitch_joint": neutral["left_shoulder_pitch_joint"] - config["arm_swing"]["shoulder_pitch"]
                },
                steps=config["steps"]
            )

            # Shift weight to right leg
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"] - config["weight_shift"]["waist_roll"],
                    "right_ankle_roll_joint": neutral["right_ankle_roll_joint"] - config["weight_shift"]["ankle_roll"]
                },
                steps=config["steps"]
            )

            # Move left leg
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "left_hip_pitch_joint": neutral["left_hip_pitch_joint"] + config["leg_lift"]["hip_pitch"],
                    "left_knee_joint": neutral["left_knee_joint"] + config["leg_lift"]["knee"],
                    "left_ankle_pitch_joint": neutral["left_ankle_pitch_joint"] + config["leg_lift"]["ankle_pitch"]
                },
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

            # Return left leg and center
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "left_hip_pitch_joint": neutral["left_hip_pitch_joint"],
                    "left_knee_joint": neutral["left_knee_joint"],
                    "left_ankle_pitch_joint": neutral["left_ankle_pitch_joint"],
                    "waist_roll_joint": neutral["waist_roll_joint"],
                    "right_ankle_roll_joint": neutral["right_ankle_roll_joint"]
                },
                steps=config["steps"]
            )

            print(f"Safe walk cycle {cycle + 1}: prepare right step")

            # Left arm forward, right arm back
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "left_shoulder_pitch_joint": neutral["left_shoulder_pitch_joint"] + config["arm_swing"]["shoulder_pitch"],
                    "left_elbow_joint": neutral["left_elbow_joint"] + config["arm_swing"]["elbow"],
                    "right_shoulder_pitch_joint": neutral["right_shoulder_pitch_joint"] - config["arm_swing"]["shoulder_pitch"]
                },
                steps=config["steps"]
            )

            # Shift weight to left leg
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "waist_roll_joint": neutral["waist_roll_joint"] + config["weight_shift"]["waist_roll"],
                    "left_ankle_roll_joint": neutral["left_ankle_roll_joint"] + config["weight_shift"]["ankle_roll"]
                },
                steps=config["steps"]
            )

            # Move right leg
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "right_hip_pitch_joint": neutral["right_hip_pitch_joint"] + config["leg_lift"]["hip_pitch"],
                    "right_knee_joint": neutral["right_knee_joint"] + config["leg_lift"]["knee"],
                    "right_ankle_pitch_joint": neutral["right_ankle_pitch_joint"] + config["leg_lift"]["ankle_pitch"]
                },
                steps=config["steps"]
            )

            yield from self.robot.sequencer.wait(config["pause_steps"])

            # Return right leg and center
            yield from self.robot.trajectory.multi_joint_linear(
                {
                    "right_hip_pitch_joint": neutral["right_hip_pitch_joint"],
                    "right_knee_joint": neutral["right_knee_joint"],
                    "right_ankle_pitch_joint": neutral["right_ankle_pitch_joint"],
                    "waist_roll_joint": neutral["waist_roll_joint"],
                    "left_ankle_roll_joint": neutral["left_ankle_roll_joint"]
                },
                steps=config["steps"]
            )

        # Return all tracked joints to neutral
        yield from self.robot.trajectory.multi_joint_linear(
            neutral,
            steps=config["steps"]
        )
        
    def slow_forward_walk_v1(self):
        """Balanced flat-ground walking; legacy state name retained for callers."""
        from motioncore.scripted_walk import ScriptedWalk
        walker = ScriptedWalk(self.robot, self.walk_speed)
        self.active_walk = walker
        print("Starting balanced scripted walk")
        try:
            yield from walker.run()
        finally:
            self.active_walk = None

    def _forward_walk_step(self, swing_side, neutral, config):
        """
        One conservative walking step.

        swing_side: the leg that moves forward.
        stance_side: the leg supporting the body.
        """

        if swing_side == "left":
            stance_side = "right"
            shift_sign = -1
        elif swing_side == "right":
            stance_side = "left"
            shift_sign = 1
        else:
            raise ValueError("swing_side must be 'left' or 'right'")

        # Opposite arm swing
        forward_arm = stance_side
        backward_arm = swing_side

        # 1. Shift weight to stance leg
        yield from self.robot.trajectory.multi_joint_linear(
            {
                "waist_roll_joint": neutral["waist_roll_joint"]
                + shift_sign * config["weight_shift"]["waist_roll"],

                f"{stance_side}_ankle_roll_joint": neutral[f"{stance_side}_ankle_roll_joint"]
                + shift_sign * config["weight_shift"]["ankle_roll"]
            },
            steps=config["steps"]
        )

        yield from self.robot.sequencer.wait(config["pause_steps"])

        # 2. Stance leg push + swing leg forward + small arm swing
        targets = {
            f"{stance_side}_hip_pitch_joint": neutral[f"{stance_side}_hip_pitch_joint"]
            + config["stance_push"]["hip_pitch"],

            f"{stance_side}_ankle_pitch_joint": neutral[f"{stance_side}_ankle_pitch_joint"]
            + config["stance_push"]["ankle_pitch"],

            f"{swing_side}_hip_pitch_joint": neutral[f"{swing_side}_hip_pitch_joint"]
            + config["swing_leg"]["hip_pitch"],

            f"{swing_side}_knee_joint": neutral[f"{swing_side}_knee_joint"]
            + config["swing_leg"]["knee"],

            f"{swing_side}_ankle_pitch_joint": neutral[f"{swing_side}_ankle_pitch_joint"]
            + config["swing_leg"]["ankle_pitch"],

            f"{forward_arm}_shoulder_pitch_joint": neutral[f"{forward_arm}_shoulder_pitch_joint"]
            + config["arm_swing"]["shoulder_pitch"],

            f"{forward_arm}_elbow_joint": neutral[f"{forward_arm}_elbow_joint"]
            + config["arm_swing"]["elbow"],

            f"{backward_arm}_shoulder_pitch_joint": neutral[f"{backward_arm}_shoulder_pitch_joint"]
            - config["arm_swing"]["shoulder_pitch"]
        }

        yield from self.robot.trajectory.multi_joint_linear(
            targets,
            steps=config["steps"]
        )

        yield from self.robot.sequencer.wait(config["pause_steps"])

        # 3. Return step joints to neutral
        return_targets = {
            "waist_roll_joint": neutral["waist_roll_joint"],

            f"{stance_side}_ankle_roll_joint": neutral[f"{stance_side}_ankle_roll_joint"],
            f"{stance_side}_hip_pitch_joint": neutral[f"{stance_side}_hip_pitch_joint"],
            f"{stance_side}_ankle_pitch_joint": neutral[f"{stance_side}_ankle_pitch_joint"],

            f"{swing_side}_hip_pitch_joint": neutral[f"{swing_side}_hip_pitch_joint"],
            f"{swing_side}_knee_joint": neutral[f"{swing_side}_knee_joint"],
            f"{swing_side}_ankle_pitch_joint": neutral[f"{swing_side}_ankle_pitch_joint"],

            f"{forward_arm}_shoulder_pitch_joint": neutral[f"{forward_arm}_shoulder_pitch_joint"],
            f"{forward_arm}_elbow_joint": neutral[f"{forward_arm}_elbow_joint"],
            f"{backward_arm}_shoulder_pitch_joint": neutral[f"{backward_arm}_shoulder_pitch_joint"]
        }

        yield from self.robot.trajectory.multi_joint_linear(
            return_targets,
            steps=config["steps"]
        )

        yield from self.robot.sequencer.wait(config["pause_steps"])