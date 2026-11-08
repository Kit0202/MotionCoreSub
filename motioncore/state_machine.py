import time

class RobotStateMachine:
    """
    State machine for controlling robot behavior.
    Uses a state registry to map state names to motion generators.
    """

    def __init__(self, robot):
        self.robot = robot
        self.current_state = "IDLE"
        self.current_motion = None
        self.start_time = None

        self.state_registry = {
            "STAND": self._stand,
            "WAVE_LEFT_HAND": self._wave_left_hand,
            "WAVE_RIGHT_HAND": self._wave_right_hand,
            "DOUBLE_WAVE": self._double_wave,
            "DEMO_WAVE": self._demo_wave,
            "DEMO_BOTH_WAVE": self._demo_both_wave,
            "SALUTE_RIGHT": self._salute_right,
            "SALUTE_LEFT": self._salute_left,
            "DEMO_SALUTE": self._demo_salute,
            "PRESENTATION_DEMO": self._presentation_demo,
            "MARCH_IN_PLACE": self._march_in_place,
            "WEIGHT_SHIFT_TEST": self._weight_shift_test,
            "KNEE_BEND_TEST": self._knee_bend_test,
            "TINY_FOOT_LIFT_TEST": self._tiny_foot_lift_test,
            "MARCH_IN_PLACE_V2": self._march_in_place_v2,
            "FORWARD_STEP_TEST": self._forward_step_test,
            "SLOW_WALK_V1": self._slow_walk_v1,
            "SLOW_WALK_V2": self._slow_walk_v2,
            "ARM_SWING_TEST": self._arm_swing_test,
            "SLOW_WALK_V2_SAFE": self._slow_walk_v2_safe,
            "SLOW_FORWARD_WALK_V1": self._slow_forward_walk_v1,
        }

    def set_state(self, state_name):
        """
        Change robot behavior state.
        """

        if self.current_state != "IDLE":
            print(f"Robot is busy. Current state: {self.current_state}")
            return

        if state_name not in self.state_registry:
            raise ValueError(f"Unknown state: {state_name}")

        self.current_state = state_name
        self.current_motion = self.state_registry[state_name]()
        
        self.start_time = time.perf_counter()
        self.robot.logger.log("STATE_STARTED", state_name)

        print(f"State started: {state_name}")

    def update(self):
        """
        Update the current state.
        This should be called every simulation step.
        """

        if self.current_motion is None:
            return

        try:
            next(self.current_motion)

        except StopIteration:
            duration = None

            if self.start_time is not None:
                duration = round(time.perf_counter() - self.start_time, 3)

            self.robot.logger.log(
                "STATE_COMPLETED",
                self.current_state,
                duration
            )

            print(f"State completed: {self.current_state}")

            self.current_motion = None
            self.current_state = "IDLE"
            self.start_time = None

    def cancel(self, reason="cancelled"):
        """Cancel the active generator without issuing another motion command."""
        if self.current_motion is None:
            self.current_state = "IDLE"
            self.start_time = None
            return False

        cancelled_state = self.current_state
        self.current_motion = None
        self.current_state = "IDLE"
        self.start_time = None
        self.robot.logger.log("STATE_CANCELLED", cancelled_state)
        print(f"State cancelled: {cancelled_state} ({reason})")
        return True

    def is_idle(self):
        return self.current_state == "IDLE"

    def get_available_states(self):
        return list(self.state_registry.keys())

    # -------------------------
    # State definitions
    # -------------------------

    def _stand(self):
        return self.robot.motion.stand()

    def _wave_left_hand(self):
        return self.robot.motion.wave_left_hand()

    def _wave_right_hand(self):
        return self.robot.motion.wave_right_hand()

    def _double_wave(self):
        return self.robot.sequencer.run([
            self.robot.motion.wave_left_hand(),
            self.robot.sequencer.wait(200),
            self.robot.motion.wave_left_hand()
        ])

    def _demo_wave(self):
        return self.robot.sequencer.run([
            self.robot.motion.stand(),
            self.robot.sequencer.wait(100),
            self.robot.motion.wave_left_hand(),
            self.robot.sequencer.wait(200),
            self.robot.motion.wave_left_hand()
        ])

    def _demo_both_wave(self):
        return self.robot.sequencer.run([
            self.robot.motion.stand(),
            self.robot.sequencer.wait(100),
            self.robot.motion.wave_left_hand(),
            self.robot.sequencer.wait(150),
            self.robot.motion.wave_right_hand(),
            self.robot.sequencer.wait(150),
            self.robot.motion.wave_left_hand(),
            self.robot.sequencer.wait(150),
            self.robot.motion.wave_right_hand()
        ])

    def _salute_right(self):
        return self.robot.motion.salute_right()

    def _salute_left(self):
        return self.robot.motion.salute_left()

    def _demo_salute(self):
        return self.robot.sequencer.run([
            self.robot.motion.stand(),
            self.robot.sequencer.wait(100),
            self.robot.motion.salute_right(),
            self.robot.sequencer.wait(150),
            self.robot.motion.wave_left_hand()
        ])
        
    def _presentation_demo(self):
        return self.robot.sequencer.run([
            self.robot.motion.stand(),
            self.robot.sequencer.wait(100),

            self.robot.motion.salute_right(),
            self.robot.sequencer.wait(150),

            self.robot.motion.wave_left_hand(),
            self.robot.sequencer.wait(150),

            self.robot.motion.wave_right_hand(),
            self.robot.sequencer.wait(150),

            self.robot.motion.wave_left_hand(),
            self.robot.sequencer.wait(100),

            self.robot.motion.wave_right_hand()
        ])
        
    def _weight_shift_test(self):
        return self.robot.motion.weight_shift_test()
    
    def _knee_bend_test(self):
        return self.robot.motion.knee_bend_test() 
    
    def _tiny_foot_lift_test(self):
        return self.robot.motion.tiny_foot_lift_test()   
        
    def _march_in_place(self):
        return self.robot.motion.march_in_place()
    
    def _march_in_place_v2(self):
        return self.robot.motion.march_in_place_v2()
    
    def _forward_step_test(self):
        return self.robot.motion.forward_step_test()
    
    def _slow_walk_v1(self):
        return self.robot.motion.slow_walk_v1()
    
    def _slow_walk_v2(self):
        return self.robot.motion.slow_walk_v2()
    
    def _arm_swing_test(self):
        return self.robot.motion.arm_swing_test()
    
    def _slow_walk_v2_safe(self):
        return self.robot.motion.slow_walk_v2_safe()
    
    def _slow_forward_walk_v1(self):
        return self.robot.motion.slow_forward_walk_v1()