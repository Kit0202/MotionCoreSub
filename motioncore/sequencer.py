class MotionSequencer:
    """
    Runs multiple motion generators in sequence.
    """

    def __init__(self, robot):
        self.robot = robot

    def wait(self, steps=200):
        """Pause motion for a number of simulation steps."""
        for _ in range(steps):
            yield

    def run(self, motions):
        """
        Run a list of motion generators one by one.
        """

        for motion in motions:
            yield from motion