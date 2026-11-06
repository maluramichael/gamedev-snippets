class FixedStepper:
    """Turns variable frame times into a whole number of fixed logic steps."""

    def __init__(self, step=1 / 60, max_frame_time=0.25):
        self.step = step
        self.max_frame_time = max_frame_time
        self._accumulator = 0.0

    def advance(self, frame_time):
        """Add one frame's time and return how many logic steps to run now."""
        self._accumulator += min(frame_time, self.max_frame_time)
        updates = 0
        while self._accumulator >= self.step:
            self._accumulator -= self.step
            updates += 1
        return updates

    @property
    def alpha(self):
        """How far we are into the next step, 0.0 to 1.0, for interpolated drawing."""
        return self._accumulator / self.step
