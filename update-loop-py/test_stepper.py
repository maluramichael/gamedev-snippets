import unittest

from stepper import FixedStepper


class FixedStepperTest(unittest.TestCase):
    def test_no_update_until_a_full_step_has_accumulated(self):
        stepper = FixedStepper(step=0.25, max_frame_time=10.0)
        self.assertEqual(stepper.advance(0.125), 0)
        self.assertEqual(stepper.alpha, 0.5)

    def test_remainder_carries_over_to_the_next_frame(self):
        stepper = FixedStepper(step=0.25, max_frame_time=10.0)
        stepper.advance(0.125)
        self.assertEqual(stepper.advance(0.125), 1)
        self.assertEqual(stepper.alpha, 0.0)

    def test_long_frame_runs_several_updates(self):
        stepper = FixedStepper(step=0.25, max_frame_time=10.0)
        self.assertEqual(stepper.advance(0.75), 3)

    def test_alpha_is_the_fraction_of_the_next_step(self):
        stepper = FixedStepper(step=0.25, max_frame_time=10.0)
        self.assertEqual(stepper.advance(0.375), 1)
        self.assertEqual(stepper.alpha, 0.5)

    def test_frame_time_is_clamped_against_the_spiral_of_death(self):
        stepper = FixedStepper(step=0.25, max_frame_time=0.5)
        self.assertEqual(stepper.advance(10.0), 2)

    def test_one_second_of_small_frames_gives_four_steps(self):
        stepper = FixedStepper(step=0.25, max_frame_time=10.0)
        total = sum(stepper.advance(0.125) for _ in range(8))
        self.assertEqual(total, 4)


if __name__ == "__main__":
    unittest.main()
