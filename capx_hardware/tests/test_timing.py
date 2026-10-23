import threading
import unittest

from fr3_capx.timing import WallProfiler


class Clock:
    def __init__(self):
        self.value = 0.0
    def __call__(self):
        return self.value
    def advance(self, value):
        self.value += value


class TimingTests(unittest.TestCase):
    def test_nested_check_and_motion_do_not_double_count(self):
        clock = Clock()
        profile = WallProfiler(clock=clock)
        with profile.measure("phase", "carry"):
            clock.advance(1)
            with profile.measure("collision_check"):
                clock.advance(3)
            with profile.measure("actuation"):
                clock.advance(2)
        report = profile.report()
        accounting = next(iter(report["thread_accounting"].values()))
        self.assertEqual(accounting["exclusive_category_s"],
                         {"phase": 1.0, "collision_check": 3.0, "actuation": 2.0})
        self.assertEqual(accounting["root_covered_wall_s"], 6)
        self.assertEqual(report["total_wall_s"], 6)

    def test_failed_callback_is_timed_and_exception_kept(self):
        clock = Clock()
        profile = WallProfiler(clock=clock)
        def failure():
            clock.advance(2)
            raise ValueError("Native path rejected")
        with self.assertRaisesRegex(ValueError, "Native path rejected"):
            profile.wrap("collision_check", failure)()
        self.assertEqual(profile.report()["spans"][0]["outcome"], "raised")
        self.assertEqual(profile.report()["spans"][0]["wall_s"], 2)

    def test_simulated_and_commanded_seconds_remain_separate(self):
        clock = Clock()
        profile = WallProfiler(clock=clock)
        profile.record_metric("simulated", 10, unit="s", basis="PhysX simulation step dt sum")
        profile.record_metric("commanded", 7, unit="s", basis="retimed issued array timestamps")
        clock.advance(100)
        report = profile.report()
        self.assertEqual(report["total_wall_s"], 100)
        self.assertEqual(report["metrics"]["simulated"]["value"], 10)
        self.assertEqual(report["thread_accounting"], {})
        with self.assertRaises(ValueError):
            profile.record_metric("commanded", 8, unit="s", basis="replacement")

    def test_separate_threads_are_not_claimed_as_wall_sum(self):
        profile = WallProfiler()
        barrier = threading.Barrier(2)
        def worker():
            with profile.measure("perception"):
                barrier.wait(timeout=1)
        thread = threading.Thread(target=worker)
        thread.start()
        with profile.measure("planning"):
            barrier.wait(timeout=1)
        thread.join(timeout=1)
        report = profile.report()
        self.assertEqual(len(report["thread_accounting"]), 2)
        self.assertFalse(report["concurrent_thread_durations_are_additive_wall_time"])


if __name__ == "__main__":
    unittest.main()
