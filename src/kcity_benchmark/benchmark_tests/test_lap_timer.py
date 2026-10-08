from types import SimpleNamespace
import unittest

from kcity_benchmark.lap_timer import LapTimer
from kcity_benchmark.common import TriggerBox


def loc(x, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def box(center):
    return TriggerBox(center, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, True, "+x", "center")


class LapTimerTests(unittest.TestCase):
    def test_spawn_inside_start_remains_armed_until_exit_edge(self):
        timer = LapTimer(box(0.0), box(10.0))
        timer.observe(20.0, loc(0.0))
        timer.observe(21.0, loc(0.5))
        self.assertEqual(timer.state, "armed")
        self.assertIsNone(timer.lap_time_sec)
        timer.observe(22.0, loc(1.5))
        self.assertEqual(timer.state, "running")
        self.assertEqual(timer.start_sim_time_sec, 21.5)
        timer.observe(30.0, loc(9.5))
        self.assertEqual(timer.state, "running")
        timer.observe(31.0, loc(11.5))
        self.assertEqual(timer.state, "finished")
        self.assertEqual(timer.finish_sim_time_sec, 30.75)
        self.assertEqual(timer.lap_time_sec, 9.25)

    def test_wrong_edge_and_first_sample_do_not_start(self):
        timer = LapTimer(box(0.0), box(10.0))
        timer.observe(1.0, loc(2.0))
        timer.observe(2.0, loc(0.0))
        timer.observe(3.0, loc(-2.0))
        self.assertEqual(timer.state, "armed")
        timer.observe(4.0, loc(0.0))
        timer.observe(5.0, loc(2.0))
        self.assertEqual(timer.state, "running")

    def test_warmup_and_post_finish_time_are_excluded(self):
        timer = LapTimer(box(0.0), box(10.0))
        timer.observe(100.0, loc(0.0))
        timer.observe(110.0, loc(0.0))
        self.assertIsNone(timer.lap_time_sec)
        self.assertEqual(timer.observe(112.0, loc(2.0)), "START")
        self.assertEqual(timer.start_sim_time_sec, 111.0)
        timer.observe(120.0, loc(9.0))
        self.assertEqual(timer.observe(122.0, loc(13.0)), "FINISH")
        self.assertEqual(timer.finish_sim_time_sec, 121.0)
        self.assertEqual(timer.lap_time_sec, 10.0)
        timer.observe(150.0, loc(20.0))
        self.assertEqual(timer.lap_time_sec, 10.0)
