"""Pure state machine for START/FINISH exit-edge timing in CARLA simulation time."""

from __future__ import annotations

from typing import Optional


class LapTimer:
    def __init__(self, start_trigger, finish_trigger):
        self.start_trigger = start_trigger
        self.finish_trigger = finish_trigger
        self.state = "armed"
        self.start_sim_time_sec: Optional[float] = None
        self.finish_sim_time_sec: Optional[float] = None
        self._prev_start_loc = None
        self._prev_finish_loc = None
        self._prev_sim_time_sec = None

    @property
    def lap_time_sec(self):
        if self.start_sim_time_sec is None or self.finish_sim_time_sec is None:
            return None
        return max(0.0, self.finish_sim_time_sec - self.start_sim_time_sec)

    def observe(self, sim_time_sec, start_loc, finish_loc=None):
        """Return START, FINISH, or None; first sample only arms the timer."""
        if finish_loc is None:
            finish_loc = start_loc
        event = None
        previous_time = self._prev_sim_time_sec
        if previous_time is not None and sim_time_sec >= previous_time:
            if self.state == "armed":
                if self.start_trigger.enabled:
                    fraction = self.start_trigger.exit_crossing_fraction(
                        self._prev_start_loc, start_loc)
                else:
                    fraction = 1.0
                if fraction is not None:
                    self.start_sim_time_sec = previous_time + fraction * (
                        sim_time_sec - previous_time)
                    self.state = "running"
                    event = "START"
            elif self.state == "running" and self.finish_trigger.enabled:
                fraction = self.finish_trigger.exit_crossing_fraction(
                    self._prev_finish_loc, finish_loc)
                if fraction is not None:
                    self.finish_sim_time_sec = previous_time + fraction * (
                        sim_time_sec - previous_time)
                    self.state = "finished"
                    event = "FINISH"
        self._prev_start_loc = start_loc
        self._prev_finish_loc = finish_loc
        self._prev_sim_time_sec = sim_time_sec
        return event
