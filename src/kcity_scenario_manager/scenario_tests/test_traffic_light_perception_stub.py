from types import SimpleNamespace
import unittest

import carla

from kcity_scenario_manager.competition_common import TriggerBox, find_ego_vehicle
from kcity_scenario_manager.tools.optional.traffic_light_perception_stub import (
    NO_DETECTION,
    SignalRuntime,
    TrafficLightPerceptionEngine,
    observation_for_carla_state,
    publisher_is_unique,
)


class FakeLight:
    def __init__(self, state="Red"):
        self.state = state
        self.is_alive = True

    def get_state(self):
        return self.state


class FakeActors(list):
    def filter(self, _pattern):
        return self


class FakeWorld:
    def __init__(self, actors):
        self.actors = FakeActors(actors)

    def get_actors(self):
        return self.actors


def runtime(signal_type="straight", actor=None):
    return SignalRuntime(
        key="TEST_TL",
        cfg={},
        trigger=TriggerBox(0, 0, 0, 1, 1, 1, enabled=True),
        signal_type=signal_type,
        actor=actor,
    )


class TrafficLightMappingTests(unittest.TestCase):
    def test_red_yellow_green_use_real_labels_and_names(self):
        red = observation_for_carla_state("Red", "straight")
        yellow = observation_for_carla_state("Yellow", "straight")
        green = observation_for_carla_state("Green", "straight")
        self.assertEqual((red.label, red.class_names), ("1301", ("RED",)))
        self.assertEqual((yellow.label, yellow.class_names), ("1302", ("ORANGE",)))
        self.assertEqual((green.label, green.class_names), ("1300", ("GREEN",)))

    def test_left_green_uses_real_left_class(self):
        left = observation_for_carla_state("Green", "left")
        self.assertEqual((left.label, left.class_names), ("1305", ("LEFT",)))

    def test_unknown_and_off_are_no_detection(self):
        self.assertEqual(observation_for_carla_state("Unknown", "straight"), NO_DETECTION)
        self.assertEqual(observation_for_carla_state("Off", "left"), NO_DETECTION)

    def test_trigger_enter_stay_transition_and_exit(self):
        light = FakeLight("Red")
        engine = TrafficLightPerceptionEngine([runtime(actor=light)])
        outside = carla.Location(x=2, y=0, z=0)
        inside = carla.Location(x=0, y=0, z=0)
        self.assertEqual(engine.sample(outside)[0], NO_DETECTION)
        self.assertEqual(engine.sample(inside)[0].class_names, ("RED",))
        light.state = "Yellow"
        self.assertEqual(engine.sample(inside)[0].class_names, ("ORANGE",))
        light.state = "Green"
        self.assertEqual(engine.sample(inside)[0].class_names, ("GREEN",))
        self.assertEqual(engine.sample(outside)[0], NO_DETECTION)

    def test_missing_light_actor_is_no_detection(self):
        observation, key = TrafficLightPerceptionEngine([runtime()]).sample(
            carla.Location(x=0, y=0, z=0)
        )
        self.assertEqual(observation, NO_DETECTION)
        self.assertEqual(key, "TEST_TL")

    def test_exact_ego_role_is_required(self):
        wrong = SimpleNamespace(attributes={"role_name": "hero"})
        ego = SimpleNamespace(attributes={"role_name": "ego_vehicle"})
        world = FakeWorld([wrong, ego])
        self.assertIs(find_ego_vehicle(world, "ego_vehicle", strict=True), ego)
        self.assertIsNone(find_ego_vehicle(FakeWorld([wrong]), "ego_vehicle", strict=True))

    def test_duplicate_publisher_prevention_contract(self):
        self.assertTrue(publisher_is_unique(1))
        self.assertFalse(publisher_is_unique(0))
        self.assertFalse(publisher_is_unique(2))


if __name__ == "__main__":
    unittest.main()
