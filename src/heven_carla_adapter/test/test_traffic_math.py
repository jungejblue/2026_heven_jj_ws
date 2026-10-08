"""Contract/trigger tests without a running CARLA server or ROS installation."""
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

from heven_carla_adapter.traffic_math import (
    NO_DETECTION, TriggerBox, TrafficSignal, observation_for_state,
    resolve_light, sample_signals, signals_from_config, simulation_stamp,
)


class Actor:
    def __init__(self, actor_id, position, state="Red"):
        self.id, self.position, self.state = actor_id, position, state
        self.is_alive = True

    def get_location(self):
        return SimpleNamespace(x=self.position[0], y=self.position[1], z=self.position[2])

    def get_state(self):
        return self.state


class TrafficContractTests(unittest.TestCase):
    def test_existing_heven_labels_and_unknown_state(self):
        for state, label, classes in (("Red", "1301", ("RED",)),
                                      ("Yellow", "1302", ("ORANGE",)),
                                      ("Green", "1300", ("GREEN",))):
            result = observation_for_state("TrafficLightState." + state, "straight")
            self.assertEqual((result.label, result.class_names, result.confidence, result.detections),
                             (label, classes, 1.0, 1))
        self.assertEqual(observation_for_state("Green", "left").class_names, ("LEFT",))
        self.assertEqual(observation_for_state("Green", "left").label, "1305")
        self.assertEqual(observation_for_state("Off", "straight"), NO_DETECTION)
        with self.assertRaises(ValueError):
            observation_for_state("Green", "right")

    def test_rotated_trigger_uses_half_extents(self):
        box = TriggerBox((22.1804, 236.3491, 1.0), (6.0, 18.0, 1.0), 120.0)
        yaw = math.radians(box.yaw_deg)
        local_to_world = lambda x, y, z: (
            box.center[0] + x * math.cos(yaw) - y * math.sin(yaw),
            box.center[1] + x * math.sin(yaw) + y * math.cos(yaw), z)
        self.assertTrue(box.contains(local_to_world(6, 18, 2)))
        self.assertFalse(box.contains(local_to_world(6.001, 0, 1)))
        self.assertFalse(box.contains(local_to_world(0, 18.001, 1)))
        self.assertFalse(box.contains(local_to_world(0, 0, 2.001)))
        self.assertEqual(TriggerBox(box.center, box.extent, 120, False).contains(box.center), False)
        for point in box.exit_line():
            self.assertAlmostEqual(box.local_coords(point)[1], -18)

    def test_only_active_region_and_live_actor_produce_detection(self):
        actor = Actor(1, (0, 0, 0), "Green")
        signal = TrafficSignal("Q_TL1", {}, TriggerBox((0, 0, 1), (6, 18, 1), 0), "straight", actor)
        self.assertEqual(sample_signals([signal], (0, 0, 1))[0].class_names, ("GREEN",))
        self.assertEqual(sample_signals([signal], (0, 18.01, 1))[0], NO_DETECTION)
        actor.is_alive = False
        self.assertEqual(sample_signals([signal], (0, 0, 1))[0], NO_DETECTION)
        actor.is_alive = True
        self.assertEqual(sample_signals([signal, signal], (0, 0, 1))[0], NO_DETECTION)

    def test_actor_binding_keeps_exact_id_and_location_semantics(self):
        actor = Actor(7, (36.4, 248.3, 0))
        far = Actor(8, (100, 100, 0))
        config = {"location": {"x": 36.4, "y": 248.3, "z": 0}, "match_radius_m": 5}
        self.assertIs(resolve_light([actor, far], config), actor)
        self.assertIs(resolve_light([actor, far], {"actor_id": 8}), far)
        with self.assertRaises(ValueError):
            resolve_light([actor, Actor(9, (36.5, 248.3, 0))], config)
        with self.assertRaises(ValueError):
            resolve_light([far], config)

    def test_simulation_stamp_uses_carla_clock_epoch(self):
        self.assertEqual(simulation_stamp(123.05), (123, 50_000_000))
        self.assertEqual(simulation_stamp(1.9999999996), (2, 0))
        with self.assertRaises(ValueError):
            simulation_stamp(-1)

    def test_packaged_config_exact_accepted_regions(self):
        import yaml
        config_path = (Path(__file__).parents[2] / "kcity_scenario_manager" /
                       "config" / "qualifier.yaml")
        config = yaml.safe_load(config_path.read_text())
        signals = signals_from_config(config)
        self.assertEqual([signal.key for signal in signals], ["Q_TL1", "Q_TL2"])
        self.assertEqual(signals[0].trigger.extent, (6, 18, 1))
        self.assertEqual(signals[1].trigger.extent, (4, 15, 1))
        self.assertEqual([signal.signal_type for signal in signals], ["straight", "straight"])


if __name__ == "__main__":
    unittest.main()
