"""vehicle_spawn blueprint selection helper test."""

import sys
import types
import unittest


if "carla" not in sys.modules:
    carla = types.ModuleType("carla")
    carla.World = object
    carla.Transform = object
    carla.Location = object
    carla.Rotation = object
    sys.modules["carla"] = carla


from kcity_scenario_manager.common.vehicle_spawn import (
    blueprint_candidates,
    choose_blueprint,
)


class FakeBlueprint:
    def __init__(self, bp_id):
        self.id = bp_id


class FakeLibrary:
    def __init__(self, ids):
        self._bps = [FakeBlueprint(i) for i in ids]

    def find(self, exact):
        for bp in self._bps:
            if bp.id == exact:
                return bp
        raise IndexError(exact)

    def filter(self, pattern):
        token = pattern.replace("*", "").lower()
        return [
            bp for bp in self._bps
            if token in bp.id.lower()
        ]


class FakeWorld:
    def __init__(self, ids):
        self._lib = FakeLibrary(ids)

    def get_blueprint_library(self):
        return self._lib


class VehicleSpawnTests(unittest.TestCase):
    def test_exact_blueprint_has_priority(self):
        world = FakeWorld(["vehicle.other", "vehicle.bp_heven", "vehicle.heven_alt"])
        candidates = blueprint_candidates(
            world, exact_id="vehicle.heven_alt", contains="heven")
        self.assertEqual(candidates[0].id, "vehicle.heven_alt")
        self.assertEqual(
            choose_blueprint(world, exact_id="vehicle.heven_alt", contains="heven").id,
            "vehicle.heven_alt",
        )

    def test_contains_fallback(self):
        world = FakeWorld(["vehicle.other", "vehicle.bp_heven"])
        bp = choose_blueprint(world, exact_id="", contains="heven")
        self.assertEqual(bp.id, "vehicle.bp_heven")
