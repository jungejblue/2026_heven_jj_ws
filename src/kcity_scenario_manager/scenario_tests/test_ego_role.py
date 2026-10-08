import unittest

from kcity_scenario_manager.competition_common import find_ego_vehicle


class Actors(list):
    def filter(self, _pattern):
        return self


class World:
    def __init__(self, roles):
        self.actors = Actors([
            type("Actor", (), {"attributes": {"role_name": role}})()
            for role in roles
        ])

    def get_actors(self):
        return self.actors


class EgoRoleTests(unittest.TestCase):
    def test_combined_launch_requires_exact_role(self):
        world = World([""])
        self.assertIsNone(find_ego_vehicle(world, "ego_vehicle", strict=True))
        self.assertIs(find_ego_vehicle(world, "ego_vehicle", strict=False), world.actors[0])
        exact = World(["", "ego_vehicle"])
        self.assertIs(find_ego_vehicle(exact, "ego_vehicle", strict=True), exact.actors[1])
