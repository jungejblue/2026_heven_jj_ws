"""Shared K-City trigger and CARLA helpers from the scenario package."""

from kcity_scenario_manager.competition_common import (
    TriggerBox, connect_carla, ego_front_location, evaluation_location,
    find_ego_vehicle, get_trigger, json_msg, load_yaml,
    resolve_traffic_light, sim_time, speed_mps,
    suite_traffic_light_configs, traffic_state_name,
)

state_name = traffic_state_name
