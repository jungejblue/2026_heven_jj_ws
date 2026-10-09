from glob import glob
import os

from setuptools import find_packages, setup


package_name = "kcity_scenario_manager"

setup(
    name=package_name,
    version="0.5.0",
    packages=find_packages(exclude=["test", "scenario_tests"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml") + glob("config/*.json"),
        ),
        (
            os.path.join("share", package_name, "routes"),
            glob("routes/*.csv"),
        ),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="K-City Scenario Team",
    maintainer_email="maintainer@example.com",
    description="K-City qualifier/final scenarios and ROS Bridge ego control",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "qualifier_scenario_manager = kcity_scenario_manager.qualifier_manager:main",
            "final_scenario_manager = kcity_scenario_manager.final_manager:main",
            "pose_probe = kcity_scenario_manager.tools.debug.pose_probe:main",
            "trigger_debugger = kcity_scenario_manager.tools.debug.trigger_debugger:main",
            "route_recorder = kcity_scenario_manager.tools.recording.route_recorder:main",
            "jj_gnss_route = kcity_scenario_manager.tools.route.jj_gnss_route:main",
            "lane_route_builder = kcity_scenario_manager.tools.route.lane_route_builder:main",
            "csv_route_agent = kcity_scenario_manager.csv_route_agent:main",
            "config_check = kcity_scenario_manager.tools.debug.config_check:main",
            "qualifier_integration_guard = kcity_scenario_manager.qualifier_integration_guard:main",
            "qualifier_ego_tool = kcity_scenario_manager.qualifier_ego_tool:main",
            (
                "traffic_light_perception_stub = "
                "kcity_scenario_manager.tools.optional.traffic_light_perception_stub:main"
            ),
            (
                "traffic_light_inventory = "
                "kcity_scenario_manager.tools.debug.traffic_light_inventory:main"
            ),
        ],
    },
)
