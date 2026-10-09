from glob import glob
from setuptools import find_packages, setup


package_name = "heven_carla_bringup"


setup(
    name=package_name,
    version="0.1.3",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "LICENSE"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/docs", glob("docs/*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="HEVEN",
    maintainer_email="heven@example.com",
    description="CARLA 0.9.15 ROS 2 Humble sensor bring-up for vehicle.heven.ev.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "heven_warmup_guard = heven_carla_bringup.warmup_guard:main",
            "heven_sensor_gate = heven_carla_bringup.sensor_gate:main",
            "heven_readiness_monitor = heven_carla_bringup.readiness_monitor:main",
            "heven_validate_config = heven_carla_bringup.config_validator:main",
        ],
    },
)
