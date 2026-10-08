from glob import glob
from setuptools import find_packages, setup

package_name = 'heven_carla_adapter'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HEVEN',
    maintainer_email='heven@example.com',
    description='CARLA 0.9.15 to HEVEN sensor and actuator adapter with integrated scenarios.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'sensor_adapter = heven_carla_adapter.sensor_adapter:main',
        'control_adapter = heven_carla_adapter.control_adapter:main',
        'traffic_light_adapter = heven_carla_adapter.traffic_light_adapter:main',
        'heven_route_spawn = heven_carla_adapter.route_spawn:main',
    ]},
)
