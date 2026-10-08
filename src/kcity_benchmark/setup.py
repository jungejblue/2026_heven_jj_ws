from setuptools import setup
from glob import glob
import os

package_name = 'kcity_benchmark'

setup(
    name=package_name,
    version='0.5.0',
    packages=[package_name],
    package_data={package_name: ['data/*.json']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=False,
    maintainer='HEVEN',
    maintainer_email='heven@example.com',
    description='K-City competition benchmark',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'benchmark_runner = kcity_benchmark.benchmark_runner:main',
            'benchmark_hud = kcity_benchmark.benchmark_hud:main',
        ],
    },
)
