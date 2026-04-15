from setuptools import setup
import os
from glob import glob

package_name = 'predictive_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhinav',
    maintainer_email='abhinav@forzaeth.com',
    description='Predictive Spline Planner for Overtaking',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sqp_node = predictive_planner.sqp_avoidance_node:main',
            'waypoint_updater = predictive_planner.update_waypoints:main',
            'collision_predictor = predictive_planner.collision_prediction:main',
            'gp_node = predictive_planner.gp_node:main',
        ],
    },
)
