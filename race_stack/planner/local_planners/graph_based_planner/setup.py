from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'graph_based_planner'


def collect(base_dir, install_root):
    """Recursively collect all data files inside a directory."""
    pairs = []
    for root, _dirs, files in os.walk(base_dir):
        if not files:
            continue
        rel = os.path.relpath(root, base_dir)
        inst = os.path.join(install_root, rel) if rel != '.' else install_root
        pairs.append((inst, [os.path.join(root, f) for f in files]))
    return pairs


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'),
     glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    (os.path.join('share', package_name, 'config'),
     glob(os.path.join('config', '*.yaml'))),
]
data_files += collect('params',     os.path.join('share', package_name, 'params'))
data_files += collect('inputs',     os.path.join('share', package_name, 'inputs'))
data_files += collect('logs',       os.path.join('share', package_name, 'logs'))
data_files += collect('graph_ltpl', os.path.join('share', package_name, 'graph_ltpl'))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    data_files=data_files,
    install_requires=[
        'setuptools',
        'python-igraph',
        'trajectory-planning-helpers',
    ],
    zip_safe=True,
    maintainer='Abhinav',
    maintainer_email='abhinav@forzaeth.com',
    description='Graph-based local trajectory planner — ROS 2 port',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'graph_planner_node  = graph_based_planner.graph_planner_node:main',
            'path_publisher_node = graph_based_planner.path_publisher_node:main',
        ],
    },
)
