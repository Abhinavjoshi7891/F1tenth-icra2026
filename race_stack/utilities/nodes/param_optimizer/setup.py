from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'param_optimizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhinav',
    maintainer_email='abhinav@iitd.ac.in',
    description='Bayesian param optimizer for race stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'param_optimizer_node = param_optimizer.param_optimizer_node:main',
        ],
    },
)