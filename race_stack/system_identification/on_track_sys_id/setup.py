from setuptools import setup
import os
from glob import glob

package_name = 'on_track_sys_id'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.helpers'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
        (os.path.join('share', package_name, 'models/SIM'), glob('models/SIM/*')),
        (os.path.join('share', package_name, 'models/NUC2'), glob('models/NUC2/*')),
        (os.path.join('share', package_name, 'models/NUC5'), glob('models/NUC5/*')),
        (os.path.join('share', package_name, 'models/NUC6'), glob('models/NUC6/*')),
        (os.path.join('share', package_name, 'models/NUC7'), glob('models/NUC7/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhinav',
    maintainer_email='abhinav@todo.todo',
    description='On-track system identification package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'on_track_sys_id = on_track_sys_id.on_track_sys_id:main',
        ],
    },
)
