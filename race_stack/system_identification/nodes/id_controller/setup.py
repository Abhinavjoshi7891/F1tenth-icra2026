from setuptools import setup

package_name = 'id_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/parameters', ['parameters/experiments.yaml']),
        ('share/' + package_name + '/launch', ['launch/id_controller_launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonathan Becker',
    maintainer_email='JonathanBecker.Tech@gmail.com',
    description='The controller for sysid experiments',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'id_controller = id_controller.controller_node:main',
        ],
    },
)
