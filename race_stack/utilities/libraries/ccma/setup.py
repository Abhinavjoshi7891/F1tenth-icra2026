from setuptools import setup, find_packages

package_name = 'ccma'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Abhinav',
    maintainer_email='abhinav@iitd.ac.in',
    description='Curvature Corrected Moving Average — vendored from UniBwTAS/ccma (BSD-3-Clause)',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)