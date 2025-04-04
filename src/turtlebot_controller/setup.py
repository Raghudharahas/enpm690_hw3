from setuptools import find_packages, setup

package_name = 'turtlebot_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='drecon',
    maintainer_email='raghudharahas@gmail.com',
    description='A ROS 2 package for TurtleBot teleoperation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = turtlebot_controller.teleop:main',
            'autonomous = turtlebot_controller.autonomous:main',
        ],
    },
)
