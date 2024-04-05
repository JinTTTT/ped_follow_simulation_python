from setuptools import find_packages, setup

package_name = 'driving_decision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jin Tao',
    maintainer_email='tj19970215@gmail.com',
    description='use this package to get the new driving command, desired driving velocity and desired steering angle',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #  pedestrain_pos_current is a executable command name, used by terminal to call ros2 run
            'driving_decision_exec = driving_decision.driving_decision_node:main',
            'update_vehicle_state_exec = driving_decision.update_vehicle_state_node:main',
        ],
    },
)
