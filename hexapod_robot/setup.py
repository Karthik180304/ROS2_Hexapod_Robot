from setuptools import setup

package_name = 'hexapod_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karthikms',
    maintainer_email='karthikms@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "builderx = hexapod_robot.builder:main",
            "leg_movement = hexapod_robot.leg_movement:main",
            "linefollower = hexapod_robot.line_following:main",
            "leg_movement_demo = hexapod_robot.leg_movement_demo:main",
        ],
    },
)