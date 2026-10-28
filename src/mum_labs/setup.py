from glob import glob

from setuptools import setup

package_name = 'mum_labs'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sydykanov S.',
    maintainer_email='ssydykanov@gmail.com',
    description='Узлы практических работ курса «Механика управляемых машин» (ROS 2 Jazzy)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fk_node = mum_labs.fk_node:main',
            'ik_node = mum_labs.ik_node:main',
            'cartesian_line_node = mum_labs.cartesian_line_node:main',
            'trajectory_node = mum_labs.trajectory_node:main',
            'pid_node = mum_labs.pid_node:main',
            'moveit_lab = mum_labs.moveit_lab:main',
            'diff_odometry_node = mum_labs.diff_odometry_node:main',
            'go_to_goal_node = mum_labs.go_to_goal_node:main',
            'nav_waypoints = mum_labs.nav_waypoints:main',
            'color_tracker = mum_labs.color_tracker:main',
        ],
    },
)
