from setuptools import find_packages, setup

package_name = 'robot_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/robot_node.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chris',
    maintainer_email='chris@example.com',
    description='Nodo Robot: adaptador ROS2 alrededor de un RobotConnectorPort',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_node = robot_node.node:main',
            'cr5_first_contact_demo = robot_node.cr5_first_contact_demo:main',
            'cr5_repeated_joint1_moves_demo = robot_node.cr5_repeated_joint1_moves_demo:main',
            'cr5_disable_demo = robot_node.cr5_disable_demo:main',
            'cr5_go_home_demo = robot_node.cr5_go_home_demo:main',
            'cr5_semicircle_demo = robot_node.cr5_semicircle_demo:main',
            'cr5_circle_demo = robot_node.cr5_circle_demo:main',
            'cr5_wave_demo = robot_node.cr5_wave_demo:main',
        ],
    },
)
