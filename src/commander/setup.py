from setuptools import find_packages, setup

package_name = 'commander'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chris',
    maintainer_email='chris@example.com',
    description='Comandante: crea sesiones, manda objetivos, escucha feedback',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander_demo = commander.commander_node:main',
            'commander_demo_two_sessions = commander.two_sessions_demo:main',
            'avoid_obstacle_demo = commander.avoid_obstacle_demo:main',
            'avoid_obstacle_demo_joint2_90 = commander.avoid_obstacle_demo_joint2_90:main',
        ],
    },
)
