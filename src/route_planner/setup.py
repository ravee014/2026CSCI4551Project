from setuptools import setup

setup(
    name='route_planner',
    version='0.1.0',
    packages=['route_planner'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/route_planner']),
        ('share/route_planner', ['package.xml']),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': ['route_planner = route_planner.route_planner_node:main']},
)
