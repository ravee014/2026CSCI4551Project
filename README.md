# CSCI 4551 - UMN Campus Navigation Simulation

ROS 2 Jazzy + Gazebo Harmonic workspace for UMN campus navigation.

## Setup & Build

```bash
cd ~/csci4551
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Launch Simulation

```bash
ros2 launch campus_sim sim.launch.py
```
