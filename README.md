## Installation
```
cd ~/catkin_ws/src/
git clone https://github.com/HJ7021/tianbot.git
cd ~/catkin_ws && catkin_make
```

## Simulation
```
roslaunch tianracer_gazebo demo_tianracer_teb_nav.launch

export TIANRACER_WORLD=raicom
rosrun tianracer_gazebo judge_system_node.py

```
