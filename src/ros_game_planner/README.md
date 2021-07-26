# ROS_GAME_PLANNER

## 1. Description
This reopsitory containts all components for testing game theoretical planner.

## 2. Requirement
- ROS Melodic
- CMake

## 3. Usage
#### I. Build
``` 
mkdir -p game_planner_workspace/src
cd game_planner_workspace/src
catkin_init_workspace
git clone https://github.com/purewater0901/ros_game_planner.git
cd ..
catkin_make
```

#### II. Start Simulation
```
source devel/setup.bash
roslaunch  open_simulator game_planner.launch
```

#### III. Run Test Code
```
cd game_planner_workspace
catkin_make run_tests
