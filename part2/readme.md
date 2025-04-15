# ENPM 661: Path Planning for Autonomous Robots
### Instructions for Project3- Phase2

Forked from template which has been modified as per requirements- https://github.com/koustubh1012/turtlebot3_project3 

## Map Dimensions

All dimensions are in milimeters.

![map](map.png)

## How to run

Package can either be copied in the src of workspace or a symbolic link can be created.
```sh
mkdir project3_ws
cd project3_ws
git clone git@github.com:munyaradziantony/A-Algorithm-Implementation-on-TurtleBot3.git
mkdir src
cp -r A-Algorithm-Implementation-on-TurtleBot3/part2 src/ 
#or 
ln -snf $PWD/A-Algorithm-Implementation-on-TurtleBot3/part2 src/pkg
# To compile
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --event-handlers console_cohesion+
# Source the package
source install/setup.bash
```

## Test Setup

Launch Environment

```sh
ros2 launch turtlebot3_project3 competition_world.launch
```

Run A* path finder
```bash
ros2 run turtlebot3_project3 a_start_path
```

Input the start and goal positions


You should see the turtlebot3 along with the maze in gazebo

![gazebo](gazebo.png)



