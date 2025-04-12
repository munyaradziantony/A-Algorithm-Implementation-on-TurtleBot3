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

You should see the turtlebot3 along with the maze in gazebo

![gazebo](gazebo.png)

Explore Topics

```sh
ros2 topic list
```

Publish to topic (cmd_vel)

```sh
ros2 topic pub \cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1"
```

## Write a script

We have provide a script fro reference in [teleop.py](/scripts/teleop.py)

You can run the script using

```sh
ros2 run turtlebot3_project3 teleop.py
```

## Add new Python executable

* Write a new python script and store it in a folder
* Update the CMakeLists.txt file 

```xml
# Install python scripts

install(PROGRAMS 
  scripts/teleop.py
  # You can add more scripts here
  DESTINATION lib/${PROJECT_NAME}
)

```

* Build the package
* Source the install folder
* Run the executable

```sh
ros2 run turtlebot3_project3 teleop.py
```


# Error

* Pynput Module Not Found

```sh
pip install pynput
```
