
# ENPM661 Project 03 Phase 02 - A* Algorithm Implementation on TurtleBot3

## Overview

This project implements the A* algorithm to navigate a differential drive robot (TurtleBot3 Waffle) from a start point to a goal point in a map environment. The robot operates under non-holonomic constraints and uses wheel RPM inputs to control movement.

## Team Members

| Name                           | Email            | UID       |
| ------------------------------ | ---------------- | --------- |
| Pranav Deshakulkarni Manjunath | pdeshaku@umd.edu | 121162090 |
| Anirudh Swarankar              | aniswa@umd.edu   | 121150653 |
| Munyaradzi Antony              | mantony2@umd.edu | 120482731 |


### Deliverables:
- Python Code for Part 1 (2D implementation)
- ROS Package for Part 2 (Gazebo Visualization)
- Simulation Videos (for both Part 1 and Part 2)
- GitHub Repository containing the code and necessary files


## Project Folder Structure

   proj3p2_anirudh#1_pranav#2_munyaradzi#3/     
   ├── Part01
   │   ├── a_star_2d.py          # Python code for A* algorithm in 2D
   │   └── map.py                # Map generation and obstacle handling (if any)
   ├── Part02/
   │   ├── ros_package/          # ROS package for Gazebo simulation
   │   ├── src/
   │   │   ├── a_star_gazebo.py  # Python node for Gazebo path planning
   │   │   └── launch/
   │   │       └── a_star_gazebo.launch # ROS launch file
   │   └── CMakeLists.txt        # ROS package build file
   ├── README.md                 # This file
   └── simulation_videos/        # Folder containing simulation video links
      ├── part1_simulation.mp4   # Part 1 simulation video (Gazebo)
      └── part2_simulation.mp4   # Part 2 simulation video (Falcon)                   


## Installation

1. Clone the GitHub repository:
   ```bash
   git clone <repository_link>
   ```

2. Install required dependencies for Part 1:
   ```bash
   pip install numpy matplotlib
   ```

3. Setup ROS environment for Part 2:
   Follow the instructions provided [here](https://github.com/shantanuparabumd/turtlebot3_project3).

## Usage

### Part 1: 2D Implementation
To run the A* algorithm for 2D pathfinding, execute the Python script:

```bash
python a_star_2d.py
```

**Inputs:**
- Start Coordinates (X_s, Y_s, Θ_s): (3-element vector)
- Goal Coordinates (X_g, Y_g): (2-element vector)
- Wheel RPMs (RPM1, RPM2): (2-element vector)
- Clearance (in mm)

**Example Input:**
```
Start Point: (0, 0, 0)
Goal Point: (5, 5)
Wheel RPMs: (50, 50)
Clearance: 50
```

### Part 2: Gazebo Visualization
To launch the ROS node and simulate the path planning in Gazebo, run:

```bash
roslaunch turtlebot3_project3 a_star_gazebo.launch
```

The ROS workspace should be correctly set up before launching the simulation.

## Dependencies
- Python 3.x
- ROS 2 (humble)
- numpy
- matplotlib
- opencv-python
- other ROS dependencies as specified in the project documentation

## Part 1: 2D Implementation

In this part, we implement the A* algorithm to find the optimal path for the TurtleBot3 on a 2D grid map. The algorithm takes into account the differential drive constraints and uses wheel RPM inputs to move the robot along the path. The configuration space is considered in a 3D space, where the robot's position and orientation are tracked.

### Non-holonomic Constraints
The robot's movement is constrained by its differential drive, which limits its motion in the y-direction. The A* algorithm is modified to incorporate these constraints.

## Part 2: Gazebo Visualization

This part involves simulating the robot's motion in Gazebo using ROS 2. The map is provided, and the A* algorithm's output is used to generate velocity commands for the TurtleBot3, which are then sent to ROS topics.

The motion of the TurtleBot3 is visualized in Gazebo, and the simulation shows how the robot reaches the goal based on the computed path.


## Links

- [GitHub Repository](<GitHub Repository URL>)
- [Part 1 Simulation Video](<Google Drive/YouTube link>)
- [Part 2 Simulation Video](<Google Drive/YouTube link>)

## License
This project is licensed under the Apache2.0 License - see the [LICENSE](LICENSE) file for details.

