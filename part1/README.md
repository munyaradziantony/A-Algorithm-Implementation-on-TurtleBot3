
# ENPM661 Project 03 Phase 02 Part1 - A* Algorithm Implementation on Non Holonomic Robot

## Overview
We used Euler Integration to compute the Non holonomic paths for Differential drive robot. 
The robot's movement is constrained by its differential drive, which limits its motion in the y-direction. The A* algorithm is modified to incorporate these constraints.

## Usage
```bash
python3 search.py
```
## Implementation Notes
To make the project more readable and manageable, the project is distributed in multiple files. 
- canvas.py contains Canvas class which creates CV mat and populates it with obstacles. 
- helpers.py contains custom classes like Node, Waypoint
- robot.py contains robot specific implementation of methods such as euler integration
- search.py contains the search class with a_star as method


## Dependencies
- Python 3.x
- numpy
- matplotlib
- opencv-python

## License
This project is licensed under the Apache2.0 License - see the [LICENSE](LICENSE) file for details.

