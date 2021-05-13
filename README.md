# Spline Racer for Autonomous Racing Path Planning

Spline racer is a flavor of an autonomous racing path planner, invented by by Chris Chang <cwkchang@mit.edu>

This repository contains a simplified implementation from scratch.
Please refer to the accompanying project report for an overview of the concepts.

## List of Limitations

- vehicle is assumed to travel at unit speed in m/s
- trajectories do not respect track boundaries
- solver uses a random heuristic and does exhaustive search
- solver considers fairly naive waypoints for each object
- objects are considered sequentially in the search

## How to Run

We use a ROS2 build environment. This project can easily be made to interface with ROS2 for visualization or for running on a real or simulated robot.

1. Build Docker environment
```
docker build -t arpp .
```

2. Open a terminal inside the environment with this repository mounted
```
docker run -it -v `pwd`:/opt/arpp arpp
```

3. Go to `/opt/arpp`, the mounted directory, build the package, and configure ROS2
```
cd /opt/arpp
colcon build
source ./install/setup.bash
```

4. Run the executable
```
ros2 run spline_racer main
```
