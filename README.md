# Spline Racer for Autonomous Racing Path Planning

Spline racer is a flavor of an autonomous racing path planner, invented and developed by [Chris Chang](mailto:cwkchang@mit.edu), [Nick Stathas](mailto:nistath@mit.edu), and [MIT Driverless](https://driverless.mit.edu) team members.

This repository contains a simplified, but well-engineered, implementation from scratch by Nick.
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
Example output:
```
Track loaded and precomputed in 4.34146 seconds
Object with cost: -20 has corners on track indices 202 203 203 204
Object with cost: 30 has corners on track indices 205 208 212 215
Object with cost: -50 has corners on track indices 217 218 218 219
Default path has travel cost 38.9929
Default path has total cost 68.9929
New best path has total cost 49.2058 with decisions: (Object Cost: -20, Decision: Right)
New best path has total cost 49.1199 with decisions: (Object Cost: -20, Decision: Collide)
New best path has total cost 49.0775 with decisions: (Object Cost: -20, Decision: Left)
New best path has total cost 19.4591 with decisions: (Object Cost: -20, Decision: Left) (Object Cost: 30, Decision: Left)
New best path has total cost 3.51051 with decisions: (Object Cost: -20, Decision: Left) (Object Cost: 30, Decision: Left) (Object Cost: -50, Decision: Collide)
New best path has total cost 2.96748 with decisions: (Object Cost: -20, Decision: Left) (Object Cost: 30, Decision: Left) (Object Cost: -50, Decision: Right)
```
