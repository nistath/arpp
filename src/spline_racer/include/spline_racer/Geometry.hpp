#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>

struct Point {
  float x;
  float y;
};

struct Pose : public Point {
  float yaw; // orientation
};

struct PathPose : public Pose {
  float k; // curvature
};

struct State : public Pose {
  float v; // velocity
};

struct Setpoint : public PathPose, public State {
  float a;  // acceleration
};
