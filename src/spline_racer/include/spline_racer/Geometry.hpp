#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>

struct Point {
  float x;
  float y;

  auto dist_from_origin() const { return std::hypot(x, y); }

  auto dist(const Point& other) const {
    return (*this - other).dist_from_origin();
  }

  bool in_bounding_box(const Point& min, const Point& max) const {
    return (min.x < x) && (x < max.x) && (min.y < y) && (y < max.y);
  }

  Point operator+(const Point& other) const {
    return Point{x + other.x, y + other.y};
  }

  Point operator-(const Point& other) const {
    return Point{x - other.x, y - other.y};
  }
};

struct Pose : public Point {
  float yaw;  // orientation
};

struct PathPose : public Pose {
  float k;  // curvature
};

struct State : public Pose {
  float v;  // velocity
};

struct Setpoint : public PathPose, public State {
  float a;  // acceleration
};

template<std::input_iterator InputIt>
InputIt closest_point(InputIt begin, InputIt end, const Point& point) {
  return std::min_element(begin, end, [&point](const Point& a, const Point& b) {
    return a.dist(point) < b.dist(point);
  });
}
