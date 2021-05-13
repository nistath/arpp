#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>
#include <numeric>

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

struct Box final : public Pose {
  float w;  // width
  float l;  // length

  Box dilated(float w_buffer, float l_buffer) {
    Box b{*this};
    b.w += w_buffer;
    b.l += l_buffer;
    return b;
  }

  Box dilated(float buffer) { return dilated(buffer, buffer); }

  Point get_corner(bool front, bool right) const {
    float offset_l = (front ? 1 : -1) * l / 2;
    float offset_w = (right ? 1 : -1) * w / 2;
    return {x + offset_l * std::cos(yaw) - offset_w * std::sin(yaw),
            y + offset_w * std::cos(yaw) + offset_l * std::sin(yaw)};
  }
};

template <std::input_iterator InputIt>
InputIt closest_point(InputIt begin, InputIt end, const Point& point) {
  return std::min_element(begin, end, [&point](const Point& a, const Point& b) {
    return a.dist(point) < b.dist(point);
  });
}

template <class T>
concept is_point = std::derived_from<T, Point>;

template <class T, class A, class B>
concept is_between = std::derived_from<T, A> &&
                     (std::same_as<T, B> || !std::derived_from<B, T>);

template <class T>
requires is_point<T>
struct PointVector : public std::vector<T> {
  auto get_length() {
    if (length < 0) {
      // discrete length
      length = 0;
      for (auto it = this->cbegin() + 1; it != this->cend(); ++it) {
        length += it->dist(*(it - 1));
      }
    }

    return length;
  }

  float length = -1;
};

using Path = PointVector<PathPose>;
