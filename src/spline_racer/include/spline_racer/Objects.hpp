#pragma once

#include <algorithm>
#include <array>
#include <vector>

#include "Geometry.hpp"
#include "Track.hpp"

inline std::array<OnTrack<Point>, 4> get_four_corners(const Box& box,
                                                      const Track& track,
                                                      int lap) {
  // HACK: This does not work correclty for objects spanning the start line.
  //       To fix, we need
  std::array<OnTrack<Point>, 4> corners = {
      OnTrack<Point>{box.get_corner(false, false), track, lap},
      OnTrack<Point>{box.get_corner(false, true), track, lap},
      OnTrack<Point>{box.get_corner(true, false), track, lap},
      OnTrack<Point>{box.get_corner(true, true), track, lap}};

  std::sort(corners.begin(), corners.end());

  return corners;
}

class Object {
 public:
  Object(const Track& track, int lap, Box box, float cost)
      : box{box}, cost{cost}, corners{get_four_corners(box, track, lap)} {}

  auto operator<=>(const Object& other) const {
    return corners[0].it <=> other.corners[0].it;
  }

  Box box;
  float cost;
  std::array<OnTrack<Point>, 4> corners;
};

using Objects = std::vector<Object>;

template <class T>
inline bool collides(const Object& object, const PointVector<T>& path) {
  // TODO: Optimize this using track iterator for proximity guess.
  return std::any_of(path.cbegin(), path.cend(), [&](const Point& point) {
    return object.box.contains(point);
  });
}
