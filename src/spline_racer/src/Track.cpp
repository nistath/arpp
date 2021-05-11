#include "Track.hpp"

#include <algorithm>
#include <functional>
#include <limits>

Track load_track(std::istream& track_csv,
                 std::function<TrackPose(TrackPose)> transform) {
  Track::Poses tposes;

  // skip first line
  while (track_csv.peek() != '\n') track_csv.ignore();

  // process subsequent lines
  while (!track_csv.eof()) {
    TrackPose tpose;

    auto get_var = [&tpose](size_t col) -> float& {
      switch (col) {
        case 0:
          return tpose.s;
        case 1:
          return tpose.x;
        case 2:
          return tpose.y;
        case 3:
          return tpose.yaw;
        case 4:
          return tpose.k;
        case 5:
          return tpose.l.w;
        case 6:
          return tpose.r.w;
        case 7:
          return tpose.l.x;
        case 8:
          return tpose.l.y;
        case 9:
          return tpose.l.yaw;
        case 10:
          return tpose.r.x;
        case 11:
          return tpose.r.y;
        case 12:
          return tpose.r.yaw;
        default:
          throw std::length_error("invalid col: " + std::to_string(col));
      }
    };

    size_t col = 0;
    while (track_csv >> get_var(col)) {
      col++;
      if (track_csv.peek() == ',') track_csv.ignore();
      if (track_csv.peek() == ' ') track_csv.ignore();
      if (track_csv.peek() == '\r') track_csv.ignore();
      if (track_csv.peek() == '\n') {
        track_csv.ignore();
        break;
      }
      if (track_csv.eof()) break;
    }
    tposes.push_back(transform(tpose));
    track_csv.peek();  // check for eof
  }

  return Track(std::move(tposes));
}

Track load_track(std::istream& track_csv, const float buffer) {
  return load_track(track_csv, [&](TrackPose tpose) {
    // change yaw to our reference
    tpose.yaw += M_PI / 2;
    tpose.l.yaw += M_PI / 2;
    tpose.r.yaw += M_PI / 2;

    // shrink width by buffer on either side
    tpose.l.w -= buffer;
    tpose.r.w -= buffer;

    // recalculate for new width
    tpose.l.x = tpose.x + tpose.l.w * std::cos(tpose.yaw + M_PI / 2);
    tpose.l.y = tpose.y + tpose.l.w * std::cos(tpose.yaw + M_PI / 2);

    tpose.r.x = tpose.x + tpose.r.w * std::cos(tpose.yaw - M_PI / 2);
    tpose.r.y = tpose.y + tpose.r.w * std::cos(tpose.yaw - M_PI / 2);
    return tpose;
  });
}

size_t Track::find_idx(const Point& point) const {
  return closest_point(poses.cbegin(), poses.cend(), point) - poses.cbegin();
}

// NOTE: the nice recursive version doesn't get optimized :(
template <std::input_iterator InputIt, typename F>
auto min_adjacent_difference(InputIt begin, InputIt end, F key) {
  auto first = begin;
  auto second = ++begin;
  using D = decltype(key(*second - *first));

  D result = std::numeric_limits<D>::max();
  while (second != end) {
    result = std::min(result, key(*second - *first));

    ++first;
    ++second;
  }
  return result;
}

TrackIdxMap generate_track_idx_map(const Track& track) {
  const auto b = track.poses.cbegin();
  const auto e = track.poses.cend();

  const float resolution =
      min_adjacent_difference(b, e, std::mem_fn(&Point::dist_from_origin));

  auto x_comp = [](const Point& a, const Point& b) { return a.x < b.x; };
  auto y_comp = [](const Point& a, const Point& b) { return a.y < b.y; };
  Point min{std::min_element(b, e, x_comp)->x - 10,
            std::min_element(b, e, y_comp)->y - 10};
  Point max{std::max_element(b, e, x_comp)->x + 10,
            std::max_element(b, e, y_comp)->y + 10};
  // HACK: Buffer the map instead of accounting for width

  const auto range = max - min;
  const size_t n_rows = std::ceil(range.x / resolution);
  const size_t n_cols = std::ceil(range.y / resolution);

  TrackIdxMap::Storage storage(n_rows * n_cols);
#pragma omp parallel for schedule(static)
  for (size_t xb = 0; xb < n_rows; ++xb) {
    for (size_t yb = 0; yb < n_cols; ++yb) {
      storage[xb * n_cols + yb] =
          track.Track::find_idx(Point{xb * resolution, yb * resolution} + min);
    }
  }

  return TrackIdxMap(storage, resolution, min, n_rows, n_cols);
}
