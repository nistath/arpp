#include "Track.hpp"

#include <algorithm>
#include <functional>

Track load_track(std::istream &track_csv,
                 std::function<TrackPose(TrackPose)> transform) {
  Track::Poses tposes;

  // skip first line
  while (track_csv.peek() != '\n') track_csv.ignore();

  // process subsequent lines
  while (!track_csv.eof()) {
    TrackPose tpose;

    auto get_var = [&tpose](size_t col) -> float & {
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
  }

  return Track(std::move(tposes));
}

Track load_track(std::istream &track_csv, const float buffer) {
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

Track::const_iterator Track::find_iterator(Point point, int lap) const {
  const size_t idx = std::min_element(poses.begin(), poses.end(),
                                      [point](TrackPose a, TrackPose b) {
                                        return a.dist(point) < b.dist(point);
                                      }) -
                     poses.begin();
  return iterator_at(lap, idx);
}
