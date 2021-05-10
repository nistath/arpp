#include <istream>
#include <vector>

#include "Geometry.hpp"

struct TrackPose : public PathPose {
  float s;  // distance along the length of track

  struct SidePose : public Pose {
    float w;  // distance from midpoint to side point
  };

  SidePose l;  // left side point
  SidePose r;  // right side point
};

class Track {
 public:
  using Poses = std::vector<TrackPose>;

  Track(Poses poses) : poses{std::move(poses)} {}
  Track(Track&&) = default;
  Track(const Track&) = default;

  const Poses poses;
};

Track load_track(std::istream &track_csv, const float buffer = 0);
