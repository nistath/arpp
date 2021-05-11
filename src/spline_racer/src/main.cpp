#include "Track.hpp"

#include <fstream>
#include <iostream>

int main() {
  std::ifstream track_csv("launch/maps/lvms/track.csv");
  auto track = PrecomputedTrack(load_track(track_csv));
  track_csv.close();

  // for (auto& tpose : track.poses) {
    // std::cout << tpose.x << " " << tpose.y << "\n";
  // }

  return 0;
}
