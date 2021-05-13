#include <fstream>
#include <iostream>

#include "Objects.hpp"
#include "Track.hpp"
#include "Search.hpp"

int main() {
  std::ifstream track_csv("launch/maps/lvms/track.csv");
  auto track = PrecomputedTrack(load_track(track_csv));
  track_csv.close();

  Objects objects(
      {{track, 0,
        Box{-102, 13, M_PI / 4 - 2.566913882499374, 3, 3}.dilated(1.2), 30},
       {track, 0,
        Box{-102, 13, M_PI / 4 - 2.566913882499374, 3, 3}.dilated(1.2), -10}});
  std::sort(objects.begin(), objects.end());

  for (const auto& obj : objects) {
    std::cout << obj.corners[0].it.idx() << " " << obj.corners[1].it.idx()
              << " " << obj.corners[2].it.idx() << " "
              << obj.corners[3].it.idx() << std::endl;
  }

  return 0;
}
