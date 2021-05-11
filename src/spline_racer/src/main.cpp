#include <fstream>
#include <iostream>

#include "Objects.hpp"
#include "Track.hpp"

int main() {
  std::ifstream track_csv("launch/maps/lvms/track.csv");
  auto track = PrecomputedTrack(load_track(track_csv));
  track_csv.close();

  Objects objects({{track, 0, {-102, 13, M_PI / 4 - 2.566913882499374, 3, 3}, -10}});
  std::sort(objects.begin(), objects.end());

  std::cout << objects[0].corners[0].it.idx() << std::endl;
  std::cout << objects[0].corners[1].it.idx() << std::endl;
  std::cout << objects[0].corners[2].it.idx() << std::endl;
  std::cout << objects[0].corners[3].it.idx() << std::endl;

  return 0;
}
