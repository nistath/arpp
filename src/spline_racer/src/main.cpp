#include <fstream>
#include <iostream>

#include "Objects.hpp"
#include "Search.hpp"
#include "Track.hpp"

Path make_path(Track::const_iterator begin, Track::const_iterator end) {
  Path path;
  while (begin != end) {
    path.emplace_back(*begin++);
  }
  return path;
}

int main() {
  std::ifstream track_csv("launch/maps/lvms/track.csv");
  auto track = PrecomputedTrack(load_track(track_csv));
  track_csv.close();

  Objects objects(
      {{track, 0,
        Box{-102.8, 13.9, M_PI / 2 + M_PI / 8 - 2.566, 5, 2}.dilated(1.2), 30},
       {track, 0, Box{-97.0, 13., M_PI / 6 - 2.566, 1.0, 1.0}, -10}});

  for (const auto& obj : objects) {
    std::cout << "Object with cost: " << obj.cost
              << " has corners on track indices ";

    for (const auto& corner : obj.corners) {
      std::cout << corner.it.idx() << " ";
    }
    std::cout << "\n";
  }

  Path default_path = make_path(track.cbegin() + 202, track.cbegin() + 259);
  Search search(std::move(objects), default_path);
  std::cout << "Default path has travel cost "
            << calculate_travel_cost(default_path) << std::endl;
  std::cout << "Default path has total cost " << search.get_best_cost()
            << std::endl;

  auto best_cost = search.get_best_cost();
  while (!search.advance()) {
    if (search.get_best_cost() != best_cost) {
      best_cost = search.get_best_cost();
      std::cout << "Best path has total cost " << best_cost << std::endl;
    }
  }

  return 0;
}
