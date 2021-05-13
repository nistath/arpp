#include <chrono>
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

  auto track_start = std::chrono::high_resolution_clock::now();
  auto track = Track(load_track(track_csv));
  track_csv.close();
  auto track_stop = std::chrono::high_resolution_clock::now();
  std::cout << "Track loaded and precomputed in "
            << std::chrono::duration<float>(track_stop - track_start).count()
            << " seconds\n";

  Objects objects(
      {{track, 0,
        Box{-102.8, 13.9, M_PI / 2 + M_PI / 8 - 2.566, 2, 2}.dilated(1.2), 30},
       {track, 0, Box{-97.0, 13., M_PI / 6 - 2.566, 1, 1}, -50},
       {track, 0, Box{-106.0, 16., M_PI / 6 - 2.566, 1, 1}, -20}});

  Path default_path = make_path(track.cbegin() + 180, track.cbegin() + 259);
  Search search(std::move(objects), default_path);

  for (const auto& object : search.get_objects()) {
    std::cout << "Object with cost: " << object.cost
              << " has corners on track indices ";

    for (const auto& corner : object.corners) {
      std::cout << corner.it.idx() << " ";
    }
    std::cout << "\n";
  }

  std::cout << "Default path has travel cost "
            << calculate_travel_cost(default_path) << std::endl;
  std::cout << "Default path has total cost " << search.get_best_cost()
            << std::endl;

  auto best_cost = search.get_best_cost();
  while (!search.advance()) {
    if (search.get_best_cost() != best_cost) {
      best_cost = search.get_best_cost();
      std::cout << "New best path has total cost " << best_cost
                << " with decisions: ";
      auto best_decisions = search.get_best_decisions();
      for (size_t i = 0; i < best_decisions.size(); ++i) {
        std::cout << "(Object Cost: " << search.get_objects()[i].cost
                  << ", Decision: ";
        switch (best_decisions[i]) {
          case Search::Decision::COLLIDE:
            std::cout << "Collide";
            break;
          case Search::Decision::LEFT:
            std::cout << "Left";
            break;
          case Search::Decision::RIGHT:
            std::cout << "Right";
            break;
        }
        std::cout << ") ";
      }
      std::cout << std::endl;
    }
  }

  return 0;
}
