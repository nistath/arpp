#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <random>

#include "Geometry.hpp"
#include "Objects.hpp"
#include "Spline.hpp"

inline float calculate_object_cost(const Path& path, const Objects& objects) {
  return std::accumulate(objects.cbegin(), objects.cend(), 0.0f,
                         [&](float cost, const Object& object) {
                           return cost +
                                  (collides(object, path) ? object.cost : 0);
                         });
}

inline float calculate_travel_cost(const Path& path) {
  return path.get_length();  // assuming unit speed
}

inline float calculate_total_cost(const Path& path, const Objects& objects) {
  return calculate_object_cost(path, objects) + calculate_travel_cost(path);
}

class Search {
 private:
  struct Node {
    // in the current state, left and right might be flipped
    enum class Decision { COLLIDE, LEFT, RIGHT };

    long unsigned int heuristic_cost;
    Objects::const_iterator objit;
    Decision decision;
    std::shared_ptr<const Node> parent;

    template <std::output_iterator<Decision> OutIt>
    void get_decisions(OutIt out) const {
      if (parent) {
        parent->get_decisions(out);
      }
      *out++ = decision;
    }

    template <std::output_iterator<Point> OutIt>
    void get_waypoints(OutIt out) const {
      if (parent) {
        parent->get_waypoints(out);
      }
      *out++ = get_waypoint();
    }

    Point get_waypoint() const {
      switch (decision) {
        default:
        case Decision::COLLIDE:
          return objit->corners[0];
        case Decision::LEFT:
          return objit->corners[1];
        case Decision::RIGHT:
          return objit->corners[2];
      }
    }

    // default comparison operator
    auto operator<=>(const Node&) const = default;
  };

 public:
  using Decision = Node::Decision;

  Search(Objects input_objects, Path initial_path)
      : objects{input_objects},
        best_path{std::move(initial_path)},
        best_cost{calculate_total_cost(best_path, objects)},
        start_pose{best_path.front()},
        end_pose{best_path.back()} {
    std::sort(objects.begin(), objects.end());
  }
  Search(Search&&) = default;
  Search(const Search&) = default;

  bool advance() {
    if (done) return true;

    auto add_object_nodes = [&](Objects::const_iterator const objit,
                                std::shared_ptr<const Node> const parent =
                                    nullptr) {
      leaves.push(Node{rand(), objit, Node::Decision::COLLIDE, parent});
      leaves.push(Node{rand(), objit, Node::Decision::LEFT, parent});
      leaves.push(Node{rand(), objit, Node::Decision::RIGHT, parent});
      remaining_nodes[objit] = 3;
    };

    if (leaves.empty()) add_object_nodes(objects.cbegin());

    // get best node according to heuristic_cost
    Node node = leaves.top();
    leaves.pop();

    // evaluate the decision
    std::vector<Point> waypoints;
    waypoints.emplace_back(start_pose.x, start_pose.y);
    node.get_waypoints(std::back_inserter(waypoints));
    waypoints.emplace_back(end_pose.x, end_pose.y);

    CubicSpline spline(waypoints, start_pose.yaw, end_pose.yaw);
    {
      const Path candidate = spline.interpolate_path(0.5);
      const auto candidate_cost = calculate_total_cost(candidate, objects);

      if (candidate_cost < best_cost) {
        best_path = std::move(candidate);
        best_cost = candidate_cost;
        best_decisions.clear();
        node.get_decisions(std::back_inserter(best_decisions));
      }
    }

    assert(remaining_nodes[node.objit] > 0);
    remaining_nodes[node.objit]--;
    if (remaining_nodes[node.objit] == 0) {
      const auto next_objit = node.objit + 1;

      // if the next object exists and has not been used then add its nodes
      // using the current node as their parent
      if (next_objit != objects.cend() &&
          remaining_nodes.count(next_objit) == 0) {
        add_object_nodes(next_objit, std::make_shared<Node>(node));
      }
    }

    return done = leaves.empty();
  }

  const auto& get_objects() const { return objects; }
  const auto& get_best_path() const { return best_path; }
  auto get_best_cost() const { return best_cost; }
  const auto& get_best_decisions() const { return best_decisions; }

 private:
  Objects objects;
  Path best_path;
  float best_cost;
  PathPose start_pose;
  PathPose end_pose;

  std::vector<Node::Decision> best_decisions{};
  std::map<Objects::const_iterator, int> remaining_nodes{};
  std::priority_queue<Node> leaves{};
  bool done = false;

  std::mt19937 rand{};
};
