#include <istream>
#include <iterator>
#include <vector>

#include "Geometry.hpp"

struct TrackPose : public PathPose {
  float s;  // distance along the length of m_track

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

  struct const_iterator {
    using self_type = const_iterator;
    using iterator_category = std::random_access_iterator_tag;
    using difference_type = std::ptrdiff_t;
    using value_type = TrackPose;
    using pointer = const value_type*;
    using reference = const value_type&;

    const_iterator(const Track& m_track) : m_track{&m_track} {}
    const_iterator(const Track& track, int lap, size_t idx)
        : m_track{&track}, m_lap{lap}, m_idx{idx} {}

    const_iterator(const self_type&) = default;
    const_iterator& operator=(const self_type&) = default;

    reference operator*() const { return m_track->poses[m_idx]; }
    pointer operator->() const { return &m_track->poses[m_idx]; }

    // automatic comparison operator
    bool operator<=>(const self_type&) const = default;

    difference_type operator-(const self_type& other) const {
      const difference_type sz = m_track->poses.size();
      return sz * (m_lap - other.m_lap) + (m_idx - other.m_idx);
    }

    self_type operator+(int i) const {
      const difference_type sz = m_track->poses.size();

      const difference_type far_m_idx = m_idx + i;
      auto div = far_m_idx / sz;
      auto new_m_idx = far_m_idx % sz;
      if (new_m_idx < 0) {
        div--;
        new_m_idx += sz;
      }

      const auto new_m_lap = m_lap + div;
      return self_type(*m_track, new_m_lap, new_m_idx);
    }

    self_type operator-(int i) const { return *this + (-i); }
    self_type& operator+=(int i) { return *this = (*this + i); }
    self_type& operator-=(int i) { return *this = (*this - i); }
    self_type& operator++() { return *this += 1; }
    self_type operator++(int) {
      auto old = *this;
      ++*this;
      return old;
    }
    self_type& operator--() { return *this -= 1; }
    self_type operator--(int) {
      auto old = *this;
      --*this;
      return old;
    }

    auto lap() const { return m_lap; }
    auto idx() const { return m_idx; }

   private:
    const Track* m_track;
    int m_lap = 0;
    size_t m_idx = 0;
  };

  const_iterator iterator_at(int lap, size_t idx) const {
    return const_iterator(*this, lap, idx);
  }
  const_iterator find_iterator(Point point, int lap) const;
  const_iterator cbegin(int lap = 0) const {
    return iterator_at(lap, 0);
  }
};

class CheckpointTrack;

Track load_track(std::istream& track_csv, const float buffer = 0);
