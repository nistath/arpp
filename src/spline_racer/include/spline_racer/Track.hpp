#pragma once

#include <istream>
#include <iterator>
#include <type_traits>
#include <vector>

#include "Geometry.hpp"
#include "assert.hpp"

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
    auto operator<=>(const self_type&) const = default;

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

    const_iterator add_lap(int delta_lap) const {
      return const_iterator(*m_track, m_lap + delta_lap, m_idx);
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
  virtual size_t find_idx(const Point& point) const;
  const_iterator find_iterator(const Point& point, int lap) const {
    return iterator_at(lap, find_idx(point));
  }
  const_iterator cbegin(int lap = 0) const { return iterator_at(lap, 0); }
};

class TrackIdxMap {
 public:
  using Idx = size_t;
  using Storage = std::vector<Idx>;

  TrackIdxMap(Storage storage, float resolution, Point min, size_t n_rows,
              size_t n_cols)
      : resolution{resolution},
        min{min},
        n_rows{n_rows},
        n_cols{n_cols},
        m_storage{std::move(storage)} {}
  TrackIdxMap(TrackIdxMap&&) = default;
  TrackIdxMap(const TrackIdxMap&) = default;

  const float resolution;
  const Point min;
  const size_t n_rows;
  const size_t n_cols;

  Idx find(const Point& point) const {
    const size_t row = std::round((point.x - min.x) / resolution);
    const size_t col = std::round((point.y - min.y) / resolution);
    ASSERT(row < n_rows);
    ASSERT(col < n_cols);
    return m_storage[row * n_cols + col];
  }

 private:
  const Storage m_storage;
};

TrackIdxMap generate_track_idx_map(const Track& track);

class PrecomputedTrack : public Track {
 public:
  PrecomputedTrack(Track track_)
      : Track{track_}, idx_map{generate_track_idx_map(*this)} {
    ASSERT(std::all_of(poses.cbegin(), poses.cend(), [&](const Point& point) {
      return Track::find_idx(point) == PrecomputedTrack::find_idx(point);
    }));
  }
  PrecomputedTrack(PrecomputedTrack&&) = default;
  PrecomputedTrack(const PrecomputedTrack&) = default;

  size_t find_idx(const Point& point) const final {
    const int approx_idx = idx_map.find(point);
    // TODO: Might be best to use Track::const_iterator to work cyclically
    const auto b = poses.cbegin() + std::max<int>(approx_idx - 1, 0);
    const auto e = poses.cbegin() + std::min<int>(approx_idx + 2, poses.size());
    return closest_point(b, e, point) - poses.cbegin();
  }

  const TrackIdxMap idx_map;
};

Track load_track(std::istream& track_csv, const float buffer = 0);

template <class T>
requires is_point<T> struct OnTrack : public T {
  OnTrack(const T& p, Track::const_iterator it) : T{p}, it{std::move(it)} {}
  OnTrack(const T& p, const Track& track, int lap)
      : OnTrack{p, track.find_iterator(p, lap)} {}

  OnTrack(OnTrack&&) = default;
  OnTrack(const OnTrack&) = default;
  OnTrack& operator=(const OnTrack&) = default;

  OnTrack add_lap(int delta_lap) const {
    return {*static_cast<T>(this), it.add_lap(delta_lap)};
  }

  auto operator<=>(const OnTrack& other) const { return it <=> other.it; }

  Track::const_iterator it;
};
