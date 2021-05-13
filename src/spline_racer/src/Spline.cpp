#include "Spline.hpp"

#include <iterator>

// Optimized version of https://github.com/TUMFTM/trajectory_planning_helpers

CubicSpline::CubicSpline(std::vector<Point> waypoints_, float psi_s,
                         float psi_e)
    : waypoints{std::move(waypoints_)} {
  const auto no_splines = waypoints.size() - 1;

  std::vector<float> el_lengths;
  el_lengths.reserve(waypoints.size());
  for (int i = 1; i < waypoints.size(); i++) {
    el_lengths.push_back(std::hypot(waypoints[i - 1].x - waypoints[i].x,
                                    waypoints[i - 1].y - waypoints[i].y));
  }

  std::vector<float> scaling;
  scaling.reserve(el_lengths.size());
  for (int i = 1; i < el_lengths.size(); i++) {
    scaling.push_back(el_lengths[i - 1] / el_lengths[i]);
  }

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> M =
      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic,
                    Eigen::RowMajor>::Zero(no_splines * 4, no_splines * 4);
  Eigen::VectorXf b_x = Eigen::VectorXf::Zero(no_splines * 4);
  Eigen::VectorXf b_y = Eigen::VectorXf::Zero(no_splines * 4);

  Eigen::Matrix<float, 4, 8, Eigen::RowMajor> template_M;
  template_M << 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 2, 3, 0,
      -1, 0, 0, 0, 0, 2, 6, 0, 0, -2, 0;

  for (int i = 0; i < no_splines; i++) {
    int j = i * 4;
    if (i < no_splines - 1) {
      M.block<4, 8>(j, j) = template_M;
      M(j + 3, j + 6) *= scaling[i] * scaling[i];
    } else {
      M.block<2, 4>(j, j) << 1, 0, 0, 0, 1, 1, 1, 1;
    }

    b_x(j) = waypoints[i].x;
    b_y(j) = waypoints[i].y;
    b_x(j + 1) = waypoints[i + 1].x;
    b_y(j + 1) = waypoints[i + 1].y;
  }

  // if not closed condition
  M(M.rows() - 2, 1) = 1;

  float el_length_s = el_lengths[0];

  b_x(b_x.size() - 2) = std::cos(psi_s) * el_length_s;
  b_y(b_y.size() - 2) = std::sin(psi_s) * el_length_s;

  // heading and point
  M.block<1, 4>(M.rows() - 1, M.cols() - 4) << 0, 1, 2, 3;

  float el_length_e = el_lengths.back();

  b_x(b_x.size() - 1) = std::cos(psi_e) * el_length_e;
  b_y(b_y.size() - 1) = std::sin(psi_e) * el_length_e;

  // TODO: This is a tridiagonal system that can be solved more efficiently
  Eigen::VectorXf xc_vec = M.colPivHouseholderQr().solve(b_x);
  Eigen::VectorXf yc_vec = M.colPivHouseholderQr().solve(b_y);

  // Optional correctness checks
  // assert(b_x.isApprox(M * xc_vec, 1.));
  // assert(b_y.isApprox(M * yc_vec, 1.));

  // TODO: This copies but it doesn't have to.
  xc = Eigen::Map<CoeffMatrix, Eigen::RowMajor>(xc_vec.data(), no_splines, 4);
  yc = Eigen::Map<CoeffMatrix, Eigen::RowMajor>(yc_vec.data(), no_splines, 4);
}

template <class T>
requires is_between<T, Point, PathPose> PointVector<T> CubicSpline::interpolate(
    float const step_size) {
  const auto no_ctrl_pts = xc.rows();

  std::vector<float> dists_cum;
  dists_cum.push_back(calc_spline_length_at(0));
  for (int i = 1; i < no_ctrl_pts; i++) {
    dists_cum.push_back(dists_cum.back() + calc_spline_length_at(i));
  }

  auto get_spline_length_at = [&](size_t i) {
    return (i == 0) ? dists_cum[0] : dists_cum[i] - dists_cum[i - 1];
  };

  const int no_interps_out = std::ceil(dists_cum.back() / step_size) + 1;

  const auto spacing =
      dists_cum.back() / static_cast<float>(no_interps_out - 1);
  auto get_dist_interp = [spacing](int i) { return spacing * i; };

  // waypoint info
  const auto no_waypoints =
      waypoints.size() - 2;  // exclude first and last points
  const auto total_spline_pts = no_interps_out + no_waypoints - 1;

  PointVector<T> path;
  path.reserve(total_spline_pts);

  for (int i = 0; i < total_spline_pts; i++) {
    // find spline that hosts current interpolation point
    // TODO: Can be made more efficient with a local search
    auto spline_idx =
        std::find_if(dists_cum.begin(), dists_cum.end(),
                     [&](float dist) { return get_dist_interp(i) < dist; }) -
        dists_cum.begin();

    // this might happen due to floating point error
    if (spline_idx == dists_cum.size()) {
      --spline_idx;
    }

    // get spline t value depending on progress within current element
    const float t = (spline_idx > 0)
                        ? (get_dist_interp(i) - dists_cum[spline_idx - 1]) /
                              get_spline_length_at(spline_idx)
                        : get_dist_interp(i) / get_spline_length_at(0);

    T p;

    // calculate coords
    p.x = xc(spline_idx, 0) + xc(spline_idx, 1) * t +
          xc(spline_idx, 2) * std::pow(t, 2) +
          xc(spline_idx, 3) * std::pow(t, 3);
    p.y = yc(spline_idx, 0) + yc(spline_idx, 1) * t +
          yc(spline_idx, 2) * std::pow(t, 2) +
          yc(spline_idx, 3) * std::pow(t, 3);

    if constexpr (std::derived_from<Pose, T>) {
      const float x_d = xc(spline_idx, 1) + 2 * xc(spline_idx, 2) * t +
                        3 * xc(spline_idx, 3) * std::pow(t, 2);
      const float y_d = yc(spline_idx, 1) + 2 * yc(spline_idx, 2) * t +
                        3 * yc(spline_idx, 3) * std::pow(t, 2);
      p.yaw = std::atan2(y_d, x_d);

      if constexpr (std::derived_from<PathPose, T>) {
        const float x_dd = 2 * xc(spline_idx, 2) + 6 * xc(spline_idx, 3) * t;
        const float y_dd = 2 * yc(spline_idx, 2) + 6 * yc(spline_idx, 3) * t;

        p.k = ((x_d * y_dd) - (y_d * x_dd)) /
              std::pow(std::pow(x_d, 2) + std::pow(y_d, 2), 1.5);
      }
    }

    path.push_back(p);
  }

  path.length = dists_cum.back();

  return path;
}

Path CubicSpline::interpolate_path(float const step_size) {
  return interpolate<PathPose>(step_size);
}
