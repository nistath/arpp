#include <Eigen/Dense>

#include "Geometry.hpp"

class PathInterpolator {
 public:
  virtual Path interpolate_path(float step_size) = 0;
};

class CubicSpline : public PathInterpolator {
 private:
  using CoeffMatrix = Eigen::Matrix<float, Eigen::Dynamic, 4, Eigen::RowMajor>;
  CoeffMatrix xc;
  CoeffMatrix yc;

  std::vector<Point> waypoints;

 public:
  CubicSpline(std::vector<Point> waypoints, float psi_start, float psi_end);
  Path interpolate_path(float step_size) override final;

  template <class T>
  requires is_between<T, Point, PathPose> PointVector<T> interpolate(
      float const step_size);
};
