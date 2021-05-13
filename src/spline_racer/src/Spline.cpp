#include "Spline.hpp"

#include <iterator>

// Optimized version of https://github.com/TUMFTM/trajectory_planning_helpers
// See the included LGPLv3 license at the end of this source file.
// The license attached at the end of this source file applies ONLY
// to this source file.
// All source files excluding this one retain the license described in
// the LICENSE file at the root of this repository.


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


/*
                   GNU LESSER GENERAL PUBLIC LICENSE
                       Version 3, 29 June 2007

 Copyright (C) 2007 Free Software Foundation, Inc. <https://fsf.org/>
 Everyone is permitted to copy and distribute verbatim copies
 of this license document, but changing it is not allowed.


  This version of the GNU Lesser General Public License incorporates
the terms and conditions of version 3 of the GNU General Public
License, supplemented by the additional permissions listed below.

  0. Additional Definitions.

  As used herein, "this License" refers to version 3 of the GNU Lesser
General Public License, and the "GNU GPL" refers to version 3 of the GNU
General Public License.

  "The Library" refers to a covered work governed by this License,
other than an Application or a Combined Work as defined below.

  An "Application" is any work that makes use of an interface provided
by the Library, but which is not otherwise based on the Library.
Defining a subclass of a class defined by the Library is deemed a mode
of using an interface provided by the Library.

  A "Combined Work" is a work produced by combining or linking an
Application with the Library.  The particular version of the Library
with which the Combined Work was made is also called the "Linked
Version".

  The "Minimal Corresponding Source" for a Combined Work means the
Corresponding Source for the Combined Work, excluding any source code
for portions of the Combined Work that, considered in isolation, are
based on the Application, and not on the Linked Version.

  The "Corresponding Application Code" for a Combined Work means the
object code and/or source code for the Application, including any data
and utility programs needed for reproducing the Combined Work from the
Application, but excluding the System Libraries of the Combined Work.

  1. Exception to Section 3 of the GNU GPL.

  You may convey a covered work under sections 3 and 4 of this License
without being bound by section 3 of the GNU GPL.

  2. Conveying Modified Versions.

  If you modify a copy of the Library, and, in your modifications, a
facility refers to a function or data to be supplied by an Application
that uses the facility (other than as an argument passed when the
facility is invoked), then you may convey a copy of the modified
version:

   a) under this License, provided that you make a good faith effort to
   ensure that, in the event an Application does not supply the
   function or data, the facility still operates, and performs
   whatever part of its purpose remains meaningful, or

   b) under the GNU GPL, with none of the additional permissions of
   this License applicable to that copy.

  3. Object Code Incorporating Material from Library Header Files.

  The object code form of an Application may incorporate material from
a header file that is part of the Library.  You may convey such object
code under terms of your choice, provided that, if the incorporated
material is not limited to numerical parameters, data structure
layouts and accessors, or small macros, inline functions and templates
(ten or fewer lines in length), you do both of the following:

   a) Give prominent notice with each copy of the object code that the
   Library is used in it and that the Library and its use are
   covered by this License.

   b) Accompany the object code with a copy of the GNU GPL and this license
   document.

  4. Combined Works.

  You may convey a Combined Work under terms of your choice that,
taken together, effectively do not restrict modification of the
portions of the Library contained in the Combined Work and reverse
engineering for debugging such modifications, if you also do each of
the following:

   a) Give prominent notice with each copy of the Combined Work that
   the Library is used in it and that the Library and its use are
   covered by this License.

   b) Accompany the Combined Work with a copy of the GNU GPL and this license
   document.

   c) For a Combined Work that displays copyright notices during
   execution, include the copyright notice for the Library among
   these notices, as well as a reference directing the user to the
   copies of the GNU GPL and this license document.

   d) Do one of the following:

       0) Convey the Minimal Corresponding Source under the terms of this
       License, and the Corresponding Application Code in a form
       suitable for, and under terms that permit, the user to
       recombine or relink the Application with a modified version of
       the Linked Version to produce a modified Combined Work, in the
       manner specified by section 6 of the GNU GPL for conveying
       Corresponding Source.

       1) Use a suitable shared library mechanism for linking with the
       Library.  A suitable mechanism is one that (a) uses at run time
       a copy of the Library already present on the user's computer
       system, and (b) will operate properly with a modified version
       of the Library that is interface-compatible with the Linked
       Version.

   e) Provide Installation Information, but only if you would otherwise
   be required to provide such information under section 6 of the
   GNU GPL, and only to the extent that such information is
   necessary to install and execute a modified version of the
   Combined Work produced by recombining or relinking the
   Application with a modified version of the Linked Version. (If
   you use option 4d0, the Installation Information must accompany
   the Minimal Corresponding Source and Corresponding Application
   Code. If you use option 4d1, you must provide the Installation
   Information in the manner specified by section 6 of the GNU GPL
   for conveying Corresponding Source.)

  5. Combined Libraries.

  You may place library facilities that are a work based on the
Library side by side in a single library together with other library
facilities that are not Applications and are not covered by this
License, and convey such a combined library under terms of your
choice, if you do both of the following:

   a) Accompany the combined library with a copy of the same work based
   on the Library, uncombined with any other library facilities,
   conveyed under the terms of this License.

   b) Give prominent notice with the combined library that part of it
   is a work based on the Library, and explaining where to find the
   accompanying uncombined form of the same work.

  6. Revised Versions of the GNU Lesser General Public License.

  The Free Software Foundation may publish revised and/or new versions
of the GNU Lesser General Public License from time to time. Such new
versions will be similar in spirit to the present version, but may
differ in detail to address new problems or concerns.

  Each version is given a distinguishing version number. If the
Library as you received it specifies that a certain numbered version
of the GNU Lesser General Public License "or any later version"
applies to it, you have the option of following the terms and
conditions either of that published version or of any later version
published by the Free Software Foundation. If the Library as you
received it does not specify a version number of the GNU Lesser
General Public License, you may choose any version of the GNU Lesser
General Public License ever published by the Free Software Foundation.

  If the Library as you received it specifies that a proxy can decide
whether future versions of the GNU Lesser General Public License shall
apply, that proxy's public statement of acceptance of any version is
permanent authorization for you to choose that version for the
Library.
*/
