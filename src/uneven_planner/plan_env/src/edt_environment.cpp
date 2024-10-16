/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/

#include <plan_env/edt_environment.h>

namespace uneven_planner {
/* ============================== edt_environment ==============================
 */
void EDTEnvironment::init() {
}

void EDTEnvironment::setMap(shared_ptr<SDFMap> map) {
  this->sdf_map_ = map;
  resolution_inv_ = 1 / sdf_map_->getResolution();
}


void EDTEnvironment::getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]) {
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
        dists[x][y] = sdf_map_->getDistance(pts[x][y]);
    }
  }
}

void EDTEnvironment::interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff,
                                         double& value, Eigen::Vector2d& grad) {
    // Bilinear interpolation
    double v0 = (1 - diff(0)) * values[0][0] + diff(0) * values[1][0]; // Interpolate along x for y=0
    double v1 = (1 - diff(0)) * values[0][1] + diff(0) * values[1][1]; // Interpolate along x for y=1
    value = (1 - diff(1)) * v0 + diff(1) * v1; // Final interpolation along y

    // Calculate gradients
    grad(0) = (values[1][0] - values[0][0]) * resolution_inv_; // Gradient in x
    grad(1) = (values[0][1] - values[0][0]) * (1 - diff(0)) + (values[1][1] - values[1][0]) * diff(0); // Gradient in y
    grad *= resolution_inv_; // Scale gradients by resolution_inv
}

void EDTEnvironment::evaluateEDTWithGrad(const Eigen::Vector2d& pos,
                                                                  double time, double& dist,
                                                                  Eigen::Vector2d& grad) {
  Eigen::Vector2d diff;
  Eigen::Vector2d sur_pts[2][2];
  sdf_map_->getSurroundPts(pos, sur_pts, diff);

  double dists[2][2];
  getSurroundDistance(sur_pts, dists);

  interpolateBilinear(dists, diff, dist, grad);
}

//double EDTEnvironment::evaluateCoarseEDT(Eigen::Vector3d& pos, double time) {
//  double d1 = sdf_map_->getDistance(pos);
//  if (time < 0.0) {
//    return d1;
//  } else {
//    double d2 = minDistToAllBox(pos, time);
//    return min(d1, d2);
//  }
//}

// EDTEnvironment::
}  // namespace fast_planner