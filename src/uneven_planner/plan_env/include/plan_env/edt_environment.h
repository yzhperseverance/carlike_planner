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



#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <utility>

#include <plan_env/sdf_map.h>

using std::cout;
using std::endl;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace uneven_planner {
class EDTEnvironment {
private:
  /* data */
  double resolution_inv_;
public:
  EDTEnvironment(/* args */) {
  }
  ~EDTEnvironment() {
  }

  SDFMap::Ptr sdf_map_;

  void init();
  void setMap(SDFMap::Ptr map);

  void getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]);
  void interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff,
                                                     double& value, Eigen::Vector2d& grad);
  void evaluateEDTWithGrad(const Eigen::Vector2d& pos, double time,
                                                    double& dist, Eigen::Vector2d& grad);

  void getMapRegion(Eigen::Vector2d& ori, Eigen::Vector2d& size) {
    sdf_map_->getRegion(ori, size);
  }

  typedef shared_ptr<EDTEnvironment> Ptr;
};

}  // namespace fast_planner

#endif