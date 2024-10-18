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



#ifndef _SDF_MAP_H
#define _SDF_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <random>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#define logit(x) (log((x) / (1 - (x))))
using namespace std;

namespace uneven_planner {

// voxel hashing
    template<typename T>
    struct matrix_hash : std::unary_function<T, size_t> {
        std::size_t operator()(T const &matrix) const {
            size_t seed = 0;
            for (size_t i = 0; i < matrix.size(); ++i) {
                auto elem = *(matrix.data() + i);
                seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

// constant parameters

    struct MappingParameters {

        /* map properties */
        Eigen::Vector2d map_origin_, map_size_;
        Eigen::Vector2d map_min_boundary_, map_max_boundary_;  // map range in pos
        Eigen::Vector2i map_voxel_num_;                        // map range in index
        Eigen::Vector2i map_min_idx_, map_max_idx_;
        Eigen::Vector2d local_update_range_;
        double resolution_, resolution_inv_;
        double obstacles_inflation_;
        string frame_id_;
        int pose_type_;
        string map_input_;  // 1: pose+depth; 2: odom + cloud


        /* local map update and clear */
        double local_bound_inflate_;
        int local_map_margin_;

        /* visualization and computation time display */
        double esdf_slice_height_, visualization_truncate_height_, virtual_ceil_height_, ground_height_;
        bool show_esdf_time_, show_occ_time_;

        /* active mapping */
        double unknown_flag_;
    };

// intermediate mapping data for fusion, esdf

    struct MappingData {
        // main map data, occupancy of each voxel and Euclidean distance

        std::vector<double> occupancy_buffer_;
        std::vector<char> occupancy_buffer_neg;
        std::vector<char> occupancy_buffer_inflate_;
        std::vector<double> distance_buffer_;
        std::vector<double> distance_buffer_neg_;
        std::vector<double> distance_buffer_all_;
        std::vector<double> tmp_buffer1_;
        std::vector<double> tmp_buffer2_;

        // lidar position and pose data
        Eigen::Vector2d lidar_pos_, last_lidar_pos_;
        Eigen::Quaterniond lidar_q_, last_lidar_q_;


        // flags of map state

        bool occ_need_update_, local_updated_, esdf_need_update_;
        bool has_first_depth_;
        bool has_odom_, has_cloud_;


        // range of updating ESDF

        Eigen::Vector2i local_bound_min_, local_bound_max_;

        // computation time

        double fuse_time_, esdf_time_, max_fuse_time_, max_esdf_time_;
        int update_num_;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class SDFMap{
    public:
        SDFMap() {}

        ~SDFMap() {}

        enum {
            POSE_STAMPED = 1, ODOMETRY = 2, INVALID_IDX = -10000
        };

        // occupancy map management
        void resetBuffer();

        void resetBuffer(Eigen::Vector2d min, Eigen::Vector2d max);

        void initMap(ros::NodeHandle &nh);

        inline void posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id);

        inline void indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos);

        inline int toAddress(const Eigen::Vector2i &id);

        inline int toAddress(int &x, int &y);

        inline bool isInMap(const Eigen::Vector2d &pos);

        inline bool isInMap(const Eigen::Vector2i &idx);

        inline void boundIndex(Eigen::Vector2i &id);

        // distance field management
        inline double getDistance(const Eigen::Vector2d &pos);

        inline double getDistance(const Eigen::Vector2i &id);

        inline double getDistWithGradBilinear(Eigen::Vector2d pos, Eigen::Vector2d &grad);

        void getSurroundPts(const Eigen::Vector2d &pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d &diff);

        void getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]);

        void interpolateBilinear(double values[2][2], const Eigen::Vector2d &diff,
                                 double &value, Eigen::Vector2d &grad);

        void evaluateEDTWithGrad(const Eigen::Vector2d &pos, double time,
                                 double &dist, Eigen::Vector2d &grad);

        void updateESDF2d();

        void publishMap();

        void publishMapInflate(bool all_info = false);

        void getRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size);

        inline int getXYNum() { return mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1); }

        double getResolution();


        typedef std::shared_ptr<SDFMap> Ptr;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
        MappingParameters mp_;
        MappingData md_;

        template<typename F_get_val, typename F_set_val>
        void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

        // get depth image and camera pose
        void cloudCallback(const sensor_msgs::LaserScanConstPtr &scan);

        void odomCallback(const nav_msgs::OdometryConstPtr &odom);

        // update occupancy by raycasting, and update ESDF
        void updateESDFCallback(const ros::TimerEvent & /*event*/);

        void visCallback(const ros::TimerEvent & /*event*/);

        Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);

        ros::NodeHandle node_;
        shared_ptr<message_filters::Subscriber < geometry_msgs::PoseStamped>> pose_sub_;
        shared_ptr<message_filters::Subscriber < nav_msgs::Odometry>> odom_sub_;

        ros::Subscriber indep_depth_sub_, indep_odom_sub_, indep_pose_sub_, indep_cloud_sub_;
        ros::Publisher map_pub_, esdf_pub_, map_inf_pub_, update_range_pub_;
        ros::Timer esdf_timer_, vis_timer_;

        //
        uniform_real_distribution<double> rand_noise_;
        normal_distribution<double> rand_noise2_;
        default_random_engine eng_;
    };

/* ============================== definition of inline function
 * ============================== */

    inline int SDFMap::toAddress(const Eigen::Vector2i &id) {
        return id(1) * mp_.map_voxel_num_(0) + id(0);
    }

    inline int SDFMap::toAddress(int &x, int &y) {
        return y * mp_.map_voxel_num_(0) + x;
    }

    inline void SDFMap::boundIndex(Eigen::Vector2i &id) {
        Eigen::Vector2i id1;
        id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
        id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
        id = id1;
    }

    inline double SDFMap::getDistance(const Eigen::Vector2d &pos) {
        Eigen::Vector2i id;
        posToIndex(pos, id);
        boundIndex(id);

        return md_.distance_buffer_all_[toAddress(id)];
    }

    inline double SDFMap::getDistance(const Eigen::Vector2i &id) {
        Eigen::Vector2i id1 = id;
        boundIndex(id1);
        return md_.distance_buffer_all_[toAddress(id1)];
    }

    inline bool SDFMap::isInMap(const Eigen::Vector2d &pos) {
        if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4) {
            return false;
        }
        if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4) {
            return false;
        }
        return true;
    }

    inline bool SDFMap::isInMap(const Eigen::Vector2i &idx) {
        if (idx(0) < 0 || idx(1) < 0) {
            return false;
        }
        if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1) {
            return false;
        }
        return true;
    }

    inline void SDFMap::posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id) {
        for (int i = 0; i < 2; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
    }

    inline void SDFMap::indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos) {
        for (int i = 0; i < 2; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
    }
}
#endif