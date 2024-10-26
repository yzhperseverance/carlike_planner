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



#include "plan_env/sdf_map.h"

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]
namespace uneven_planner {

    void SDFMap::initMap(ros::NodeHandle &nh) {
        node_ = nh;

        /* get parameter */
        double x_size, y_size;
        node_.param("sdf_map/resolution", mp_.resolution_, -1.0);
        node_.param("sdf_map/map_size_x", x_size, -1.0);
        node_.param("sdf_map/map_size_y", y_size, -1.0);
        node_.param("sdf_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
        node_.param("sdf_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
        node_.param("sdf_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);


        node_.param("sdf_map/esdf_slice_height", mp_.esdf_slice_height_, -0.1);
        node_.param("sdf_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);
        node_.param("sdf_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);

        node_.param("sdf_map/show_occ_time", mp_.show_occ_time_, false);
        node_.param("sdf_map/show_esdf_time", mp_.show_esdf_time_, false);
        node_.param("sdf_map/pose_type", mp_.pose_type_, 1);

        node_.param("sdf_map/frame_id", mp_.frame_id_, string("world"));
        node_.param("sdf_map/local_bound_inflate", mp_.local_bound_inflate_, 1.0);
        node_.param("sdf_map/local_map_margin", mp_.local_map_margin_, 1);

        node_.param("sdf_map/lidar_pos_x", mp_.lidar_pos_x, 0.0);
        node_.param("sdf_map/lidar_pos_y", mp_.lidar_pos_y, 0.0);

        mp_.local_bound_inflate_ = max(mp_.resolution_, mp_.local_bound_inflate_);
        mp_.resolution_inv_ = 1 / mp_.resolution_;
        mp_.map_origin_ = Eigen::Vector2d(-x_size / 2.0, -y_size / 2.0);
        mp_.map_size_ = Eigen::Vector2d(x_size, y_size);


        for (int i = 0; i < 2; ++i) mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

        mp_.map_min_boundary_ = mp_.map_origin_;
        mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

        mp_.map_min_idx_ = Eigen::Vector2i::Zero();
        mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector2i::Ones();

        // initialize data buffers

        int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1);

        md_.occupancy_buffer_neg = vector<char>(buffer_size, 0);
        md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

        md_.distance_buffer_ = vector<double>(buffer_size, 10000);
        md_.distance_buffer_neg_ = vector<double>(buffer_size, 10000);
        md_.distance_buffer_all_ = vector<double>(buffer_size, 10000);

        md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
        md_.tmp_buffer2_ = vector<double>(buffer_size, 0);

        /* init callback */


        // use odometry and point cloud
        odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "odom", 100));
        cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "cloud", 100));
        sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(
                SyncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));
        sync_cloud_odom_->registerCallback(boost::bind(&SDFMap::cloudOdomCallback, this, _1, _2));

//        indep_cloud_sub_ =
//                node_.subscribe<sensor_msgs::LaserScan>("cloud", 10, &SDFMap::cloudCallback, this);
//        indep_odom_sub_ =
//                node_.subscribe<nav_msgs::Odometry>("odom", 10, &SDFMap::odomCallback, this);

        esdf_timer_ = node_.createTimer(ros::Duration(0.05), &SDFMap::updateESDFCallback, this);
        vis_timer_ = node_.createTimer(ros::Duration(0.05), &SDFMap::visCallback, this);

        map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy", 10);
        map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_inflate", 10);
        esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);

        md_.occ_need_update_ = false;
        md_.local_updated_ = false;
        md_.esdf_need_update_ = false;
        md_.has_first_depth_ = false;
        md_.has_odom_ = false;
        md_.has_cloud_ = false;

        md_.esdf_time_ = 0.0;
        md_.fuse_time_ = 0.0;
        md_.update_num_ = 0;
        md_.max_esdf_time_ = 0.0;
        md_.max_fuse_time_ = 0.0;

        rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
        rand_noise2_ = normal_distribution<double>(0, 0.2);
        random_device rd;
        eng_ = default_random_engine(rd());

    }

    void SDFMap::resetBuffer() {
        Eigen::Vector2d min_pos = mp_.map_min_boundary_;
        Eigen::Vector2d max_pos = mp_.map_max_boundary_;

        resetBuffer(min_pos, max_pos);

        md_.local_bound_min_ = Eigen::Vector2i::Zero();
        md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector2i::Ones();
    }

    void SDFMap::resetBuffer(Eigen::Vector2d min_pos, Eigen::Vector2d max_pos) {

        Eigen::Vector2i min_id, max_id;
        posToIndex(min_pos, min_id);
        posToIndex(max_pos, max_id);

        boundIndex(min_id);
        boundIndex(max_id);

        /* reset occ and dist buffer */
        for (int x = min_id(0); x <= max_id(0); ++x)
            for (int y = min_id(1); y <= max_id(1); ++y) {
                md_.occupancy_buffer_inflate_[toAddress(x, y)] = 0;
                md_.distance_buffer_[toAddress(x, y)] = 10000;
            }
    }

    template<typename F_get_val, typename F_set_val>
    void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
        int v[mp_.map_voxel_num_(dim)];
        double z[mp_.map_voxel_num_(dim) + 1];

        int k = start;
        v[start] = start;
        z[start] = -std::numeric_limits<double>::max();
        z[start + 1] = std::numeric_limits<double>::max();

        for (int q = start + 1; q <= end; q++) {
            k++;
            double s;

            do {
                k--;
                s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
            } while (s <= z[k]);

            k++;

            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<double>::max();
        }

        k = start;

        for (int q = start; q <= end; q++) {
            while (z[k + 1] < q) k++;
            double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
            f_set_val(q, val);
        }
    }

// 分别在三个方向做一次fillESDF
    void SDFMap::updateESDF2d() {
        Eigen::Vector2i min_esdf = md_.local_bound_min_;
        Eigen::Vector2i max_esdf = md_.local_bound_max_;

        /* ========== compute positive DT ========== */

        for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
            fillESDF(
                    [&](int y) {
                        return md_.occupancy_buffer_inflate_[toAddress(x, y)] == 1 ?
                               0 :
                               std::numeric_limits<double>::max();
                    },
                    [&](int y, double val) { md_.tmp_buffer1_[toAddress(x, y)] = val; }, min_esdf[1],
                    max_esdf[1], 1);

        }

        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
            fillESDF(
                    [&](int x) { return md_.tmp_buffer1_[toAddress(x, y)]; },
                    [&](int x, double val) {
                        md_.distance_buffer_[toAddress(x, y)] = mp_.resolution_ * std::sqrt(val);
                    },
                    min_esdf[0], max_esdf[0], 0);

        }

        /* ========== compute negative distance ========== */
        for (int x = min_esdf(0); x <= max_esdf(0); ++x)
            for (int y = min_esdf(1); y <= max_esdf(1); ++y) {
                int idx = toAddress(x, y);
                if (md_.occupancy_buffer_inflate_[idx] == 0) {
                    md_.occupancy_buffer_neg[idx] = 1;

                } else if (md_.occupancy_buffer_inflate_[idx] == 1) {
                    md_.occupancy_buffer_neg[idx] = 0;
                } else {
                    ROS_ERROR("what?");
                }
            }

        ros::Time t1, t2;

        for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
            fillESDF(
                    [&](int y) {
                        return md_.occupancy_buffer_neg[toAddress(x, y)] == 1 ?
                               0 :
                               std::numeric_limits<double>::max();
                    },
                    [&](int y, double val) { md_.tmp_buffer1_[toAddress(x, y)] = val; }, min_esdf[1],
                    max_esdf[1], 1);

        }

        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
            fillESDF(
                    [&](int x) { return md_.tmp_buffer1_[toAddress(x, y)]; },
                    [&](int x, double val) {
                        md_.distance_buffer_neg_[toAddress(x, y)] = mp_.resolution_ * std::sqrt(val);
                    },
                    min_esdf[0], max_esdf[0], 0);

        }
        /* ========== combine pos and neg DT ========== */
        for (int x = min_esdf(0); x <= max_esdf(0); ++x)
            for (int y = min_esdf(1); y <= max_esdf(1); ++y) {

                int idx = toAddress(x, y);
                md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];

                if (md_.distance_buffer_neg_[idx] > 0.0)
                    md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
            }
    }

    void SDFMap::visCallback(const ros::TimerEvent & /*event*/) {
        publishMap();
        publishMapInflate(false);
    }


    void SDFMap::updateESDFCallback(const ros::TimerEvent & /*event*/) {
        if (!md_.esdf_need_update_) return;

        /* esdf */
        ros::Time t1, t2;
        t1 = ros::Time::now();

        updateESDF2d();

        t2 = ros::Time::now();

        md_.esdf_time_ += (t2 - t1).toSec();
        md_.max_esdf_time_ = max(md_.max_esdf_time_, (t2 - t1).toSec());

        if (mp_.show_esdf_time_)
            ROS_WARN("ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
                     md_.esdf_time_ / md_.update_num_, md_.max_esdf_time_);

        md_.esdf_need_update_ = false;
    }

//    void SDFMap::odomCallback(const nav_msgs::OdometryConstPtr &odom) {
//        // base_link->laser_link
//        // 发布base_link在map系的坐标
//        buff_mutex_.lock();
//        md_.lidar_pos_(0) = odom->pose.pose.position.x + mp_.lidar_pos_x;
//        md_.lidar_pos_(1) = odom->pose.pose.position.y + mp_.lidar_pos_y;
//        buff_mutex_.unlock();
//        md_.has_odom_ = true;
//    }

    void SDFMap::cloudOdomCallback(const sensor_msgs::LaserScanConstPtr &scan,
                                   const nav_msgs::OdometryConstPtr& odom) {
        // 把ros中激光雷达的scan转化成点云
        laser_geometry::LaserProjection projector;

        sensor_msgs::PointCloud2 cloud;
        ros::Time time;

        cloud.header = scan->header;
        try {
            //time = scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment);

            if(!listener.waitForTransform("map", scan->header.frame_id, scan->header.stamp, ros::Duration(1.0))){
                std::cout << "No laser tf!" << std::endl;
                return;
            }
            projector.transformLaserScanToPointCloud("map", *scan, cloud, listener);

        } catch (tf::TransformException &e) {
            ROS_ERROR("%s", e.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> latest_cloud;
        pcl::fromROSMsg(cloud, latest_cloud);
        md_.has_cloud_ = true;
        md_.has_odom_ = true;
        // map系下雷达坐标 也可以使用定位模块的位姿
        md_.lidar_pos_(0) = odom->pose.pose.position.x + mp_.lidar_pos_x;
        md_.lidar_pos_(1) = odom->pose.pose.position.y + mp_.lidar_pos_y;

        if (latest_cloud.points.size() == 0) return;

        if (isnan(md_.lidar_pos_(0)) || isnan(md_.lidar_pos_(1))) return;

        // local地图更新的范围
        this->resetBuffer(md_.lidar_pos_ - mp_.local_update_range_,
                          md_.lidar_pos_ + mp_.local_update_range_);

        pcl::PointXYZ pt;
        Eigen::Vector2d p2d, p2d_inf;

        int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
        int inf_step_z = 1;

        double max_x, max_y, min_x, min_y;

        min_x = mp_.map_max_boundary_(0);
        min_y = mp_.map_max_boundary_(1);

        max_x = mp_.map_min_boundary_(0);
        max_y = mp_.map_min_boundary_(1);

        for (size_t i = 0; i < latest_cloud.points.size(); ++i) {
            pt = latest_cloud.points[i];
            p2d(0) = pt.x, p2d(1) = pt.y;

            // point inside update range
            Eigen::Vector2d devi = p2d - md_.lidar_pos_; // laser_link系下的点云坐标
            Eigen::Vector2i inf_pt;

            if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1)) {

                // inflate the point
                for (int x = -inf_step; x <= inf_step; ++x)
                    for (int y = -inf_step; y <= inf_step; ++y) {
                        p2d_inf(0) = pt.x + x * mp_.resolution_;
                        p2d_inf(1) = pt.y + y * mp_.resolution_;

                        max_x = max(max_x, p2d_inf(0));
                        max_y = max(max_y, p2d_inf(1));

                        min_x = min(min_x, p2d_inf(0));
                        min_y = min(min_y, p2d_inf(1));

                        posToIndex(p2d_inf, inf_pt);

                        if (!isInMap(inf_pt)) continue;

                        int idx_inf = toAddress(inf_pt);

                        md_.occupancy_buffer_inflate_[idx_inf] = 1;
                    }
            }
        }

        min_x = min(min_x, md_.lidar_pos_(0));
        min_y = min(min_y, md_.lidar_pos_(1));

        max_x = max(max_x, md_.lidar_pos_(0));
        max_y = max(max_y, md_.lidar_pos_(1));

        // local_bound是esdf更新的范围，根据点云范围变化
        posToIndex(Eigen::Vector2d(max_x, max_y), md_.local_bound_max_);
        posToIndex(Eigen::Vector2d(min_x, min_y), md_.local_bound_min_);

        boundIndex(md_.local_bound_min_);
        boundIndex(md_.local_bound_max_);

        md_.esdf_need_update_ = true;

    }

    void SDFMap::publishMap() {

        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> cloud;

        Eigen::Vector2i min_cut = md_.local_bound_min_;
        Eigen::Vector2i max_cut = md_.local_bound_max_;

        int lmm = mp_.local_map_margin_ / 2;
        min_cut -= Eigen::Vector2i(lmm, lmm);
        max_cut += Eigen::Vector2i(lmm, lmm);

        boundIndex(min_cut);
        boundIndex(max_cut);

        for (int x = min_cut(0); x <= max_cut(0); ++x)
            for (int y = min_cut(1); y <= max_cut(1); ++y) {
                if (md_.occupancy_buffer_inflate_[toAddress(x, y)] == 0) continue;

                Eigen::Vector2d pos;
                indexToPos(Eigen::Vector2i(x, y), pos);

                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = 0;
                cloud.push_back(pt);
            }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = mp_.frame_id_;
        sensor_msgs::PointCloud2 cloud_msg;

        pcl::toROSMsg(cloud, cloud_msg);
        map_pub_.publish(cloud_msg);
    }

    void SDFMap::publishMapInflate(bool all_info) {
        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> cloud;

        Eigen::Vector2i min_cut = md_.local_bound_min_;
        Eigen::Vector2i max_cut = md_.local_bound_max_;

        if (all_info) {
            int lmm = mp_.local_map_margin_;
            min_cut -= Eigen::Vector2i(lmm, lmm);
            max_cut += Eigen::Vector2i(lmm, lmm);
        }

        boundIndex(min_cut);
        boundIndex(max_cut);

        for (int x = min_cut(0); x <= max_cut(0); ++x)
            for (int y = min_cut(1); y <= max_cut(1); ++y) {
                if (md_.occupancy_buffer_inflate_[toAddress(x, y)] == 0) continue;

                Eigen::Vector2d pos;
                indexToPos(Eigen::Vector2i(x, y), pos);

                pt.x = pos(0);
                pt.y = pos(1);
                pt.z = 0;
                cloud.push_back(pt);
            }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = mp_.frame_id_;
        sensor_msgs::PointCloud2 cloud_msg;

        pcl::toROSMsg(cloud, cloud_msg);
        map_inf_pub_.publish(cloud_msg);

        // ROS_INFO("pub map");
    }


    double SDFMap::getResolution() { return mp_.resolution_; }


    void SDFMap::getRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size) {
        ori = mp_.map_origin_, size = mp_.map_size_;
    }

    void SDFMap::getSurroundPts(const Eigen::Vector2d &pos, Eigen::Vector2d pts[2][2],
                                Eigen::Vector2d &diff) {
        if (!isInMap(pos)) {
            // cout << "pos invalid for interpolation." << endl;
        }

        /* interpolation position */
        Eigen::Vector2d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector2d::Ones();
        Eigen::Vector2i idx;
        Eigen::Vector2d idx_pos;

        posToIndex(pos_m, idx);
        indexToPos(idx, idx_pos);
        // 当前位置与栅格中心位置之间的差
        diff = (pos - idx_pos) * mp_.resolution_inv_;

        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
                {
                    Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
                    Eigen::Vector2d current_pos;
                    indexToPos(current_idx, current_pos);
                    pts[x][y] = current_pos;
                }
            }
        }
    }

    void SDFMap::getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]) {
        for (int x = 0; x < 2; x++) {
            for (int y = 0; y < 2; y++) {
                dists[x][y] = this->getDistance(pts[x][y]);
            }
        }
    }


    void SDFMap::interpolateBilinear(double values[2][2], const Eigen::Vector2d &diff,
                                     double &value, Eigen::Vector2d &grad) {
        // Bilinear interpolation
        double v0 = (1 - diff(0)) * values[0][0] + diff(0) * values[1][0]; // Interpolate along x for y=0
        double v1 = (1 - diff(0)) * values[0][1] + diff(0) * values[1][1]; // Interpolate along x for y=1
        value = (1 - diff(1)) * v0 + diff(1) * v1; // Final interpolation along y
        // Calculate gradients
        grad(0) = (values[1][0] - values[0][0]) * mp_.resolution_inv_; // Gradient in x
        grad(1) = (values[0][1] - values[0][0]) * (1 - diff(0)) +
                  (values[1][1] - values[1][0]) * diff(0); // Gradient in y

        grad(1) *= mp_.resolution_inv_; // Scale gradients by resolution_inv
    }

    void SDFMap::evaluateEDTWithGrad(const Eigen::Vector2d &pos,
                                     double time, double &dist,
                                     Eigen::Vector2d &grad) {
        Eigen::Vector2d diff;
        Eigen::Vector2d sur_pts[2][2];
        getSurroundPts(pos, sur_pts, diff);

        double dists[2][2];
        getSurroundDistance(sur_pts, dists);

        interpolateBilinear(dists, diff, dist, grad);
    }
} // namespace

