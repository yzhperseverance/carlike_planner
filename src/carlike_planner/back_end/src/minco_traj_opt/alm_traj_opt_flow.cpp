//
// Created by yzh on 24-10-12.
//
#include "back_end/minco_traj_opt/alm_traj_opt_flow.h"

namespace carlike_planner {
    AlmTrajOptFlow::AlmTrajOptFlow(ros::NodeHandle &nh) {
        alm_traj_ptr_ = std::make_shared<ALMTrajOpt>(nh);
        se2_pub = nh.advertise<nav_msgs::Path>("/alm/se2_path", 1);
        se3_pub = nh.advertise<nav_msgs::Path>("/alm/se3_path", 1);
        yaw_pub = nh.advertise<visualization_msgs::MarkerArray>("/alm/yaw_path", 1);
        nh.getParam("alm_traj_opt/max_vel", max_vel_);

    }


    void AlmTrajOptFlow::Run(const std::vector<Eigen::Vector3d> &init_path) {
        if(!alm_traj_ptr_->sdf_map){
            std::cout << "No SDF Map" << std::endl;
            return;
        }
        if(!alm_traj_ptr_->sdf_map->md_.has_cloud_){
            std::cout << "No Cloud" << std::endl;
            return;
        }

        GetLocalInitPath(init_path);
        SmoothYaw();
        // init solution
        Eigen::Matrix<double, 2, 3> init_xy, end_xy;
        Eigen::Vector3d init_yaw, end_yaw;
        Eigen::MatrixXd inner_xy;
        Eigen::VectorXd inner_yaw;
        double total_time;

        init_xy << init_path_[0].x(), 0.0, 0.0, \
               init_path_[0].y(), 0.0, 0.0;
        end_xy << init_path_.back().x(), 0.0, 0.0, \
               init_path_.back().y(), 0.0, 0.0;
        init_yaw << init_path_[0].z(), 0.0, 0.0;
        end_yaw << init_path_.back().z(), 0.0, 0.0;

        // 为什么要乘0.05？？？可能是因为这是约束条件，表示方向即可，所以乘了一个系数让他缩小
        // 这就是控制开始位姿和结束位姿的方法！？
        init_xy.col(1) << 0.05 * cos(init_yaw(0)), 0.05 * sin(init_yaw(0));
        end_xy.col(1) << 0.05 * cos(end_yaw(0)), 0.05 * sin(end_yaw(0));

        GetInnerPoint(inner_xy, inner_yaw, total_time);

        alm_traj_ptr_->optimizeSE2Traj(init_xy, end_xy, inner_xy, \
                    init_yaw, end_yaw, inner_yaw, total_time);
        SE2Trajectory back_end_traj = alm_traj_ptr_->getTraj();
        PublishSE2Traj(back_end_traj);
        //visualizeYaw(back_end_traj);
        //PublishSE3Traj(back_end_traj);
    }

    void AlmTrajOptFlow::SmoothYaw() {
        double dyaw;
        for (size_t i = 0; i < init_path_.size(); i++) {
            dyaw = init_path_[i + 1].z() - init_path_[i].z();
            while (dyaw >= M_PI / 2) {
                init_path_[i + 1].z() -= M_PI * 2;
                dyaw = init_path_[i + 1].z() - init_path_[i].z();
            }
            while (dyaw <= -M_PI / 2) {
                init_path_[i + 1].z() += M_PI * 2;
                dyaw = init_path_[i + 1].z() - init_path_[i].z();
            }
        }

    }

    void AlmTrajOptFlow::GetLocalInitPath(const std::vector<Eigen::Vector3d> &init_path){
        init_path_.clear();
        for(auto &point: init_path){
            Eigen::Vector2d pos = point.head(2);
            if(!alm_traj_ptr_->sdf_map->isInMap(pos)) break;
            init_path_.push_back(point);
        }
    }
    void AlmTrajOptFlow::GetInnerPoint(Eigen::MatrixXd &inner_xy, Eigen::VectorXd &inner_yaw, double &total_time) {
        double temp_len_yaw = 0.0;
        double temp_len_pos = 0.0;
        double total_len = 0.0;
        double piece_len = 0.3;
        double piece_len_yaw = piece_len / 2.0; // 意思就是yaw插入的点更细致呗
        std::vector<Eigen::Vector2d> inner_xy_node;
        std::vector<double> inner_yaw_node;

        // 在一条初始化路径（init_path）中插入一些内部节点
        for (int k = 0; k < init_path_.size() - 1; k++) {
            double temp_seg = (init_path_[k + 1] - init_path_[k]).head(2).norm();
            temp_len_yaw += temp_seg;
            temp_len_pos += temp_seg;
            total_len += temp_seg;

            if (temp_len_yaw > piece_len_yaw) {
                double temp_yaw = init_path_[k].z() + (1.0 - (temp_len_yaw - piece_len_yaw) / temp_seg) *
                                                      (init_path_[k + 1] - init_path_[k]).z();
                inner_yaw_node.push_back(temp_yaw);
                temp_len_yaw -= piece_len_yaw;
            }
            if (temp_len_pos > piece_len) {
                Eigen::Vector3d temp_node = init_path_[k] + (1.0 - (temp_len_pos - piece_len) / temp_seg) *
                                                            (init_path_[k + 1] - init_path_[k]);

                inner_xy_node.emplace_back(temp_node.head(2));
                inner_yaw_node.push_back(temp_node.z()); // 这俩还不完全分开？？这还插入yaw？
                temp_len_pos -= piece_len;
            }
        }

        total_time = total_len / max_vel_ * 1.2;
        inner_xy.resize(2, inner_xy_node.size());
        inner_yaw.resize(inner_yaw_node.size());
        for (int i = 0; i < inner_xy_node.size(); i++) {
            inner_xy.col(i) = inner_xy_node[i];
        }
        for (int i = 0; i < inner_yaw_node.size(); i++) {
            inner_yaw(i) = inner_yaw_node[i];
        }

    }

    void AlmTrajOptFlow::PublishSE2Traj(const SE2Trajectory& traj)
    {
        nav_msgs::Path back_end_path;
        back_end_path.header.frame_id = "world";
        back_end_path.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped p;
        for (double t=0.0; t < traj.getTotalDuration(); t+=0.03)
        {
            Eigen::Vector2d pos = traj.getPos(t);
            double yaw = traj.getAngle(t);
            p.pose.position.x = pos(0);
            p.pose.position.y = pos(1);
            p.pose.position.z = 0.0;
            p.pose.orientation.w = cos(yaw/2.0);
            p.pose.orientation.x = 0.0;
            p.pose.orientation.y = 0.0;
            p.pose.orientation.z = sin(yaw/2.0);
            back_end_path.poses.push_back(p);
        }
        Eigen::Vector2d pos = traj.getPos(traj.getTotalDuration());
        double yaw = traj.getAngle(traj.getTotalDuration());
        p.pose.position.x = pos(0);
        p.pose.position.y = pos(1);
        p.pose.position.z = 0.0;
        p.pose.orientation.w = cos(yaw/2.0);
        p.pose.orientation.x = 0.0;
        p.pose.orientation.y = 0.0;
        p.pose.orientation.z = sin(yaw/2.0);
        back_end_path.poses.push_back(p);

        se2_pub.publish(back_end_path);
    }


    void AlmTrajOptFlow::PublishSE3Traj(const SE2Trajectory& traj)
    {
        nav_msgs::Path back_end_path;
        back_end_path.header.frame_id = "world";
        back_end_path.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped p;
        for (double t=0.0; t<traj.getTotalDuration(); t+=0.03)
        {
            Eigen::Vector3d pos = traj.getNormSE2Pos(t);
            Eigen::Vector3d pos_3d;
            Eigen::Matrix3d R;
            alm_traj_ptr_->uneven_map->getTerrainPos(pos, R, pos_3d);
            p.pose.position.x = pos_3d(0);
            p.pose.position.y = pos_3d(1);
            p.pose.position.z = pos_3d(2);
            Eigen::Quaterniond q(R);
            p.pose.orientation.w = q.w();
            p.pose.orientation.x = q.x();
            p.pose.orientation.y = q.y();
            p.pose.orientation.z = q.z();
            back_end_path.poses.push_back(p);
        }
        Eigen::Vector3d pos = traj.getNormSE2Pos(traj.getTotalDuration());
        Eigen::Vector3d pos_3d;
        Eigen::Matrix3d R;
        alm_traj_ptr_->uneven_map->getTerrainPos(pos, R, pos_3d);
        p.pose.position.x = pos_3d(0);
        p.pose.position.y = pos_3d(1);
        p.pose.position.z = pos_3d(2);
        Eigen::Quaterniond q(R);
        p.pose.orientation.w = q.w();
        p.pose.orientation.x = q.x();
        p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z();
        back_end_path.poses.push_back(p);

        se3_pub.publish(back_end_path);
    }

    void AlmTrajOptFlow::visualizeYaw(const SE2Trajectory& traj) {
        visualization_msgs::MarkerArray marker_array;


        geometry_msgs::PoseStamped p;
        int id = 0;
        for (double t=0.0; t < traj.getTotalDuration(); t+=0.2)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "path_yaw";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.1;  // 箭头长度
            marker.scale.y = 0.1;  // 箭头宽度
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            Eigen::Vector2d pos = traj.getPos(t);

            double yaw = traj.getAngle(t);
            p.pose.position.x = pos(0);
            p.pose.position.y = pos(1);
            p.pose.position.z = 0.0;
            p.pose.orientation.w = cos(yaw/2.0);
            p.pose.orientation.x = 0.0;
            p.pose.orientation.y = 0.0;
            p.pose.orientation.z = sin(yaw/2.0);

            marker.pose.position = p.pose.position;
            marker.pose.orientation = p.pose.orientation;

            marker_array.markers.push_back(marker);  // 将marker添加到MarkerArray中

        }
        yaw_pub.publish(marker_array);
    }
}