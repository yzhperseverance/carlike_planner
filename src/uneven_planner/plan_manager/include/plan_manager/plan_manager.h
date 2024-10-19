#pragma once

#include <fstream>
#include <string.h>
#include <random>
#include <time.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include "plan_env/edt_environment.h"
#include "kino_astar.h"
#include "back_end/minco_traj_opt/alm_traj_opt.h"
#include "back_end/minco_traj_opt/alm_traj_opt_flow.h"
#include "mpc_controller/SE2Traj.h"

namespace uneven_planner
{
    class PlanManager
    {
        private:
            bool in_plan = false;
            double piece_len;
            double mean_vel;
            double init_time_times;
            double yaw_piece_times;
            double init_sig_vel;
            bool is_uneven;
            bool has_global_map;
            Eigen::Vector3d odom_pos;
            string bk_dir;

            UnevenMap::Ptr uneven_map;
            SDFMap::Ptr sdf_map;
            GlobalMap::Ptr global_map;
            KinoAstar::Ptr kino_astar;
            std::shared_ptr<AlmTrajOptFlow> traj_opt_flow;
            SE2Trajectory opted_traj;

            ros::Publisher path_pub_;
            ros::Publisher traj_pub;
            ros::Subscriber odom_sub;
            ros::Subscriber target_sub;
            ros::Subscriber global_map_sub;
            
        public:
            void init(ros::NodeHandle& nh);
            void rcvOdomCallBack(const nav_msgs::OdometryConstPtr& msg);
            void rcvWpsCallBack(const geometry_msgs::PoseStamped msg);
            void rcvGlobalMapCallBack(const nav_msgs::OccupancyGridPtr& msg);
            void rcvGlobalMapCallBack(const nav_msgs::OccupancyGridPtr& msg);
            void PublishPath(const std::vector<Eigen::Vector3d> &path);
    };
}
