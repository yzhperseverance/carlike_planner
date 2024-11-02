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

namespace carlike_planner
{
    class PlanManager
    {
        private:
            enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };

            bool in_plan = false;
            double piece_len;
            double mean_vel;
            double init_time_times;
            double yaw_piece_times;
            double init_sig_vel;
            double no_replan_thresh;
            double replan_thresh;
            bool have_global_map, have_odom, have_goal;
            Eigen::Vector3d odom_pos;
            string bk_dir;

            UnevenMap::Ptr uneven_map;
            SDFMap::Ptr sdf_map;
            GlobalMap::Ptr global_map;
            KinoAstar::Ptr kino_astar;
            std::shared_ptr<AlmTrajOptFlow> traj_opt_flow;
            SE2Trajectory local_traj;

            ros::Publisher path_pub_;
            ros::Publisher traj_pub;
            ros::Subscriber odom_sub;
            ros::Subscriber target_sub;
            ros::Subscriber global_map_sub;
            ros::Timer replan_timer;

            geometry_msgs::PoseStamped goal;

            FSM_EXEC_STATE exec_state_;

        public:
            void init(ros::NodeHandle& nh);
            void rcvOdomCallBack(const nav_msgs::OdometryConstPtr& msg);
            void replanCallBack(const ros::TimerEvent & /*event*/);
            void rcvGoalCallBack(const geometry_msgs::PoseStamped msg);
            void rcvGlobalMapCallBack(const nav_msgs::OccupancyGridPtr& msg);
            void PublishPath(const std::vector<Eigen::Vector3d> &path);
            bool plan(const Eigen::Vector3d& start_state, const Eigen::Vector3d& end_state);
            void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);

    };
}
