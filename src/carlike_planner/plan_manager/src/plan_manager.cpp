#include "plan_manager/plan_manager.h"

namespace carlike_planner
{
    void PlanManager::init(ros::NodeHandle& nh)
    {
        nh.getParam("manager/piece_len", piece_len);
        nh.getParam("manager/mean_vel", mean_vel);
        nh.getParam("manager/init_time_times", init_time_times);
        nh.getParam("manager/yaw_piece_times", yaw_piece_times);
        nh.getParam("manager/init_sig_vel", init_sig_vel);
        nh.getParam("manager/no_replan_thresh", no_replan_thresh);
        nh.getParam("manager/replan_thresh", replan_thresh);
        nh.param<string>("manager/bk_dir", bk_dir, "xxx");

        exec_state_  = FSM_EXEC_STATE::INIT;

        uneven_map.reset(new UnevenMap);
        sdf_map.reset(new SDFMap); // local_sdf_map
        kino_astar.reset(new KinoAstar);
        traj_opt_flow.reset(new AlmTrajOptFlow(nh));
        global_map.reset(new GlobalMap());

        sdf_map->initMap(nh);
        kino_astar->init(nh);
        traj_opt_flow->SetEnvironment(sdf_map);


        traj_pub = nh.advertise<mpc_controller::SE2Traj>("traj", 1);
        global_map_sub = nh.subscribe("/global_costmap/costmap/costmap", 1, &PlanManager::rcvGlobalMapCallBack, this);
        path_pub_ = nh.advertise<nav_msgs::Path>("global_path", 1);
        odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &PlanManager::rcvOdomCallBack, this);
        target_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &PlanManager::rcvGoalCallBack, this);

        replan_timer = nh.createTimer(ros::Duration(0.05), &PlanManager::replanCallBack, this);

        have_global_map = false;
        have_odom = false;
        have_goal = false;
    }

    void PlanManager::rcvOdomCallBack(const nav_msgs::OdometryConstPtr& msg)
    {
        odom_pos(0) = msg->pose.pose.position.x;
        odom_pos(1) = msg->pose.pose.position.y;
        odom_pos(2) = tf::getYaw(msg->pose.pose.orientation);
        have_odom = true;
    }

    void PlanManager::rcvGlobalMapCallBack(const nav_msgs::OccupancyGridPtr& msg) {
        global_map->init(msg);
        kino_astar->setEnvironment(global_map);
        have_global_map = true;
        std::cout << "receive global map" << std::endl;
    }
    void PlanManager::rcvGoalCallBack(const geometry_msgs::PoseStamped msg){
        goal = msg;
        have_goal = true;
        if (exec_state_ == WAIT_TARGET)
            changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
        else if (exec_state_ == EXEC_TRAJ)
            changeFSMExecState(REPLAN_TRAJ, "TRIG");
    }

    void PlanManager::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
        string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
        int    pre_s        = int(exec_state_);
        exec_state_         = new_state;
        cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
    }

    void PlanManager::replanCallBack(const ros::TimerEvent & /*event*/){

        switch (exec_state_) {
            case INIT: {
                if (!have_odom) {
                    return;
                }
                if(!sdf_map->md_.has_esdf_){
                    return;
                }
                changeFSMExecState(WAIT_TARGET, "FSM");
                break;
            }

            case WAIT_TARGET: {
                if (!have_goal)
                    return;
                else {
                    changeFSMExecState(GEN_NEW_TRAJ, "FSM");
                }
                break;
            }

            case GEN_NEW_TRAJ: {

                Eigen::Vector3d end_state(goal.pose.position.x, \
                          goal.pose.position.y, \
                          atan2(2.0*goal.pose.orientation.z*goal.pose.orientation.w, \
                                2.0*pow(goal.pose.orientation.w, 2)-1.0)             );
                bool success = plan(odom_pos, end_state);
                if (success) {
                    changeFSMExecState(EXEC_TRAJ, "FSM");
                } else {
                    changeFSMExecState(GEN_NEW_TRAJ, "FSM");
                }
                break;
            }

            case EXEC_TRAJ: {
                /* determine if need to replan */
                ros::Time      time_now = ros::Time::now();
                // info->start_time是上一次规划轨迹的开始时间，可以认为是t=0时刻，因此t_cur就是相对开始时间的当前时刻
                double         t_cur    = (time_now - local_traj.start_time).toSec();

                t_cur                   = min(local_traj.pos_traj.getTotalDuration(), t_cur);
//
                Eigen::Vector2d start_pos = local_traj.pos_traj.getValue(0.0);
                Eigen::Vector2d cur_pos = local_traj.pos_traj.getValue(t_cur);
                Eigen::Vector3d end_state(goal.pose.position.x, \
                          goal.pose.position.y, \
                          atan2(2.0*goal.pose.orientation.z*goal.pose.orientation.w, \
                                2.0*pow(goal.pose.orientation.w, 2)-1.0)             );

                if(in_plan){
                    return;
                }
                if (t_cur > local_traj.pos_traj.getTotalDuration() - 1e-2) {
                    have_goal = false;
                    changeFSMExecState(WAIT_TARGET, "FSM");
                    return;

                } else if ((end_state.head(2) - cur_pos).norm() < no_replan_thresh) {
                    return;

                } else if ((start_pos - cur_pos).norm() < replan_thresh) {
                    return;

                } else {
                    changeFSMExecState(REPLAN_TRAJ, "FSM");
                }
                break;
            }

            case REPLAN_TRAJ: {
                ros::Time      time_now = ros::Time::now();
                double         t_cur    = (time_now - local_traj.start_time).toSec() + 0.4;

                Eigen::Vector2d cur_pt  = local_traj.pos_traj.getValue(t_cur);
                double cur_yaw = local_traj.yaw_traj.getValue(t_cur)[0];

                Eigen::Vector3d cur_pos(cur_pt.x(), cur_pt.y(), cur_yaw);

                Eigen::Vector3d end_pos(goal.pose.position.x, \
                          goal.pose.position.y, \
                          atan2(2.0*goal.pose.orientation.z*goal.pose.orientation.w, \
                                2.0*pow(goal.pose.orientation.w, 2)-1.0)             );

                bool success = plan(cur_pos, end_pos);
                if (success) {
                    changeFSMExecState(EXEC_TRAJ, "FSM");
                } else {
                    changeFSMExecState(GEN_NEW_TRAJ, "FSM");
                }
                break;
            }
        }
    }
    bool PlanManager::plan(const Eigen::Vector3d& start_state, const Eigen::Vector3d& end_state)
    {
        in_plan = true;
        ros::Time start_time = ros::Time::now();

        std::cout << "---------------hybrid A* Start--------------" << std::endl;
        // TODO:没有考虑初始速度和加速度
        std::vector<Eigen::Vector3d> init_path = kino_astar->Plan(start_state, end_state);
        if (init_path.empty())
        {
            in_plan = false;
            return false;
        }
        PublishPath(init_path);
        std::cout << "---------------hybrid A* End--------------" << std::endl;


        std::cout << "-------------------ALM Start------------------" << std::endl;
        // minco optimize

        int ret = traj_opt_flow->Run(init_path);
        std::cout << "--------------------ALM End-------------------" << std::endl;
        if(ret == -1) return false;
        // visualization
        SE2Trajectory back_end_traj = traj_opt_flow->GetTraj();
        // 更新local_traj，用于replan决策,back_end_traj里没有设置start_time，因为start_time应该是在hybrid A*规划前的
        local_traj = back_end_traj;
        local_traj.start_time = start_time;
        // publish to mpc controller
        mpc_controller::SE2Traj traj_msg;
        traj_msg.start_time = ros::Time::now();
        traj_msg.init_v.x = 0.0;
        traj_msg.init_v.y = 0.0;
        traj_msg.init_v.z = 0.0;
        traj_msg.init_a.x = 0.0;
        traj_msg.init_a.y = 0.0;
        traj_msg.init_a.z = 0.0;
        for (int i=0; i<back_end_traj.pos_traj.getPieceNum(); i++)
        {
            geometry_msgs::Point pospt;
            Eigen::Vector2d pos = back_end_traj.pos_traj[i].getValue(0.0);
            pospt.x = pos[0];
            pospt.y = pos[1];
            traj_msg.pos_pts.push_back(pospt);
            traj_msg.posT_pts.push_back(back_end_traj.pos_traj[i].getDuration());
        }
        geometry_msgs::Point pospt;
        Eigen::Vector2d pos = back_end_traj.pos_traj.getValue(back_end_traj.pos_traj.getTotalDuration());
        pospt.x = pos[0];
        pospt.y = pos[1];
        traj_msg.pos_pts.push_back(pospt);

        for(int i = 0; i < back_end_traj.yaw_traj.getPieceNum(); i ++){
            geometry_msgs::Point anglept;
            Eigen::VectorXd angle = back_end_traj.yaw_traj[i].getValue(0.0);
            anglept.x = angle[0];
            traj_msg.angle_pts.push_back(anglept);
            traj_msg.angleT_pts.push_back(back_end_traj.yaw_traj[i].getDuration());
        }
        geometry_msgs::Point anglept;
        Eigen::VectorXd angle = back_end_traj.yaw_traj.getValue(back_end_traj.yaw_traj.getTotalDuration());
        anglept.x = angle[0];
        traj_msg.angle_pts.push_back(anglept);
        traj_pub.publish(traj_msg);

        in_plan = false;
        return true;
    }

    void PlanManager::PublishPath(const std::vector<Eigen::Vector3d> &path) {
        nav_msgs::Path nav_path;
        geometry_msgs::PoseStamped pose_stamped;
        for (const auto &pose: path) {
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = pose.x();
            pose_stamped.pose.position.y = pose.y();
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

            nav_path.poses.emplace_back(pose_stamped);
        }

        nav_path.header.frame_id = "map";
        nav_path.header.stamp = ros::Time::now();

        path_pub_.publish(nav_path);
    }

}