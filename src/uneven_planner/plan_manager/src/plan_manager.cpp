#include "plan_manager/plan_manager.h"

namespace uneven_planner
{
    void PlanManager::init(ros::NodeHandle& nh)
    {
        nh.getParam("manager/piece_len", piece_len);
        nh.getParam("manager/mean_vel", mean_vel);
        nh.getParam("manager/init_time_times", init_time_times);
        nh.getParam("manager/yaw_piece_times", yaw_piece_times);
        nh.getParam("manager/init_sig_vel", init_sig_vel);
        nh.getParam("manager/is_uneven", is_uneven);
        nh.getParam("manager/has_global_map", has_global_map);
        nh.param<string>("manager/bk_dir", bk_dir, "xxx");

        uneven_map.reset(new UnevenMap);
        sdf_map.reset(new SDFMap); // local_sdf_map
        kino_astar.reset(new KinoAstar);
        traj_opt_flow.reset(new AlmTrajOptFlow(nh));

        if(is_uneven){
            uneven_map->init(nh);
            kino_astar->init(nh);
            kino_astar->setEnvironment(uneven_map);
            traj_opt_flow->SetEnvironment(uneven_map);
        }
        else{
            sdf_map->initMap(nh);
            kino_astar->init(nh);
            kino_astar->setEnvironment(sdf_map);
            traj_opt_flow->SetEnvironment(sdf_map);
        }

        traj_pub = nh.advertise<mpc_controller::SE2Traj>("traj", 1);
        global_map_sub = nh.subscribe("/global_costmap/costmap/costmap", 1, &PlanManager::rcvGlobalMapCallBack, this);
        odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &PlanManager::rcvOdomCallBack, this);
        target_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &PlanManager::rcvWpsCallBack, this);

    }

    void PlanManager::rcvOdomCallBack(const nav_msgs::OdometryConstPtr& msg)
    {
        odom_pos(0) = msg->pose.pose.position.x;
        odom_pos(1) = msg->pose.pose.position.y;
        Eigen::Quaterniond q(msg->pose.pose.orientation.w, \
                             msg->pose.pose.orientation.x, \
                             msg->pose.pose.orientation.y, \
                             msg->pose.pose.orientation.z  );
        Eigen::Matrix3d R(q);
        odom_pos(2) = UnevenMap::calYawFromR(R);
    }
    void PlanManager::rcvGlobalMapCallBack(const nav_msgs::OccupancyGridPtr& msg) {
        global_map->init(msg);

    }
    void PlanManager::rcvWpsCallBack(const geometry_msgs::PoseStamped msg)
    {
        if (in_plan || !uneven_map->mapReady())
            return;

        in_plan = true;
        Eigen::Vector3d end_state(msg.pose.position.x, \
                                  msg.pose.position.y, \
                                  atan2(2.0*msg.pose.orientation.z*msg.pose.orientation.w, \
                                        2.0*pow(msg.pose.orientation.w, 2)-1.0)             );
        
        std::vector<Eigen::Vector3d> init_path = kino_astar->Plan(odom_pos, end_state);
        if (init_path.empty())
        {
            in_plan = false;
            return;
        }
        // minco optimize
        traj_opt_flow->Run(init_path);
        // visualization
        SE2Trajectory back_end_traj = traj_opt_flow->GetTraj();

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

    }
}