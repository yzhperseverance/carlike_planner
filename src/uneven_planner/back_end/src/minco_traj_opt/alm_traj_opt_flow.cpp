//
// Created by yzh on 24-10-12.
//
#include "back_end/minco_traj_opt/alm_traj_opt_flow.h"


AlmTrajOptFlow::AlmTrajOptFlow(ros::NodeHandle &nh) {
    odom_sub_ptr_ = std::make_shared<OdomSubscriber>(nh, "odom", 5);
    wps_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &AlmTrajOptFlow::rcvWpsCallBack, this);

    const double rho_T = nh.param("alm_traj_opt/rho_T", rho_T);
    const double rho_ter = nh.param("alm_traj_opt/rho_ter", rho_ter);
    const double max_vel = nh.param("alm_traj_opt/max_vel", max_vel);
    const double max_acc_lon = nh.param("alm_traj_opt/max_acc_lon", max_acc_lon);
    const double max_acc_lat = nh.param("alm_traj_opt/max_acc_lat", max_acc_lat);
    const double max_kap = nh.param("alm_traj_opt/max_kap", max_kap);
    const double min_cxi = nh.param("alm_traj_opt/min_cxi", min_cxi);
    const double max_sig = nh.param("alm_traj_opt/max_sig", max_sig);
    const bool use_scaling = nh.param("alm_traj_opt/use_scaling", use_scaling);
    const double rho = nh.param("alm_traj_opt/rho", rho);
    const double beta = nh.param("alm_traj_opt/beta", beta);
    const double gamma = nh.param("alm_traj_opt/gamma", gamma);
    const double epsilon_con = nh.param("alm_traj_opt/epsilon_con", epsilon_con);
    const int max_iter = nh.param("alm_traj_opt/max_iter", max_iter);
    const double g_epsilon = nh.param("alm_traj_opt/g_epsilon", g_epsilon);
    const double min_step = nh.param("alm_traj_opt/min_step", min_step);
    const int inner_max_iter = nh.param("alm_traj_opt/inner_max_iter", inner_max_iter);
    const double delta = nh.param("alm_traj_opt/delta", delta);
    const int mem_size = nh.param("alm_traj_opt/mem_size", mem_size);
    const int past = nh.param("alm_traj_opt/past", past);
    const double int_K = nh.param("alm_traj_opt/int_K", int_K);
    const bool in_test = nh.param("alm_traj_opt/in_test", in_test);
    const bool in_debug = nh.param("alm_traj_opt/in_debug", in_debug);

}


void AlmTrajOptFlow::run() {

}