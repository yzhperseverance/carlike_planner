

#ifndef SRC_ALM_TRAJ_OPT_FLOW_H
#define SRC_ALM_TRAJ_OPT_FLOW_H
#include <ros/ros.h>
#include <Eigen/Core>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include "back_end/minco_traj_opt/alm_traj_opt.h"
#include "utils/se2traj.hpp"
#include <plan_env/edt_environment.h>
namespace carlike_planner {

    class AlmTrajOptFlow {
    public:
        AlmTrajOptFlow() = delete;

        explicit AlmTrajOptFlow(ros::NodeHandle &nh);

        int Run(const std::vector<Eigen::Vector3d> &init_path);

        inline void SetEnvironment(const SDFMap::Ptr& env) { alm_traj_ptr_->setEnvironment(env); }

        inline void SetEnvironment(const UnevenMap::Ptr& env) { alm_traj_ptr_->setEnvironment(env); }

        inline SE2Trajectory GetTraj() { return alm_traj_ptr_->getTraj(); }

    private:

        void SmoothYaw();

        void GetInnerPoint(Eigen::MatrixXd &inner_xy, Eigen::VectorXd &inner_yaw, double &total_time);

        void PublishSE2Traj(const SE2Trajectory& traj);

        void PublishSE3Traj(const SE2Trajectory& traj);

        void visualizeYaw(const SE2Trajectory& traj);

        void GetLocalInitPath(const std::vector<Eigen::Vector3d> &init_path);

    private:
        std::shared_ptr<ALMTrajOpt> alm_traj_ptr_;

        std::vector<Eigen::Vector3d> init_path_;

        ros::Publisher se2_pub;
        ros::Publisher se3_pub;
        ros::Publisher yaw_pub;

        double max_vel_;


    };
}

#endif //SRC_ALM_TRAJ_OPT_FLOW_H
