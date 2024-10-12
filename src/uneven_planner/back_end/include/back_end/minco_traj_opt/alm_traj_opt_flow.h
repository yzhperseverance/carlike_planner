

#ifndef SRC_ALM_TRAJ_OPT_FLOW_H
#define SRC_ALM_TRAJ_OPT_FLOW_H
#include <ros/ros.h>
#include <Eigen/Core>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include "back_end/subscriber/path_subscriber.h"
#include "back_end/minco_traj_opt/alm_traj_opt.h"
#include "utils/se2traj.hpp"
namespace uneven_planner {

    class AlmTrajOptFlow {
    public:
        AlmTrajOptFlow() = delete;

        explicit AlmTrajOptFlow(ros::NodeHandle &nh);

        void Run();

    private:

        bool HasPath();

        void InitData();

        void ReadData();

        void ConvertPathToInitPath();

        void SmoothYaw();

        void GetInnerPoint(Eigen::MatrixXd &inner_xy, Eigen::VectorXd &inner_yaw, double &total_time);

        void PublishSE2Traj(const SE2Trajectory& traj);

        void PublishSE3Traj(const SE2Trajectory& traj);

    private:
        std::shared_ptr<ALMTrajOpt> alm_traj_ptr_;
        std::shared_ptr<PathSubscriber> path_sub_ptr_;
        std::deque<nav_msgs::PathPtr> path_deque_;

        nav_msgs::PathPtr current_path_ptr_;
        std::vector<Eigen::Vector3d> init_path_;

        ros::Publisher se2_pub;
        ros::Publisher se3_pub;

        double max_vel_;


    };
}

#endif //SRC_ALM_TRAJ_OPT_FLOW_H
