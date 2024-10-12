

#ifndef SRC_ALM_TRAJ_OPT_FLOW_H
#define SRC_ALM_TRAJ_OPT_FLOW_H
#include <ros/ros.h>
#include "back_end/subscriber/odom_subscriber.h"
#include <geometry_msgs/PoseStamped.h>

class AlmTrajOptFlow{
public:
    AlmTrajOptFlow() = delete;

    explicit AlmTrajOptFlow(ros::NodeHandle &nh);

    void run();

private:
    void rcvWpsCallBack(const geometry_msgs::PoseStamped msg);


private:
    std::shared_ptr<OdomSubscriber>  odom_sub_ptr_;
    ros::Subscriber wps_sub;



    std::deque<nav_msgs::OdometryConstPtr> odom_deque_;

    nav_msgs::OdometryConstPtr current_odom_ptr_;






};


#endif //SRC_ALM_TRAJ_OPT_FLOW_H
