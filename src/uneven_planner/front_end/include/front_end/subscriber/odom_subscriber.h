//
// Created by yzh on 24-10-12.
//
#ifndef SRC_ODOM_SUBSCRIBER_H
#define SRC_ODOM_SUBSCRIBER_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <deque>
#include <mutex>
#include <thread>
#include <string>

class OdomSubscriber {
public:
    OdomSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);

    void ParseData(std::deque<nav_msgs::OdometryConstPtr> &deque_odom_msg_ptr);

private:
    void MessageCallBack(const nav_msgs::OdometryConstPtr &odom_msg_ptr);

private:
    ros::Subscriber subscriber_;
    std::deque<nav_msgs::OdometryConstPtr> deque_odom_;

    std::mutex buff_mutex_;
};


#endif //SRC_ODOM_SUBSCRIBER_H
