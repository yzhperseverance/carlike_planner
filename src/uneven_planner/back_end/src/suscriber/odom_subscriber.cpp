//
// Created by yzh on 24-10-12.
//
#include "back_end/subscriber/odom_subscriber.h"

OdomSubscriber::OdomSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) {
    subscriber_ = nh.subscribe(topic_name, buff_size, &OdomSubscriber::MessageCallBack, this);
}

void OdomSubscriber::MessageCallBack(const nav_msgs::OdometryConstPtr &amcl_msg_ptr) {
    buff_mutex_.lock();
    deque_odom_.emplace_back(amcl_msg_ptr);
    buff_mutex_.unlock();
}

void OdomSubscriber::ParseData(std::deque<nav_msgs::OdometryConstPtr> &deque_amcl_msg_ptr) {
    buff_mutex_.lock();
    if (!deque_odom_.empty()) {
        deque_amcl_msg_ptr.insert(deque_amcl_msg_ptr.end(),
                                  deque_odom_.begin(),
                                  deque_odom_.end()
        );

        deque_odom_.clear();
    }
    buff_mutex_.unlock();
}