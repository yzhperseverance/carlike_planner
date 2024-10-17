//
// Created by yzh on 24-10-17.
//
#ifndef SRC_GLOBAL_MAP_H
#define SRC_GLOBAL_MAP_H

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

namespace uneven_planner {

    class GlobalMap {
    public:
        void init(const nav_msgs::OccupancyGridPtr& map);

        void posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id);

        void indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos);

        int toAddress(const Eigen::Vector2i &id);

        int toAddress(int x, int y);

        bool isInMap(const Eigen::Vector2d &pos);

        bool isInMap(const Eigen::Vector2i &idx);

        bool isOccupancy(const Eigen::Vector2d& pos);

        bool isOccupancy(const Eigen::Vector2i& id);

        nav_msgs::OccupancyGridPtr global_map;

        double resolution, resolution_inv;
        unsigned int width, height;
        Eigen::Vector2d min_boundary, max_boundary;
        typedef std::shared_ptr<GlobalMap> Ptr;
    };
}
#endif //SRC_GLOBAL_MAP_H
