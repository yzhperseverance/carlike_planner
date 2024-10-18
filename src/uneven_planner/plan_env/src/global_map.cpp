//
// Created by yzh on 24-10-17.
//
#include "plan_env/global_map.h"
namespace uneven_planner{

    void GlobalMap::init(const nav_msgs::OccupancyGridPtr &map) {
        global_map = map;
        resolution = map->info.resolution;
        resolution_inv = 1 / map->info.resolution;
        width = map->info.width;
        height = map->info.height;
        min_boundary(0) = map->info.origin.position.x;
        min_boundary(1) = map->info.origin.position.y;
        max_boundary(0) = map->info.origin.position.x + width * resolution;
        max_boundary(1) = map->info.origin.position.y + height * resolution;
    }

    void GlobalMap::posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id) {
        id(0) = floor((pos(0) - global_map->info.origin.position.x) * resolution_inv);
        id(1) = floor((pos(1) - global_map->info.origin.position.y) * resolution_inv);
    }

    void GlobalMap::indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos) {
        pos(0) = (id(0) + 0.5) * resolution + global_map->info.origin.position.x;
        pos(1) = (id(1) + 0.5) * resolution + global_map->info.origin.position.y;
    }

    int GlobalMap::toAddress(const Eigen::Vector2i &id) {
        return id(1) * width + id(0);
    }

    int GlobalMap::toAddress(int x, int y) {
        return y * width + x;
    }

    bool GlobalMap::isInMap(const Eigen::Vector2d &pos) {
        if (pos(0) < min_boundary(0) + 1e-4 || pos(1) < min_boundary(1) + 1e-4) {
            return false;
        }
        if (pos(0) > max_boundary(0) - 1e-4 || pos(1) > max_boundary(1) - 1e-4) {
            return false;
        }
        return true;
    }


    bool GlobalMap::isOccupancy(const Eigen::Vector2d& pos)
    {
        if (!isInMap(pos)){
            std::cout << "Error occur in GlobalMap::isOccupancy.Pos out of range!" << std::endl;
            return false;
        }
        Eigen::Vector2i id;

        posToIndex(pos, id);

        // 膨胀的时候有些值不是100，向上取整数
        if(global_map->data[toAddress(id)]){
            return true;
        }
        else{
            return false;
        }

    }


}