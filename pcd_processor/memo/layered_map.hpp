#pragma once

#include <stdexcept>
#include <grid_map_core/grid_map_core.hpp>
#include <Eigen/Core>

#include "utils/data_structure.hpp"

namespace offroad_planner {
namespace utils
{

class LayeredMap {
    public:
        LayeredMap() = delete;
        explicit LayeredMap(const grid_map::GridMap &grid_map);
        
        bool is_inside(const Eigen::Vector2d &pos) const;
        bool is_inside(const PathPoint &pos) const;

        double get_obstacle_distance(const Eigen::Vector2d &pos) const;
        double get_obstacle_distance(const PathPoint &pos) const;
        
        double get_elevation(const Eigen::Vector2d &pos) const;
        double get_elevation(const PathPoint &pos) const;
        Eigen::Vector3d get_vehicle_rpy(const Eigen::Vector2d &position,
                                        double heading) const;
        Eigen::Quaterniond get_vehicle_quaternion(
            const Eigen::Vector2d &position, double heading) const;

    private:
        const grid_map::GridMap map;
};

} // namespace utils
} // namespace offroad_planner