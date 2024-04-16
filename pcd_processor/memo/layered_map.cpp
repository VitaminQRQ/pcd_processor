#include "utils/layered_map.hpp"
#include "params/gflags.hpp"

namespace offroad_planner {
namespace utils {

LayeredMap::LayeredMap(const grid_map::GridMap &grid_map)
    : map(grid_map){    
    // BUG: 这里换成 FLAGS_distance_layer 之后就报错了，我寻思是 string 中带下划线的就会报错，很怪
    // if (!grid_map.exists(FLAGS_distance_layer)) {
    //     throw std::runtime_error("grid map must contain 'distance' layer");
    //     std::exit(1);
    // }

    // BUG: 这里换成 FLAGS_elevation_layer 之后就报错了，很怪
    // if (!grid_map.exists(FLAGS_elevation_layer)) {
    //     throw std::runtime_error("grid map must contain 'elevation' layer");
    //     std::exit(1);
    // }
}

bool LayeredMap::is_inside(const Eigen::Vector2d &pos) const {
    return this->map.isInside(pos);
} 

bool LayeredMap::is_inside(const PathPoint &pos) const {
    return this->map.isInside(Eigen::Vector2d(pos.x, pos.y));
}

double LayeredMap::get_obstacle_distance(const Eigen::Vector2d &pos) const {
    if (this->is_inside(pos)) {
        return this->map.atPosition(
            FLAGS_distance_layer, 
            pos, 
            grid_map::InterpolationMethods::INTER_LINEAR
        );
    } else {
        return 0.0;
    }
}

double LayeredMap::get_obstacle_distance(const PathPoint &pos) const {
    if (this->is_inside(pos)) {
        return this->map.atPosition(
            FLAGS_distance_layer, 
            Eigen::Vector2d(pos.x, pos.y), 
            grid_map::InterpolationMethods::INTER_LINEAR
        );
    } else {
        return 0.0;
    }
}

double LayeredMap::get_elevation(const Eigen::Vector2d &pos) const {
    if (this->is_inside(pos)) {
        return this->map.atPosition(
            FLAGS_elevation_layer, 
            pos, 
            grid_map::InterpolationMethods::INTER_LINEAR
        );
    } else {
        return 0.0;
    }
}

double LayeredMap::get_elevation(const PathPoint &pos) const {
    if (this->is_inside(pos)) {
        return this->map.atPosition(
            FLAGS_elevation_layer, 
            Eigen::Vector2d(pos.x, pos.y),
            grid_map::InterpolationMethods::INTER_LINEAR
        );
    } else {
        return 0.0;
    }
}

Eigen::Vector3d LayeredMap::get_vehicle_rpy(const Eigen::Vector2d &position,
                                            double heading) const {
    if (!this->is_inside(position)) {
        throw std::runtime_error("position not in map");
        std::exit(1);
    } else {
        const Eigen::Vector3d normal_vector = {
            this->map.atPosition(FLAGS_normal_vectors_x, position),
            this->map.atPosition(FLAGS_normal_vectors_y, position),
            this->map.atPosition(FLAGS_normal_vectors_z, position)};

        const Eigen::AngleAxisd rotation_vector(heading, normal_vector);
        return rotation_vector.matrix().eulerAngles(0, 1, 2);
    }
}

Eigen::Quaterniond LayeredMap::get_vehicle_quaternion(
    const Eigen::Vector2d &position, double heading) const {
    if (!this->is_inside(position)) {
        throw std::runtime_error("position not in map");
        std::exit(1);
    } else {
        const Eigen::Vector3d normal_vector = {
            this->map.atPosition(FLAGS_normal_vectors_x, position),
            this->map.atPosition(FLAGS_normal_vectors_y, position),
            this->map.atPosition(FLAGS_normal_vectors_z, position)};

        const Eigen::AngleAxisd rotation_vector(heading, normal_vector);
        return Eigen::Quaterniond(rotation_vector);
    }
}

} // namespace utils
} // namespace offroad_planner