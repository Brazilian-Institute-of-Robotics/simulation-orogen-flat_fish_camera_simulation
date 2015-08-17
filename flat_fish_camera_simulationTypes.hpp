#ifndef flat_fish_camera_simulation_TYPES_HPP
#define flat_fish_camera_simulation_TYPES_HPP

#include <string>
#include <base/Eigen.hpp>

namespace flat_fish_camera_simulation {

    struct LaserLineParams {

        LaserLineParams() :
            line_color(0.0, 1.0, 0.0),
            line_width(1.0)
        {
        }
        std::string link_name;
        std::string pose_port_name;

        base::Vector3d line_color;
        double line_width;
    };
}

#endif
