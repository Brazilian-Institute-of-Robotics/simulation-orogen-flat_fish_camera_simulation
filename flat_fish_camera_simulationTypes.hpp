#ifndef flat_fish_camera_simulation_TYPES_HPP
#define flat_fish_camera_simulation_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <string>

namespace flat_fish_camera_simulation {

    struct LaserLineParams {
        std::string link_name;
        std::string pose_port_name;
    };
}

#endif
