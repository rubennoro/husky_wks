#pragma once
#include <vector>
#include "husky_planner/struct_utils/struct_utils.h"
#include "husky_planner/params.h"
#include <cstdint>

namespace Map{

    /**
     * Returns the environment obstacles in a vector for publishing node in rviz2.
     */
    std::vector<Body> load_environment_obstacles();

    /**
     * Returns the start node pose from the parameters.
     */
    Vec get_start_position();

    /**
     * Returns the goal node pose from the parameters.
     */
    Vec get_goal_position();

}