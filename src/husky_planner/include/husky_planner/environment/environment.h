#pragma once
#include <vector>
#include "husky_planner/struct_utils/struct_utils.h"
#include "husky_planner/params.h"
#include <cstdint>

namespace Map{

    std::vector<Body> load_environment_obstacles();

    Vec get_start_position();

    Vec get_goal_position();

}