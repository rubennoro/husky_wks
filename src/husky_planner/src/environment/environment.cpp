#include "husky_planner/environment/environment.h"

namespace Map {

    std::vector<Body> load_environment_obstacles() {
        std::vector<Body> bodies(Params::env.blocks.x_min.size());
        for(uint32_t i = 0; i < Params::env.blocks.x_min.size(); i++){
            bodies[i].x_min = Params::env.blocks.x_min[i];
            bodies[i].x_max = Params::env.blocks.x_max[i];
            bodies[i].y_min = Params::env.blocks.y_min[i];
            bodies[i].y_max = Params::env.blocks.y_max[i];
            bodies[i].z_min = Params::env.blocks.z_min[i];
            bodies[i].z_max = Params::env.blocks.z_max[i];
        }
        return bodies;
    }

    /*
     * Start Position of the robot.
     */
    Vec get_start_position() {
        return {Params::env.start.x, Params::env.start.y, Params::env.start.z};
    }

    /*
     * Goal position of the robot.
     */
    Vec get_goal_position() {
        return {Params::env.goal.x, Params::env.goal.y, Params::env.goal.z};
    }

}  
