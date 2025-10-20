#include "husky_planner/sampler.h"

/*
 * Initializes the kernel size for average filtering. 
 */
void Sampler::init_kernel(float foot_width, float foot_length){
    foot.d_x = foot_width;
    foot.d_y = foot_length;
}

/*
 * Relies on the parameters set to provide each cell in the grid map with a height.
 */
void Sampler::init_height(){

    /*
     * Initialize the x,y bounds for each cell to specify their size.  
     */
    map.init_coords();

    /*
     * TODO(): VERIFY THIS WORKS
     */
    for(uint32_t i = 0; i < map.rows(); i++){
        for(uint32_t j = 0; j < map.cols(); j++){

            const auto x_mins = Params::env.blocks.x_min;
            const auto x_maxs = Params::env.blocks.x_min;
            const auto y_mins = Params::env.blocks.y_min;
            const auto y_maxs = Params::env.blocks.y_max;

            /*
             * Evaluate if any of these fit the bounds, set the height to a corresponding block.
             */
            for(uint32_t k = 0; k < Params::env.blocks.num_blocks; k++){

                if(map(i)(j).x() >= x_mins[k] && map(i)(j).x() <= x_maxs[k] && map(i)(j).y() >= y_mins[k] && map(i)(j).y() <= y_maxs[k]){
                    map(i)(j).set_z(z_max[k]);
                }
            }
        }
    }


}