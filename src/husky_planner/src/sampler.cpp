#include "husky_planner/sampler.h"
#include <chrono>

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
            const auto x_maxs = Params::env.blocks.x_max;
            const auto y_mins = Params::env.blocks.y_min;
            const auto y_maxs = Params::env.blocks.y_max;
            const auto z_maxs = Params::env.blocks.z_max;

            /*
             * Evaluate if any of these fit the bounds, set the height to a corresponding block.
             */
            for(uint32_t k = 0; k < Params::env.blocks.num_blocks; k++){

                /*
                 * Non 0 height in the first conditional, otherwise z = 0.
                 */
                if(map(i, j).x() >= x_mins[k] && map(i, j).x() <= x_maxs[k] && map(i, j).y() >= y_mins[k] && map(i, j).y() <= y_maxs[k]){
                    map(i, j).set_z(z_maxs[k]);

                    /*
                     * Decrease the number of ground cells from total,
                     * and increase the number of elevation cells from 0.
                     */
                    map.inc_elev_cells();
                    map.dec_ground_cells();
                }
                else{
                    /*
                     * Get all of the ground cells for managing the distribution.
                     */
                    dist.add_cell(map(i, j));
                }
            }
        }
    }

}

void Sampler::sample_point(uint32_t &index, float &x, float &y, float &z){

    /*
     * Ensure that the point is on the ground. Do not exit the loop until sampling a node that
     * is not on the ground.
     */
    z = -1;
    while(z != 0){
        /*
        * Generate a sample index from 0 to total # of cells.
        */
        dist.generate_sample(index, x, y, z);
    }

    /*
     * Update the number of randomly sampled occurrences in a single cell here.
     * This is necessary for adaptive sampling.
     */
    dist.add_sample_meas(index);

}

/*
 * Given the index, see if there are surrounding indices that have similar 
 */
void Sampler::smooth_densities(uint32_t index){
    dist.update_densities(index, foot);
}

void Sampler::update_probabilities(){
    dist.update_probs();
}

void Sampler::sample_process(float &x, float &y, float &z){
    
    /*
     * 1) Sample a Point
     */
    uint32_t index;
    sample_point(index, x, y, z);

    /*
     * 2) Adaptive Sampling. Update the densities of cells affected by the new sample location. 
     * Update the maximum if necessary.
     */
    smooth_densities(index);

    /*
     * 3) Update the distribution. Loop over all the new densities and using the formula:
     * p(x,y) = max(D(x,y)) - D(x,y) + e
     * produce the new probabilities. Smooth them out in relation to each other.
     */
    update_probabilities();

}