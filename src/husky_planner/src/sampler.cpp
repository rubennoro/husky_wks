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

                /*
                 * Non 0 height in the first conditional, otherwise z = 0.
                 */
                if(map(i)(j).x() >= x_mins[k] && map(i)(j).x() <= x_maxs[k] && map(i)(j).y() >= y_mins[k] && map(i)(j).y() <= y_maxs[k]){
                    map(i)(j).set_z(z_max[k]);

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
                    dist.add_cell(map(i)(j));
                }
            }
        }
    }

}

void Sampler::sample_point(float &x, float &y, float &z){

    /*
     * Ensure that the point is on the ground. Do not exit the loop until sampling a node that
     * is not on the ground.
     */
    z = -1;
    while(z != 0){
        /*
        * Generate a sample index from 0 to total # of cells.
        */
        int index;
        dist.generate_sample(index);

        /*
        * Use the distribution's index output to index the GridMap.
        */
       Cell cell = map.index_cell(index);

       /*
        * TODO(): How are the cells sized?
        * Get a random point within the cell itself.
        */
       std::uniform_real_distribution<float> offset_dist(0, Params::sampling.res);
       float x = cell.x();
       float y = cell.y();
       float z = cell.z();
    }

}

/*
 * TODO(): Flesh this out so I know what to build step by step. 
 */
void Sampler::sample_proc(float &x, float &y, float &z){
    
    /*
     * 1) Sample a Point
     */
    sample_point(x, y, z);

    /*
     * Adaptive Sampling:
     * - need to update the GridMap to account for the cell that was sampled from.
     * <- can probably do the above in the previous step as 1
     */
    // update_grid();

    /*
     * Update the distribution.
     */
    // update_distribution();




}