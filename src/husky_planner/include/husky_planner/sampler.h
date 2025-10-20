#pragma once

#include "husky_planner/sampler_structs/grid_map.h"
#include "husky_planner/sampler_structs/distribution.h"
#include "husky_planner/params.h"
#include <random>
#include <vector>
/*
Steps for the Sampler:
**THIS APPLIES TO JUST GROUND**
1. Get the size of a husky footprint -> can be added as a param
2. Count the # of points in each 2D kernel w/ size of footprint
3. Average-filter each kernel with neighbors
4. Get the pdf w/ the equation:
- p(x, y) = max(D(x,y)) - D(x,y) + bias
- need the max sample density
- iterate across all sample densities computing this

5. Compute the pdf
- get the total of the p(x,y)'s, and compute the probability
of each p(x,y) / total p(x,y) to get prob's per kernel

6. Sample based on this distribution
-> TODO(); FIGURE OUT HOW TO SAMPLE GIVEN A DISTRIBUTION?s

*/

class Sampler{
public:

    /** 
     * Constructor assuming parameter sizes for the grid map.
     * GridMap receives the x and y bounds for ground sampling.
     * NormalDistribution receives the total number of cells for initial random sampling.
     */
    Sampler(): map(Params::env.limits.x_max, Params::env.limits.y_max){}

    /** 
     * Constructor for custom params for the grid map.
     */
    Sampler(uint32_t x, uint32_t y): map(x, y){}

    /** 
     * Constructor for custom params for the grid map.
     */
    Sampler(uint32_t x, uint32_t y, float foot_w, float foot_l): map(x, y), foot(foot_w, foot_l) {}

    /**
     * Initializes kernel with sizings
     */
    void init_kernel(float foot_width, float foot_length);
    
    /*
     * Initialize the height map. Given the parameters, 
     * fill the map with cells mapping to the proper heights.
     */
    void init_height();

    /**
     * Sample an (x, y) point from the GridMap. This distribution is not multivariate because
     * the (x,y) coordinates of each cell share the same probabilities. 
     */
    void sample_point(int &index, float &x, float &y, float &z);

    /**
     * Update the densities surrounding the sampled point.
     */
    void smooth_densities(int index);

    /**
     * Wrapper for distribution updating of the probabilities.
     */
    void update_probabilities();

    /**
     * Carries out the process of:
     * 1) Sampling
     * 2) Updating the densities
     * 3) Updating the probabilities and the distribution.
     */
    void sample_process(float &x, float &y, float &z);

private:
    Kernel foot;
    NormalDistribution dist;
    GridMap map;
};