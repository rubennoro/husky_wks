#pragma once

struct GridMap{

};

/*
 * An individual cell in the grid for sampling. Contains the 2D position,
 * the number of currently sampled nodes from it, and the density for distribution sampling. 
 */
struct Cell{
    uint32_t x;
    uint32_t y;
    uint32_t z;
    uint32_t num_nodes;
    float density;
}

/*
 * Size of body to operate over grid for updating sampling pdf. 
 */
struct Kernel{
    uint32_t d_x;
    uint32_t d_y;
}

