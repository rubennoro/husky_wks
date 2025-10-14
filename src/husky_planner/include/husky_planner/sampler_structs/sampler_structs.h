#pragma once

struct GridMap{

};

/*
 * An individual cell in the grid for sampling.
 */
struct Cell{
    uint32_t x;
    uint32_t y;
    uint32_t num_nodes;
    float density;
}

struct Kernel{
    uint32_t d_x;
    uint32_t d_y;
}

