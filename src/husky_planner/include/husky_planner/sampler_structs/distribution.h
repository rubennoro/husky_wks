#pragma once
#include "husky_planner/sampler_structs/grid_map.h"
#include "husky_planner/params.h"
#include <random>

/**
 * p(x,y) = max(D(x,y)) - D(x,y) + epsilon
 * Normal Distribution class carries out the main implementation maintaining the 
 * distribution for sampling ground nodes given the formula above. 
 * 
 * Densities are determined as the number of times sampled for a certain cell, and
 * are smoothed out with an average filter using a kernel size equal to that of 
 * given robot's feet params.
 * The probabilities for each ground cell are then updated using the formula above, and the
 * distribution generates a random sample. 
 */
class NormalDistribution{
public:
    NormalDistribution() = default;

    void add_cell(Cell &c){
        ground_cells.push_back(&c);
        num_cells++;
    }

    Cell& operator[](uint32_t index){
        /*
         * Out of bounds check.
         */
        if(index >= num_cells){
            return *ground_cells[0];
        }

        return *ground_cells[index];
    }

    uint32_t get_num_cells(){
        return num_cells;
    }

    /**
     * Initializes the distribution after putting together the vector of all ground
     * cells.
     */
    void init_dist();

    /**
     * Update the distribution to account for adaptive sampling implementation and ensure
     * non-sampled regions get higher probabilities.
     */
    void update_dist(const std::vector<float> &updated_probs);

    /**
     * Get the (x,y,z) coords of a randomly sampled cell. This is done by 
     * first sampling a cell given the distribution, and given the bounds, sampling a point within
     * the cell at random. 
     */
    void generate_sample(uint32_t &index, float &x, float &y, float &z);

    /**
     * Wrapper around the add node, which increments the sample used.
     */
    void add_sample_meas(uint32_t index);

    /**
     * Updates the densities affected by the new sampled cell. 
     */
    void update_densities(uint32_t index, const Kernel &k);
    
    /**
     * Updates the probabilities overall to create a new distribution.
     */
    void update_probs();

private:
    std::default_random_engine generator;
    std::default_random_engine rng;
    std::discrete_distribution<int> sampler;

    /*
     * The total number of ground cells.
     */
    uint32_t num_cells;

    /*
     * Stored as a pointer, cells shared with the GridMap.
     */
    std::vector<Cell*> ground_cells;

    /*
     * Storing the maximum density, to be updated when checked after each iteration
     */
    float max_density;
};