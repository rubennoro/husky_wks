#pragma once
#include "husky_planner/sampler_structs/grid_map.h"
#include <random>

/**
 * TODO(): Flesh this out, outlign goals more concretely.
 * 
 * We have a group of cells on a 2D ground plane. We need to know to pick a certain 
 * cell using a distribution, which is:
 * 1) Adaptive
 *  - It constantly updates after each sampling, rebuilding the distribution.
 * 2) Once in the cell, need to choose a random (x,y) between the bounds- > this serves as the sample
 * 
 * 
 * 
 * p(x,y) = max(D(x,y)) - D(x,y) + epsilon
 * - epsilon is error
 */
class NormalDistribution{
public:
    NormalDistribution() = default;

    /*
     * Constructor that implements distribution with custom params.
     */
    NormalDistribution(float cell_mean, float cell_std_dev): cell_dist(cell_mean, cell_std_dev), num_cells(0) {}

    /**
     * TODO(): Verify that this works. This makes the assumption that sampling the non-ground points doesn't matter.
     * If it does matter, and we wan't a distribution purely for ground nodes, then we need to ONLY include ground cells.
     * Constructor that takes the known grid map sizing params and produces a cell sampler.
     */
    NormalDistribution(float res, float x_size, float y_size): num_cells(0){
        float num_cells = x_size / res * y_size / res;
        float mean_cells = num_cells / 2;
        /**
         * TODO(): DETERMINE IF THIS STD DEV MAKES SENSE
         */
        float std_dev_cells = num_cells / 4;
        cell_dist = std::normal_distribution<float>(mean_cells, std_dev_cells);
    }

    void add_cell(Cell &c){
        ground_cells.push_back(c);
        num_cells++;
    }

    int get_num_cells(){
        return num_cells;
    }

    /**
     * Get the (x,y,z) coords of a randomly sampled cell. This is done by 
     * first sampling a cell given the distribution, and given the bounds, sampling a point within
     * the cell at random. 
     */
    void generate_sample(int &index);

    /**
     * Update the distribution to account for adaptive sampling implementation and ensure
     * non-sampled regions get higher probabilities.
     */
    void update_dist();

private:
    /*
     * GridMap distribution by cell.
     */
    std::normal_distribution<float> cell_dist;
    int num_cells;
    std::vector<Cell> ground_cells;
};