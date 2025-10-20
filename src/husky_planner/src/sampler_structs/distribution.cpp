#include "husky_planner/sampler_structs/distribution.h"
#include <random>

void NormalDistribution::init_dist(){
    
    std::vector<float> probs;
    for(uint32_t i = 0; i < num_cells; i++){
        probs.push_back(1);
    }
    sampler = std::discrete_distribution<int>(probs.begin(), probs.end());
}

/**
 * This takes place after the re-smoothing of the densities and calculations of p(x,y)
 * For each probability, it has to be smoothed given the total probability
 */
void NormalDistribution::update_dist(const std::vector<float> &updated_probs){
    sampler = std::discrete_distribution<int>(updated_probs.begin(), updated_probs.end());
}

void NormalDisribution::generate_sample(int &index, float &x, float &y, float &z){

    std::default_random_engine generator;
    std::default_random_engine rng;

    /*
     * TODO(): THIS FUNCTION WORKS, BUT NEED TO UPDATE THE SAMPLER
     */
    index = sampler(generator);
    Cell &cell = ground_cells[index];

    /*
    * TODO(): How are the cells sized?
    * Get a random point within the cell itself.
    */
    std::uniform_real_distribution<float> offset_dist(0, Params::sampling.res);
    float x = cell.x() + offset_dist(rng);
    float y = cell.y() + offset_dist(rng);
    float z = cell.z();
}

/**/
void NormalDistribution::add_sample_meas(int index){
    ground_cells[index].add_node();
}

/*
 * Only need to update 1 value in this case. 
 */
void NormalDistribution::update_densities(int index, const Kernel &k){
    float res = Params::sampling.res;
    
    Cell &cell = ground_cells[index];
    float x = cell.x();
    float y = cell.y();

    /*
     * Updated densities includes the sampled cell.
     */
    for(uint32_t i = 0; i < num_cells; i++){
        Cell &c = ground_cells[i];
        float c_x = c.x();
        float c_y = c.y();

        /*
         * MAINTODO(): 
         * Update the density of the node, check for new max density.
         */
        if(std::abs(x - c_x) <= k.d_x && std::abs(y - c_y) <= k.d_y){
            /* 
             * The total number of cells in a kernel. Used to smooth the updated density.
             */
            float size = (k.d_x / Params::sampling.res) * (k.d_y / Params::sampling.res);

            float updated_density = c.density() + (1 / size);
            c.update_density(updated_density);

            /*
             * Max density update.
             */
            max_density = std::max(updated_density, max_density);
        }

    }
}

void NormalDistribution::update_probs(){

    /*
     * Epsilon Error
     */
    float eps = 1e-6f;

    std::vector<float> updated_probs;
    /*
     * Iterate through all the ground cells, updating the probabilities.
     * 
     */
    float sum_prob = 0;
    for(const auto& cell_ptr : ground){
        float p = max_density - cell_ptr->density() + eps;
        updated_probs.push_back(p);
        sum_prob += p;
    }

    for (auto &p: updated_probs){
        p /= sum_prob;
    }

    /*
     * Update the distribution.
     */
    updated_dist(updated_probs);
}