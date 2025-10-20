#include "husky_planner/sampler_structs/distribution.h"
#include <random>

void NormalDisribution::generate_sample(int &index){

    std::default_random_engine generator;

    index = static_cast<int>(cell_dist(generator));

}