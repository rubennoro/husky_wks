#pragma once

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
class UniformDistribution{
public:
    /*
     * Constructor that implements distributions with the default params.
     */
    UniformDistribution(){
        const auto lims = Params::env.limits;
        float x_var = (lims.x_max - lims.x_min) / 2;
        float y_var = (lims.y_max - lims.y_min) / 2;
        x_dist(x_var, x_var);
        y_dist(y_var, y_var);
    }

    /*
     * Constructor that implements distributions with custom params.
     */
    UniformDistribution(float x_mean, float x_std_dev, float y_mean, float y_std_dev){
        x_dist(x_mean, x_std_dev);
        y_dist(y_mean, y_std_dev);
    }
private:
    /*
     * x and y distributions, which are independent.
     */
    std::normal_distribution<float> x_dist;
    std::normal_distribution<float> y_dist;
};