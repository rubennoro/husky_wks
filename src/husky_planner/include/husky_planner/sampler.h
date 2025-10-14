#pragma once

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
    Sampler(){
        // Get some params for this
        foot_width = ;
        foot_length = ;
    }

    create_ground_distribution(){

        // 1. Counts the # of points in each 2D kernel where kernel = footprint area

        // 2. Average-filter each kernel with neighbors

    }

private:
    float foot_width;
    float foot_length;
};