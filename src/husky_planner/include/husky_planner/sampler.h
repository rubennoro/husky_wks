#pragma once

#include <random>
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

    /*
     * Sample from the normal distribution. 
     * Multivariate gaussian -> need to generate points for (x, y)
     */
    sample_normal_dist(){

    }

private:
    float foot_width;
    float foot_length;
    std::normal_distribution dist;
};
/*
Understanding the other code:
grid_map ? 

ompl: 

grid_map::Position SE3FromSE2Sampler::samplePositionInMapFromDist() {
  // Sample uniform values.
  const auto samp_col = rng_.uniform01();
  const auto samp_row = rng_.uniform01();

  ** The above use ompl::RNG rng_ and sample a double between 0 and 1 **

  Eigen::Index col, row;

  ** 

  const auto& cum_prob = map_->getLayer("cum_prob");

  ** map_ is a Map class type, which is build around a GridMap object **
  ** the above returns grid_map::Matrix, returning the cum_prob layer **
  ** TODO() -> cumulative_prob layer?? **
  ** 

  const Eigen::Matrix<grid_map::DataType, Eigen::Dynamic, 1> cum_prob_rowwise =
      map_->getLayer("cum_prob_rowwise_hack").col(0);

  // Apply distribution
  for (row = 0; row < cum_prob_rowwise.rows()-1; ++row) {
    if (cum_prob_rowwise(row, 0) > samp_row) break;
  }
  for (col = 0; col < cum_prob.cols()-1; ++col) {
    if (cum_prob(row, col) > samp_col) break;
  }

  return map_->getPositionOfIndex(grid_map::Index(row, col));

  // TODO: Sample continuous value inside sampled cell.
}



void SE3FromSE2Sampler::sampleUniform(ob::State* state) {
  auto state_se3 = state->as<ob::SE3StateSpace::StateType>();

  // Update z value with height from map.
  grid_map::Position pos;
  if (params_->sampler.sample_from_distribution) {
    pos = samplePositionInMapFromDist();
  } else {
    pos = samplePositionInMap();
  }

  // Get index of position to slightly speed up next operations.
  const auto ind = map_->getIndexOfPosition(pos);

  state_pos_.get()->values[0] = pos.x();
  state_pos_.get()->values[1] = pos.y();
  state_pos_.get()->values[2] = map_->getHeightAtIndex(ind);

  // Apply small random perturbation in normal direction.
  const Eigen::Vector3d normal_w = map_->getNormal(ind);

  const auto std = map_->getPlaneFitStdDev(ind);

  const auto pert = rng_.uniformReal(-1, 1) * std::min(std, 0.5f) * params_->robot.feet.reach.z;

  state_pos_.get()->values[0] += normal_w.x() * pert;
  state_pos_.get()->values[1] += normal_w.y() * pert;
  state_pos_.get()->values[2] += normal_w.z() * pert;

  state_se3->setX(state_pos_.get()->values[0]);
  state_se3->setY(state_pos_.get()->values[1]);
  state_se3->setZ(state_pos_.get()->values[2]);

  // Sample rotation.
  rng_.eulerRPY(state_pos_.get()->values);  // Abuse position to avoid memory allocation.

  // Get roll and pitch from elevation map.

  const Eigen::Quaterniond R_wb(Eigen::AngleAxisd(state_pos_.get()->values[2],
                                                  Eigen::Vector3d::UnitZ()));

  const auto normal_b = R_wb.inverse() * normal_w;

  state_pos_.get()->values[0] = -atan2(normal_b.y(), normal_b.z())
                                + state_pos_.get()->values[0] * params_->sampler.max_roll_pert / M_PI_2; // Roll.
  state_pos_.get()->values[1] = atan2(normal_b.x(), normal_b.z())
                                + state_pos_.get()->values[1] * params_->sampler.max_pitch_pert / M_PI_4; // Pitch.

  setSO3FromRPY(state_se3->rotation(), state_pos_.get()->values);
}
*/