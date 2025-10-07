#pragma once

#include "husky_planner/struct_utils/struct_utils.h"
#include <cstdint>

namespace a_star_utils{

    bool AreSame(float a, float b);

    void cost_function(float x_1, float y_1, float z_1, float x_2, float y_2, float z_2, const Graph &prm_graph, float &cost, uint32_t &mode);

    float distance(Node n1, Node n2);

    float distance_2d(Node n1, Node n2);

    void expand_array(float x, float y, float z, float hn, const std::vector<Node> &closed, const Graph &prm_graph, std::vector<AStarNode> &exp_array);

    float heuristic_function(Node n1, Node n2, const Graph &prm_graph);

    int min_fn(const std::vector<AStarNode> &open, float goal_x, float goal_y, float goal_z);
}