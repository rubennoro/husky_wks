#pragma once

#include "husky_planner/struct_utils/struct_utils.h"
#include "husky_planner/params.h"
#include <cstdint>

namespace prm_utils{

    /**
     * Generates a random point between 0 and 1 for sampling nodes within the environment.
     */
    float randomZeroToOne();
    
    /**
     * Manual float comparison.
     */
    bool AreSame(float a, float b);
    
    /**
     * 3D distance between two nodes on x,y,z.
     */
    float distance(Node n1, Node n2);

    /**
     * TODO(): docs
     */
    void find_nearest_node(const Node &new_node, const Graph &graph, std::vector<Node> &nearest_nodes);

    /**
     * TODO(): docs
     */
    bool obstacles_free(Node node);

    /**
     * TODO(): docs
     */
    bool free_segment(const Node &from_node, const Node &to_node);
}