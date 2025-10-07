#pragma once

#include "husky_planner/struct_utils/struct_utils.h"
#include "husky_planner/params.h"
#include <cstdint>
// PRM already exists for ompl -> can use this version

namespace prm_utils{

    float randomZeroToOne();
    
    bool AreSame(float a, float b);
    
    float distance(Node n1, Node n2);

    void find_nearest_node(const Node &new_node, const Graph &graph, std::vector<Node> &nearest_nodes);

    bool obstacles_free(Node node);

    bool free_segment(const Node &from_node, const Node &to_node);

}