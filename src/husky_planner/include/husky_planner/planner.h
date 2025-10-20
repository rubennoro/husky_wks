#pragma once

#include <vector>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include "husky_planner/struct_utils/struct_utils.h"
#include "husky_planner/prm_utils/prm_utils.h"
#include "husky_planner/a_star_utils/a_star_utils.h"
#include "husky_planner/params.h"

class Planner{
public:

    /**
     * Executes the planner steps sequentially.
     * Begins by setting the start and goal nodes.
     * Next it runs the prm.
     * Lastly, it runs the a_star.
     */
    void main_plan();

    /**
     * Sets the start and goal nodes given the parameters.
     */
    void set_points();

    /**
     * Loop to sample 600 random nodes, either ground/platform/flying.
     * They are checked for obstacles, and if available, added to the prm_graph
     * with their node_value. 
     * The start_node and goal_node are added after sampling iterations are completed.
     */
    void run_prm();

    /**
     * Finds the waypoints that designate a path of lowest cost.
     */
    void run_a_star();

    const Node& get_start() const{
        return start;
    }

    const Node& get_goal() const{
        return goal;
    }

    float get_cost() const{
        return total_cost;
    }

    const Graph& get_graph() const{
        return prm_graph;
    }

    /**
     * Get the waypoints object for the ROS2 Publisher.
     */
    const std::vector<Node>& get_waypoints() const { return waypoints; }
private:
    Graph prm_graph; 
    Node start;
    Node goal;
    std::vector<Node> waypoints;
    
    float total_cost;
};