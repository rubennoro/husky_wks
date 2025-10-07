#include "husky_planner/planner.h"
#include <ctime>
#include <cstdlib>
#include <cstdio>
/*
class Planner:
Objects:
- Map Object -> special DS
- Robot Constraints -> simple struct probably works
- Graph of sampled nodes, output from the PRM -> special DS
- Final Path -> special DS?
Methods:
- constructor/initializer
- get_graph() -> runs prm
- a_star() -> runs a* on the graph, gets a final path


Actions:
1. Initialize a map object -> the 3d space that the we already have full context for
2. Initialize the constraints for the robot, flight/transition/walking costs
3. Run PRM:
- Operate on the map, use the robot constraints
- Output should be a graph of nodes 
4. 
*/
void Planner::main_plan(){
    
    // Test print
    // printf("here_init");
    // fflush(stdout);
    set_points();
    // Input should be a map, output is a graph
    run_prm();

    // Test print
    // printf("here3");
    // fflush(stdout);

    // Input should be a graph, output is a waypoints vectors and cost
    run_a_star();

    printf("complete");
    fflush(stdout);
}

void Planner::set_points(){

    start.set_coords(Params::env.start.x, Params::env.start.y, Params::env.start.z);    
    goal.set_coords(Params::env.goal.x, Params::env.goal.y, Params::env.goal.z);  

}

void Planner::run_prm(){

    // Test print
    // printf("run_prm started\n");
    // fflush(stdout);

    /*
     * PRM number_nodes, number_ground_nodes, and radius are in params.h
     */

    // New vector storing nearby nodes 
    std::vector<Node> nearest_nodes;
    // uint32_t edge_counter = 1;
    
    // Initialize node to the start node
    prm_graph.add_node(start); // need to add a 1 to the end of this, to signify number of edges

    Node new_node;
    uint32_t i = 0;
    while(i <= Params::prm.number_nodes){

        // Test print
        // printf("prm_cyc_a, i = %u\n", i);
        // fflush(stdout);

        /**
         * Sample from randomly generated nodes within the bounds.
         * First conditional: ground nodes  
         */
        if(i <= std::floor(0.1 * Params::prm.number_ground_nodes) && Params::env.platform.active_platform){
            float temp_x = prm_utils::randomZeroToOne() * (Params::env.platform.x_max - Params::env.platform.x_min) + Params::env.platform.x_min;
            float temp_y = prm_utils::randomZeroToOne() * (Params::env.platform.y_max - Params::env.platform.y_min) + Params::env.platform.y_min;
            float temp_z = Params::env.platform.z_max;
            new_node.set_coords(temp_x, temp_y, temp_z);
        }
        // Next 90% of Ground Nodes
        else if(i <= Params::prm.number_ground_nodes){
            float temp_x = prm_utils::randomZeroToOne() * Params::env.limits.x_max;
            float temp_y = prm_utils::randomZeroToOne() * Params::env.limits.y_max;
            float temp_z = 0;
            new_node.set_coords(temp_x, temp_y, temp_z);
        }
        // All other nodes, in space
        else{
            float temp_x = prm_utils::randomZeroToOne() * Params::env.limits.x_max;
            float temp_y = prm_utils::randomZeroToOne() * Params::env.limits.y_max;
            float temp_z = prm_utils::randomZeroToOne() * Params::env.limits.z_max;
            new_node.set_coords(temp_x, temp_y, temp_z);
        }

        if(prm_utils::obstacles_free(new_node)){

            /*
             Finding the nearest sampled nodes based on distance from the new node.
             */
            std::vector<Node> nearest_nodes;
            prm_utils::find_nearest_node(new_node, prm_graph, nearest_nodes);
            std::sort(nearest_nodes.begin(), nearest_nodes.end());

            int node_value = 0;

            // Values used to determine Driving or Flying node to add to graph
            float check_x, check_y, check_z;
            new_node.get_coords(check_x, check_y, check_z);

            /*
             * TODO() -> DONE: Need to get a proper Map input with these attributes
             */
            float landing_clearance = Params::env.clearance.landing_area;
            if((check_z == 0) || (check_x >= Params::env.platform.x_min + landing_clearance && check_x <= Params::env.platform.x_max - landing_clearance && check_y >= Params::env.platform.y_min + landing_clearance && check_y <= Params::env.platform.y_max - landing_clearance && check_z == Params::env.platform.z_max && Params::env.platform.active_platform == 1)){
                node_value = 1;
            }
            else{
                node_value = 2;
            }

            // Let this node val be 
            new_node.set_value(node_value);
            prm_graph.add_node(new_node);

            for(uint32_t j = 0; j < nearest_nodes.size(); j++){
                if(prm_utils::free_segment(new_node, nearest_nodes[j])){
                    // The from edge = index 0 -> stores locomotion mode / node_value
                    // The to edge = index 1 -> stores distance
                    prm_graph.add_edge(new_node, nearest_nodes[j]);
                }
            }
            i++;
        }
    }

    // Test print
    // printf("here1");
    // fflush(stdout);

    // Let start be the start node from the Map struct
    int node_value = 1;
    Node start_node{start.get_x(), start.get_y(), start.get_z()};
    start_node.set_value(node_value);

    /*
        * Redo the collection of the nearest_nodes, this time with the start
        */
    nearest_nodes.clear();
    prm_utils::find_nearest_node(start_node, prm_graph, nearest_nodes);
    std::sort(nearest_nodes.begin(), nearest_nodes.end());

    prm_graph.add_node(start_node);

    for(uint32_t j = 0; j < nearest_nodes.size(); j++){
        if(prm_utils::free_segment(start_node, nearest_nodes[j])){
            prm_graph.add_edge(start_node, nearest_nodes[j]);
        }
    }

    Node goal_node{goal.get_x(), goal.get_y(), goal.get_z()};
    goal_node.set_value(node_value);

    nearest_nodes.clear();
    prm_utils::find_nearest_node(goal_node, prm_graph, nearest_nodes);
    std::sort(nearest_nodes.begin(), nearest_nodes.end());

    prm_graph.add_node(goal_node);

    for(uint32_t j = 0; j < nearest_nodes.size(); j++){
        if(prm_utils::free_segment(goal_node, nearest_nodes[j])){
            prm_graph.add_edge(goal_node, nearest_nodes[j]);
        }
    }

    // Test print
    // printf("here2");
    // fflush(stdout);

    // ---- DEBUG PRM GRAPH ----
    // printf("\n----- PRM Summary -----\n");
    // printf("Total Nodes: %lu\n", prm_graph.get_nodes().size());
    // printf("Total Edges: %lu\n", prm_graph.get_edges().size());

    // float sx = start.get_x(), sy = start.get_y(), sz = start.get_z();
    // float gx = goal.get_x(), gy = goal.get_y(), gz = goal.get_z();

    // int start_connections = 0;
    // int goal_connections = 0;

    // for (const auto& edge : prm_graph.get_edges()) {
    //     float fx, fy, fz;
    //     edge.from.get_coords(fx, fy, fz);

    //     float tx, ty, tz;
    //     edge.to.get_coords(tx, ty, tz);

    //     if ((std::fabs(fx - sx) < 1e-6 && std::fabs(fy - sy) < 1e-6 && std::fabs(fz - sz) < 1e-6) ||
    //         (std::fabs(tx - sx) < 1e-6 && std::fabs(ty - sy) < 1e-6 && std::fabs(tz - sz) < 1e-6)) {
    //         start_connections++;
    //     }

    //     if ((std::fabs(fx - gx) < 1e-6 && std::fabs(fy - gy) < 1e-6 && std::fabs(fz - gz) < 1e-6) ||
    //         (std::fabs(tx - gx) < 1e-6 && std::fabs(ty - gy) < 1e-6 && std::fabs(tz - gz) < 1e-6)) {
    //         goal_connections++;
    //     }
    // }

    // printf("Start Node Connections: %d\n", start_connections);
    // printf("Goal Node Connections: %d\n", goal_connections);
    // printf("----------------------------\n\n");

}

// Graph.grid is the prm.nodes ***
void Planner::run_a_star(){

    // Test print
    printf("run_a_star started\n");
    fflush(stdout);

    /*
     * Stores the is_open, current node's x,y,z, the parent node's x,y,z, the path_cost, the goal_distance, and the goal_distance
     * Index 2, 3, 4 of Matlab code align with curr.x,.y,.z
     * Index 5, 6, 7 of Matlab code align with parent.x,.y,.z
     * The costs are denoted by:
     * g(n) -> cost from the start to current node
     * h(n) -> heuristic cost to goal
     * f(n) -> total estimated cost
     */
    std::vector<AStarNode> open;
    
    /*
     * Stores the node's x,y,z coords.
     */
    std::vector<Node> closed;

    for(const auto& node: prm_graph.get_nodes()){
        /*
         * NThis should initially be 0, no nodes have been visited.
         */
        if(node.get_value() == -1){
            printf("closed nodes: %f, %f, %f", node.get_x(), node.get_y(), node.get_z());
            closed.push_back(node);
        }
    }

    // Get the closed nodes
    uint32_t closed_count = closed.size();

    /*
     * Extract the coordinates from the start and goal node's here for use later. 
     */
    float x_start, y_start, z_start;
    start.get_coords(x_start, y_start, z_start);
    
    float x_node, y_node, z_node;
    start.get_coords(x_node, y_node, z_node);
    
    float x_goal, y_goal, z_goal;
    goal.get_coords(x_goal, y_goal, z_goal);

    printf("goal: %f, %f, %f\n", x_goal, y_goal, z_goal);
    printf("start: %f, %f, %f\n", x_start, y_start, z_start);

    uint32_t open_count = 1;
    float path_cost = 0;
    Node goal{Params::env.goal.x, Params::env.goal.y, Params::env.goal.z};
    float goal_distance = a_star_utils::heuristic_function(start, goal, prm_graph);
    
    /*
     * This calls the constructor, creating the AStarNode object with the start, goal Nodes.
     */
    open.emplace_back(0, start, goal, path_cost, goal_distance, goal_distance);
    
    closed_count++;
    // Add the x,y,z node's actual node, rather than the 3 values in a matrix value form
    closed.push_back(start);

    uint32_t no_path = 1;

    while(((!a_star_utils::AreSame(x_node, x_goal)) || (!a_star_utils::AreSame(y_node, y_goal)) || (!a_star_utils::AreSame(z_node, z_goal))) && no_path == 1){

        // Figure out what type of data structure this should be
        std::vector<AStarNode> exp_array;
        a_star_utils::expand_array(x_node, y_node, z_node, path_cost, closed, prm_graph, exp_array);
        uint32_t exp_count = exp_array.size();

        printf("exp_count size %u\n", exp_count);
        for(uint32_t i = 0; i < exp_count; i++){
            uint32_t flag = 0;

            /*
             * Can substitute open_count with open.size()
             */
            for(uint32_t j = 0; j < open_count; j++){
                
                // The 3 x,y,z of each match, and the cost_function output is the same
                float exp_x, exp_y, exp_z;
                exp_array[i].curr.get_coords(exp_x, exp_y, exp_z);
                // float exp_cost = exp_array[i].g_cost;

                float open_x, open_y, open_z;
                open[j].curr.get_coords(open_x, open_y, open_z);
                // float open_cost = open[j].g_cost;

                if(a_star_utils::AreSame(exp_x, open_x) && a_star_utils::AreSame(exp_y, open_y) && a_star_utils::AreSame(exp_z, open_z)){
                    /*
                     * Store the minimum cost in the open array
                     */
                    // Store the minimum cost in the last index of the open array, comparing current minimum to exp_array new min.
                    open[j].f_cost = std::min(open[j].f_cost, exp_array[j].f_cost);

                    if(open[j].f_cost == exp_array[i].f_cost){
                        /*
                         * open vector is modified at indices 5, 6, 7, 8, 9 -> corresponds to parent's coords, g_cost, and h_cost
                         */
                        open[j].parent.set_coords(x_node, y_node, z_node);
                        open[j].g_cost = exp_array[i].g_cost; // g-cost
                        open[j].h_cost = exp_array[i].h_cost; // h-cost
                    }
                    flag = 1;
                }
            }
            if(flag == 0){
                open_count++;
                // Updating open 
                open.emplace_back(1, exp_array[i].curr.get_x(), exp_array[i].curr.get_y(), exp_array[i].curr.get_z(), x_node, y_node, z_node, exp_array[i].g_cost, exp_array[i].h_cost, exp_array[i].f_cost);
            }
        }

        int index_min_node = a_star_utils::min_fn(open, x_goal, y_goal, z_goal);
        printf("open size: %lu\n", open.size());
        printf("index_min_node: %i\n", index_min_node);
        if(index_min_node != -1){

            /*
             * Updating the minimum with new coords
             */
            open[index_min_node].curr.get_coords(x_node, y_node, z_node);

            path_cost = open[index_min_node].g_cost;

            closed_count++;
            Node new_node{x_node, y_node, z_node};
            closed.push_back(new_node);
            open[index_min_node].is_open = 0; // Mark as Visited
        }
        else{
            no_path = 0;
        }
    }

    // Test print
    printf("here 5\n");
    fflush(stdout);

    /*
     * optimal_path stores x,y,z coordinates as nodes
     */
    std::vector<Node> optimal_path;

    // Get the last node of the closed nodes' x_val, y_val, z_val for comparisons later
    uint32_t i = closed.size() - 1;
    float x_val, y_val, z_val;
    closed[i].get_coords(x_val, y_val, z_val);

    /* 
     * Optimal Path vector begins backward, and is reversed into waypoints.
     */
    optimal_path.push_back(closed[i]);
    i++;

    printf("vals: %f, %f, %f\n", x_val, y_val, z_val);
    printf("goals: %f, %f, %f\n", x_goal, y_goal, z_goal);
    if((x_val == x_goal) && (y_val == y_goal) && (z_val == z_goal)){
        uint32_t inode = 0;

        // Not used: uint32_t idx = 0;
        float parent_x = -1, parent_y = -1, parent_z = -1;
        for(const auto& node : open){
            // node is AStarNode
            Node curr = node.curr;
            Node parent = node.parent;
            if(a_star_utils::AreSame(curr.get_x(), x_val) && a_star_utils::AreSame(curr.get_y(), y_val) && a_star_utils::AreSame(curr.get_z(), z_val)){
                
                // VERIFIED HERE
                // printf("FOUND");
                // fflush(stdout);
                
                parent_x = parent.get_x();
                parent_y = parent.get_y();
                parent_z = parent.get_z();
                break;
            }
        }

        // printf("open_count %u\n", open_count);
        // printf("parent_x %f\n", parent_x);
        // fflush(stdout);
        // printf("parent_y %f\n", parent_y);
        // fflush(stdout);
        // printf("parent_z %f\n", parent_z);
        // fflush(stdout);

        // printf("Looking for parent: (%f, %f, %f)\n", parent_x, parent_y, parent_z);
        // for (uint32_t j = 0; j < open.size(); ++j) {
        //     printf("Open[%d]: (%f, %f, %f)\n", j,
        //         open[j].curr.get_x(), open[j].curr.get_y(), open[j].curr.get_z());
        // }

        /**
         * TODO(): FIGURE OUT WHY THIS IS GIVING AN INFINITE FAILURE EVEN FOR FLOAT COMPARISONS
         */
        while((!a_star_utils::AreSame(parent_x, x_start)) || (!a_star_utils::AreSame(parent_y, y_start)) || (!a_star_utils::AreSame(parent_z, z_start))){
            
            // printf("here %u\n", i);
            // fflush(stdout);

            /*
             * The additional parents between the start and the goal while backtracking.
             */
            Node parent_node{parent_x, parent_y, parent_z};
            optimal_path.push_back(parent_node);

            // inode = find(open(:,2) == parent_x & open(:,3) == parent_y & open(:,4) == parent_z);
            for(uint32_t j = 0; j < open.size(); j++){
                if(a_star_utils::AreSame(open[j].curr.get_x(), parent_x) && a_star_utils::AreSame(open[j].curr.get_y(), parent_y) && a_star_utils::AreSame(open[j].curr.get_z(), parent_z)){
                    inode = j;
                    parent_x = open[inode].parent.get_x();
                    parent_y = open[inode].parent.get_y();
                    parent_z = open[inode].parent.get_z();
                    // printf("here %u\n", i);
                    // fflush(stdout);
                    break;
                }
            }

            i++;
        }

        // optimal_path(i,:) = [x_start y_start z_start];
        Node new_start{x_start, y_start, z_start};
        optimal_path.push_back(new_start);

        // This gets you the reversed optimal_path: waypoints = optimal_path((end:-1:1),:);
        waypoints = optimal_path;
        std::reverse(waypoints.begin(), waypoints.end());

        printf("here 3\n");
        fflush(stdout);
        for(uint32_t i = 0; i < waypoints.size(); i++){

            printf("points: %f, %f, %f\n", waypoints[i].get_x(), waypoints[i].get_y(), waypoints[i].get_z());
            fflush(stdout);

            // Pass these by reference to the cost_function()
            float cost = 0;
            uint32_t mode = 0;

            float x, y, z;
            waypoints[i].get_coords(x, y, z);
            float x_1, y_1, z_1;
            waypoints[i+1].get_coords(x_1, y_1, z_1);
            // Input the specific node, and the next node
            a_star_utils::cost_function(x, y, z, x_1, y_1, z_1, prm_graph, cost, mode);
            waypoints[i].set_value(mode);
            total_cost += cost;
        }
        
    }
    else{
        /*
         * NO PATH
         */
        printf("no path 5\n");
        fflush(stdout);
    }
    
}