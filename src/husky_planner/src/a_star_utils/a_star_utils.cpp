#include "husky_planner/a_star_utils/a_star_utils.h"
#include "husky_planner/params.h"
#include <cmath>
#include <cstdio>
#include <limits>

namespace a_star_utils{

    bool AreSame(float a, float b)
    {
        constexpr float EPSILON = 1e-5f;
        return fabs(a - b) < EPSILON;
    }

    void cost_function(float x_1, float y_1, float z_1, float x_2, float y_2, float z_2, const Graph &prm_graph, float &cost, uint32_t &mode){

        /*
         * Which nodes from the prm graph match the inputted coordinates.
         */
        Node node_one;
        Node node_two;
        /*
         * Graph.grid in MATLAB is equal to the prm_graph.nodes.
         */
        for(const auto& node : prm_graph.get_nodes()){
            /*
             * Get the coordinates from the given graph node.
             */
            float x, y, z;
            node.get_coords(x, y, z);

            /*
             * Compare x_1, y_1, z_1 coords with the node's coords.
             */
            float tolerance = 1e-6;
            if(std::fabs(x_1 - x) <= tolerance &&
                std::fabs(y_1 - y) <= tolerance &&
                std::fabs(z_1 - z) <= tolerance){
                    node_one = node;
            }

            /*
             * Compare x_2, y_2, z_2 coords with the node's coords.
             */
            if(std::fabs(x_2 - x) <= tolerance &&
                std::fabs(y_2 - y) <= tolerance &&
                std::fabs(z_2 - z) <= tolerance){
                    node_two = node;
            }
        }

        /*
         * Get the node_value from the matching node to input 1.
         */
        int node_value1 = node_one.get_value();

        /*
         * Get the node_value from the matching node to input 2.
         */
        int node_value2 = node_two.get_value();

        /*
         * Perform distance operation.
         */
        Node n1{x_1, y_1, z_1};
        Node n2{x_2, y_2, z_2};
        float dist = distance(n1, n2);

        switch(node_value1){
            case 1:
                switch(node_value2){
                    case 1:
                        // cost = dist * robot.leg.COT * robot.mass * robot.world.g;
                        cost = dist * Params::leg.COT * Params::physical.mass * Params::world.g;
                        mode = 1;
                        break;
                    case 2:
                        // cost = dist * robot.fly.power_consumption / robot.fly.speed...
                        // + robot.mass * robot.world.g * (z_2 - z_1); % cost leg/flying + To Do transition cost?
                        // cost = cost + robot.costs.leg_to_fly;
                        cost = dist * (Params::fly.power_consumption / Params::fly.speed) + Params::physical.mass * Params::world.g * (z_2 - z_1);
                        cost += Params::transition.leg_to_fly;
                        mode = 3;
                        break;
                }
                break;
            case 2:
                switch(node_value2){
                    case 1:
                        // cost = dist * robot.fly.power_consumption / robot.fly.speed...
                        //     + robot.mass * robot.world.g * (z_2 - z_1); 
                        // cost = cost + robot.costs.fly_to_leg;
                        cost = dist * Params::fly.power_consumption / Params::fly.speed + Params::physical.mass * Params::world.g * (z_2 - z_1);
                        cost += Params::transition.fly_to_leg;
                        mode = 4;
                        break;
                    case 2:
                        cost = dist * Params::fly.power_consumption / Params::fly.speed + Params::physical.mass * Params::world.g * (z_2 - z_1);
                        // cost = dist * robot.fly.power_consumption / robot.fly.speed...
                        // + robot.mass * robot.world.g * (z_2 - z_1);
                        mode = 2;
                        break;
                }
                break;
            default:
                break;
        }
        
    }

    /*
     * General euclidean distance function.
     */
    float distance(Node n1, Node n2){
        float dist = std::sqrt(std::pow(n1.get_x() - n2.get_x(), 2) + std::pow(n1.get_y()-n2.get_y(), 2) + std::pow(n1.get_z()-n2.get_z(), 2));
        if(dist < 0){
            return 0;
        }
        return dist;
    }

    /*
     * Tracking distance in 2D, negating the height coordinates. 
     */
    float distance_2d(Node n1, Node n2){
        float dist = std::sqrt(std::pow(n1.get_x() - n2.get_x(), 2) + std::pow(n1.get_y()-n2.get_y(), 2));
        if(dist < 0){
            return 0;
        }
        return dist;
    }

    /*
     * Pass input array exp_array by reference.
     */
    void expand_array(float x, float y, float z, float hn, const std::vector<Node> &closed, const Graph &prm_graph, std::vector<AStarNode> &exp_array){
        
        uint32_t exp_count = 1;

        std::vector<Edge> edges_from_node;
        std::vector<Edge> edges_to_node;

        float tolerance = 1e-6;

        /*
         * Each edge has a from Node and a to Node
         */
        for (const auto& edge : prm_graph.get_edges()) {
            float fx = edge.from.get_x();
            float fy = edge.from.get_y();
            float fz = edge.from.get_z();

            float tx = edge.to.get_x();
            float ty = edge.to.get_y();
            float tz = edge.to.get_z();

            /*
             * Find out if the current FROM node is the current node's x,y,z
             */
            if (std::fabs(fx - x) <= tolerance &&
                std::fabs(fy - y) <= tolerance &&
                std::fabs(fz - z) <= tolerance) {
                edges_from_node.push_back(edge);
            }

            /*
             * Find out if the current TO node is the current node's x,y,z
             */
            if (std::fabs(tx - x) <= tolerance &&
                std::fabs(ty - y) <= tolerance &&
                std::fabs(tz - z) <= tolerance) {
                edges_to_node.push_back(edge);
            }
        }

        /*
         * Storing edges here.
         */
        std::vector<Edge> neighbors;

        /*
         * Get the current node's edges in the original direction
         */
        for (const auto& edge : edges_from_node) {
            neighbors.emplace_back(edge.from, edge.to);  // same direction
        }

        /*
         * Get the current node's edges in the opposite direction.
         */
        for (const auto& edge : edges_to_node) {
            neighbors.emplace_back(edge.to, edge.from);  // reversed
        }
        
        /*
         * Neighbors is a combination of the FROM -> TO edges and the TO -> FROM edges
         * For FROM->TO edges, the current node is FROM.
         * For TO->FROM flipped edges, the current node was TO, but is changed to be FROM. 
         * 
         * This loop gets the coordinates of a neighbor, sees if it exists in closed. If it
         * a node matches one or more of the closed nodes, it will move to the next neighbor,
         * setting the flag to 0. 
         */
        for(const auto& neighbor : neighbors){
            // .first -> from
            // .second -> to
            Node n_node = neighbor.to;

            /*
             * The TO node's x,y,z coords.
             */
            float s_x, s_y, s_z;
            n_node.get_coords(s_x, s_y, s_z);

            std::vector<Node> matching_nodes;
            for(const auto& node : closed){
                if(std::fabs(node.get_x() - s_x) <= tolerance && std::fabs(node.get_y() - s_y) <= tolerance && std::fabs(node.get_z() - s_z) <= tolerance){
                    matching_nodes.push_back(node);
                }
            }
            uint32_t flag = 1;

            /*
             * If the neighbor matches a closed node, it is already closed, no need to re-visit it. 
             */  
            if(matching_nodes.size() != 0){
                flag = 0;
            }
            if(flag == 1){
                float new_hn;
                uint32_t mode;
                /*
                 * Parameters are the current node's coords, the neighbor coord's, and some pass by ref.
                 */
                cost_function(x, y, z, s_x, s_y, s_z, prm_graph, new_hn, mode);
                new_hn += hn;

                // Pass the node's second and map.goal to this instead of the x,y,z values
                Node goal_node{Params::env.goal.x, Params::env.goal.y, Params::env.goal.z};
                float heuristic = heuristic_function(n_node, goal_node, prm_graph);
                
                /*
                 * Total cost.
                 */
                float total_cost = new_hn + heuristic;

                exp_array.emplace_back(s_x, s_y, s_z, new_hn, heuristic, total_cost);
                exp_count++;
            }
        }
    }

    float heuristic_function(Node n1, Node n2, const Graph &prm_graph){

        float cost = 0;

        /*
         * Get Coordinates for the start and goal coordinates
         */
        float x_1, y_1, z_1;
        n1.get_coords(x_1, y_1, z_1);
        
        float x_2, y_2, z_2;
        n2.get_coords(x_2, y_2, z_2);

        Node near_n1;
        Node near_n2;

        /*
         * Getting other nodes that are in close proximity to the current node
         */
        float tol = 1e-6;
        for(const auto& node : prm_graph.get_nodes()){
            if((std::abs(node.get_x() - x_1) <= tol) && (std::abs(node.get_y() - y_1) <= tol) && (std::abs(node.get_z() - z_1) <= tol)){
                near_n1 = node;
            }
            if((std::abs(node.get_x() - x_2) <= tol) && (std::abs(node.get_y() - y_2) <= tol) && (std::abs(node.get_z() - z_2) <= tol)){
                near_n2 = node;
            }
        }

        float ground_dist = distance_2d(n1, n2);
        float flying_dist = std::abs(n2.get_z() - n1.get_z());

        // Cost calculated by ground_dist, flying_dist
        cost = ground_dist * Params::leg.COT * Params::physical.mass * Params::world.g + flying_dist * Params::fly.power_consumption / Params::fly.speed + Params::physical.mass * Params::world.g * (n2.get_z() - n1.get_z());

        return cost;
    }

    int min_fn(const std::vector<AStarNode> &open, float goal_x, float goal_y, float goal_z){

        int flag = 0;
        int goal_index = 0;
        /*
         * To determine the minimum f_cost, store the specific index as well within a pair.
         */
        std::vector<std::pair<AStarNode, int>> temp_array;

        for(uint32_t j = 0; j < open.size(); j++){
            if(open[j].is_open == 1){
                temp_array.push_back({open[j], j});
                if(AreSame(open[j].curr.get_x(), goal_x) && AreSame(open[j].curr.get_y(), goal_y) && AreSame(open[j].curr.get_z(), goal_z)){
                    flag = 1;
                    goal_index = j;
                }
            }
        }
        if(flag == 1){
            return goal_index;
        }
        int i_min = -1;
        if(temp_array.size() != 0){
            float min_cost = std::numeric_limits<float>::infinity();
            for(const auto& pair : temp_array){
                const AStarNode& node = pair.first;
                uint32_t idx = pair.second;

                if(node.f_cost < min_cost){
                    min_cost = node.f_cost;
                    i_min = idx;
                }
            }
        }
        return i_min;
    }
}