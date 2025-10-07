#include "husky_planner/prm_utils/prm_utils.h"
#include <cmath>
#include <random>


namespace prm_utils{

    float randomZeroToOne() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<float> dis(0.0f, 1.0f);
        return dis(gen);
    }

    bool AreSame(float a, float b)
    {
        constexpr float EPSILON = 1e-5f;
        return fabs(a - b) < EPSILON;
    }
    
    float distance(Node n1, Node n2){
        float dist = std::sqrt(std::pow(n1.get_x() - n2.get_x(), 2) + std::pow(n1.get_y()-n2.get_y(), 2)+std::pow(n1.get_z()-n2.get_z(), 2));
        if(dist < 0){
            return 0;
        }
        return dist;
    }

    /*
     * This modifies the nearest_nodes vector, getting the distances of the nodes closest to the new_node.
     */
    void find_nearest_node(const Node &new_node, const Graph &prm_graph, std::vector<Node> &nearest_nodes){

        /*
         * Get the nodes in the graph's nodes list that are within the range of the new node.
         */
        for(uint32_t i = 0; i < prm_graph.num_nodes() ; i++){
            if(distance(prm_graph.get_nodes()[i], new_node) <= Params::prm.radius){
                nearest_nodes.push_back(prm_graph.get_nodes()[i]);
            }
        }
        /*
         * Add the distance to the nodes info.
         */
        for(auto &node : nearest_nodes){
            /*
             * Recompute distance.
             */
            float dist = distance(node, new_node);

            /**
             * Update distance attribute of the node
             */
            node.add_distance(dist);
        }
        
    }

    bool obstacles_free(Node node){
        bool result = true;
        // int node_value = 0;

        float x = node.get_x();
        float y = node.get_y();
        float z = node.get_z();

        float clearance = Params::env.clearance.obstacles;
        float landing_clearance = Params::env.clearance.landing_area;
        float platform_clearance = Params::env.clearance.platform;

        const auto& blocks = Params::env.blocks;
        const auto& platform = Params::env.platform;

        /**
         * The blocks vectors contain the obstacles 1-10 and the goal platform, don't count the goal_platform here subtract 1.
         */
        for(uint32_t i = 0; i < blocks.x_min.size() - 1; i++){
            if(x >= blocks.x_min[i] - clearance && x <= blocks.x_max[i] + clearance && y >= blocks.y_min[i] - clearance && y <= blocks.y_max[i] + clearance && z >= blocks.z_min[i] - clearance && z <= blocks.z_max[i] + clearance){
                // node_value = -1;
                result = false;
                return false;
            }

            // Check platform
            if (x >= platform.x_min - clearance && x <= platform.x_max + clearance &&
                y >= platform.y_min - clearance && y <= platform.y_max + clearance &&
                z >= platform.z_min - platform_clearance && z <= platform.z_max + platform_clearance &&
                platform.active_platform == 1) 
            {
                // Inside platform area but check landing clearance
                if (!(x >= platform.x_min + landing_clearance && x <= platform.x_max - landing_clearance &&
                    y >= platform.y_min + landing_clearance && y <= platform.y_max - landing_clearance &&
                    z >= platform.z_min && z <= platform.z_max + platform_clearance)) 
                {
                    // node_value = -1;
                    result = false;
                    return false;
                }
            }
        }
        return result;
    }

    /*
     * Checks if the segment along the path between two nodes is available for travel.
     */
    bool free_segment(const Node &from_node, const Node &to_node){

        /*
         * Find 10 evenly spaced points between the from_node and the to_node.
         */ 
        uint32_t interpolate_samples = 10;

        for(uint32_t i = 0; i < interpolate_samples; i++){

            /*
             * Percentage distance between the two nodes for a sample.
             */ 
            float t = static_cast<float>(i) / (interpolate_samples - 1);

            float x = (1 - t) * from_node.get_x() + t * to_node.get_x();
            float y = (1 - t) * from_node.get_y() + t * to_node.get_y();
            float z = (1 - t) * from_node.get_z() + t * to_node.get_z();
            Node point{x, y, z};
            
            if (!obstacles_free(point)) {
                return false;
            }
        }

        return true;
    }
}