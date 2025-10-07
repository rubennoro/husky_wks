#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "husky_planner/planner.h"

class PathPlannerNode1 : public rclcpp::Node
{
public:
    PathPlannerNode1() : Node("path_planner_node1")
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("planned_path_marker1", 10);
        node_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("node_marker", 10); // Publisher for nodes

        RCLCPP_INFO(this->get_logger(), "Before main_plan");
        planner_ = std::make_shared<Planner>();
        RCLCPP_INFO(this->get_logger(), "Before main_plan");
        planner_->main_plan();
        RCLCPP_INFO(this->get_logger(), "After main_plan");

        publishPathMarkers();
        publishNodeMarkers(); // Publish nodes and edges
    }

private:
    void publishPathMarkers()
    {
        const auto& waypoints = planner_->get_waypoints();
        // const auto cost = planner_->get_cost();
        if (waypoints.empty()) {
            RCLCPP_WARN(this->get_logger(), "No path found to publish!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", waypoints.size());

        // Publish path markers (same as before)
        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.header.stamp = this->now();
        line_strip.ns = "planned_path";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;
        line_strip.scale.x = 0.1;
        line_strip.color.r = 0.0;
        line_strip.color.g = 1.0;
        line_strip.color.b = 0.0;
        line_strip.color.a = 1.0;

        for (const auto& node : waypoints)
        {
            geometry_msgs::msg::Point p;
            float x, y, z;
            node.get_coords(x, y, z);
            p.x = x;
            p.y = y;
            p.z = z;
            line_strip.points.push_back(p);
        }

        marker_pub_->publish(line_strip);
        RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", waypoints.size());
    }

    void publishNodeMarkers()
    {
        const auto& graph = planner_->get_graph(); // Get the graph that contains all the nodes and edges
        visualization_msgs::msg::Marker node_marker;
        node_marker.header.frame_id = "map";
        node_marker.header.stamp = this->now();
        node_marker.ns = "all_nodes";
        node_marker.id = 1;
        node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        node_marker.action = visualization_msgs::msg::Marker::ADD;
        node_marker.scale.x = 0.2; // Sphere size for nodes
        node_marker.scale.y = 0.2;
        node_marker.scale.z = 0.2;
        node_marker.color.r = 0.0;
        node_marker.color.g = 0.0;
        node_marker.color.b = 1.0; // Blue for nodes
        node_marker.color.a = 1.0;

        for (const auto& node : graph.get_nodes()) // Assuming you have a get_nodes() method
        {
            geometry_msgs::msg::Point p;
            float x, y, z;
            node.get_coords(x, y, z);
            p.x = x;
            p.y = y;
            p.z = z;
            node_marker.points.push_back(p);
        }

        node_pub_->publish(node_marker);

        // Now publish the edges as lines (connections between nodes)
        visualization_msgs::msg::Marker edge_marker;
        edge_marker.header.frame_id = "map";
        edge_marker.header.stamp = this->now();
        edge_marker.ns = "all_edges";
        edge_marker.id = 2;
        edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edge_marker.action = visualization_msgs::msg::Marker::ADD;
        edge_marker.scale.x = 0.05; // Line width for edges
        edge_marker.color.r = 1.0; // Red for edges
        edge_marker.color.g = 0.0;
        edge_marker.color.b = 0.0;
        edge_marker.color.a = 1.0;

        for (const auto& edge : graph.get_edges()) // Assuming you have a get_edges() method
        {
            geometry_msgs::msg::Point p1, p2;
            float x1, y1, z1, x2, y2, z2;
            edge.from.get_coords(x1, y1, z1); // Get start node coordinates
            edge.to.get_coords(x2, y2, z2);   // Get end node coordinates

            p1.x = x1;
            p1.y = y1;
            p1.z = z1;
            p2.x = x2;
            p2.y = y2;
            p2.z = z2;

            edge_marker.points.push_back(p1);
            edge_marker.points.push_back(p2);
        }

        marker_pub_->publish(edge_marker);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr node_pub_;
    std::shared_ptr<Planner> planner_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode1>());
    rclcpp::shutdown();
    return 0;
}
