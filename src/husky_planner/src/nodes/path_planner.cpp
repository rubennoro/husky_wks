#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "husky_planner/planner.h"

using namespace std::chrono_literals;

/**
 * This is a publisher node that runs the PRM / A* planner and produces a set of waypoints
 * that are connected and visualized via a line in Rviz2. 
 */
class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode() : Node("path_planner_node")
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("planned_path_marker", 10);
        
        RCLCPP_INFO(this->get_logger(), "Before main_plan");
        planner_ = std::make_shared<Planner>();
        RCLCPP_INFO(this->get_logger(), "Before main_plan");
        planner_->main_plan();

        RCLCPP_INFO(this->get_logger(), "After main_plan");
        timer_ = this->create_wall_timer(
            500ms, // Publish every 500ms
            std::bind(&PathPlannerNode::publishPathMarkers, this)
        );
    }

private:
    void publishPathMarkers()
    {
        /*
         * Print the waypoints to the log.
         */
        const auto& waypoints = planner_->get_waypoints();  // We'll add this getter next.
        const auto cost = planner_->get_cost();
        if (waypoints.empty()) {
            RCLCPP_WARN(this->get_logger(), "No path found to publish!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", waypoints.size());

        for (size_t i = 0; i < waypoints.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Waypoint %zu: x=%.2f, y=%.2f, z=%.2f",
                i,
                waypoints[i].get_x(),
                waypoints[i].get_y(),
                waypoints[i].get_z());
        }
        
        RCLCPP_INFO(this->get_logger(), "Total Cost: %.2f\n", cost);

        /*
         * Visualize the connected waypoints.
         */
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.header.stamp = this->now();
        line_strip.ns = "planned_path";
        line_strip.id = 0;
        line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::msg::Marker::ADD;

        line_strip.scale.x = 0.1;  // Line width

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
        marker_array.markers.push_back(line_strip);

        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published path with %zu waypoints", waypoints.size());
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<Planner> planner_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
