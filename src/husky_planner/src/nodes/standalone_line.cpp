#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

/**
 * Testing a basic line publisher. 
 */
class SimpleMarkerNode : public rclcpp::Node
{
public:
    SimpleMarkerNode() : Node("simple_marker_node")
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("simple_marker_topic", 10);

        // Set up a timer to periodically publish the marker
        timer_ = this->create_wall_timer(
            500ms, // Publish every 500ms
            std::bind(&SimpleMarkerNode::publish_marker, this)
        );
    }

private:
    void publish_marker()
    {
        // Create the MarkerArray
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // Ensure "map" frame is used
        marker.header.stamp = this->now();
        marker.ns = "simple_marker";
        marker.id = 0;  // Use 0 or unique ids
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 1.0;
        marker.pose.position.y = 1.0;
        marker.pose.position.z = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // Add the marker to the array
        marker_array.markers.push_back(marker);

        // Publish the marker array
        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published marker at (%.2f, %.2f, %.2f)", marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleMarkerNode>());
    rclcpp::shutdown();
    
    return 0;
}

