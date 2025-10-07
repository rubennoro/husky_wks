#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "husky_planner/environment/environment.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class EnvironmentPublisher : public rclcpp::Node
{
  public:
    EnvironmentPublisher()
    : Node("environment_pub")
    {
      marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("environment_markers", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&EnvironmentPublisher::publishMarkers, this));

      bodies_ = Map::load_environment_obstacles();
    }

  private:
    void publishMarkers(){
        visualization_msgs::msg::MarkerArray marker_array;
        uint32_t id = 0;

        for(const auto& body : bodies_){
            visualization_msgs::msg::Marker marker;

            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "environment_obstacles";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = (body.x_min + body.x_max) / 2.0;
            marker.pose.position.y = (body.y_min + body.y_max) / 2.0;
            marker.pose.position.z = (body.z_min + body.z_max) / 2.0;

            marker.pose.orientation.w = 1.0;

            marker.scale.x = body.x_max - body.x_min;
            marker.scale.y = body.y_max - body.y_min;
            marker.scale.z = body.z_max - body.z_min;

            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.8f;

            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
        RCLCPP_INFO(this->get_logger(), "Published %zu obstacles as markers", bodies_.size());
    };
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::vector<Body> bodies_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnvironmentPublisher>());
  rclcpp::shutdown();
  
  return 0;
}