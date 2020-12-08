#include "../include/march_pcl_conversion/pcl_conversion_node.h"
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class PclConversionNode : public rclcpp::Node
{
public:
  PclConversionNode() : Node("pcl_conversion_node")
  {
    std::string topic_name;
    // Temporary solution to switch between connection with simulation or with physical camera
    topic_name = "/camera/depth/color/points"; // Simulation
//    topic_name = "/camera/pointcloud";  // Physical camera
    subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic_name, 10, std::bind(&PclConversionNode::pointcloud_callback, this, _1)
        );
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard something");
  }
};