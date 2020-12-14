#include <march_pcl_conversion/pcl_conversion_node.h>
#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

PclConversionNode::PclConversionNode() : rclcpp::Node::Node("march_pcl_conversion_node")
{
  std::string topic_name;
  // Temporary solution to switch between connection with simulation or with physical camera
  topic_name = "/camera/depth/color/points"; // Simulation
  topic_name = "/camera/pointcloud";  // Physical camera
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "test", 10, std::bind(&PclConversionNode::pointcloud_callback, this, std::placeholders::_1)
  );
}

void PclConversionNode::pointcloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard something: '%s'", msg->header);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclConversionNode>());
  rclcpp::shutdown();
  return 0;
}