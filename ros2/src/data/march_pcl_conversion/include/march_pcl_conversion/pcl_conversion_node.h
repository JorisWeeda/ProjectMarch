#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>

#ifndef MARCH_GAZEBO_PLUGINS_PCL_CONVERSION_NODE_H
#define MARCH_GAZEBO_PLUGINS_PCL_CONVERSION_NODE_H

class PclConversionNode : public rclcpp::Node
{
public:
  PclConversionNode();
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr);
};

#endif  // MARCH_GAZEBO_PLUGINS_PCL_CONVERSION_NODE_H
