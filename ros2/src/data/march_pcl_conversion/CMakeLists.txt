cmake_minimum_required(VERSION 3.5)
project(march_pcl_conversion)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Boost 1.55.0 REQUIRED COMPONENTS system filesystem)

include_directories(include ${Boost_INCLUDE_DIRS})

add_executable(march_pcl_conversion_node src/pcl_conversion_node.cpp)
ament_target_dependencies(march_pcl_conversion_node rclcpp sensor_msgs std_msgs pcl_conversions Boost)

ament_export_include_directories(include)

install(TARGETS
  march_pcl_conversion_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
