#!/usr/bin/env bash

USERNAME=$(whoami)

function print_info ()
{
    echo -e "\033[0;93m[MARCH] $1\033[0;0m"
}

function check_error ()
{
    ERROR_CODE=$?
    if [ $ERROR_CODE -ne 0 ]
    then
        exit $ERROR_CODE
    fi
}

print_info "Build March specific ROS 2 dependencies..."
# Delete old folders
schroot -d "/home/$USERNAME" -c ros -- zsh -c "cd /home/$USERNAME/march/.ros2_foxy/src/ros2 && rm xacro urdf_parser_py control_msgs -rf"
check_error
# Clone the folders and build them
schroot -d "/home/$USERNAME" -c ros -- zsh -c "
cd /home/$USERNAME/march/.ros2_foxy/src/ros2 &&
git clone https://github.com/ros/xacro.git -b dashing-devel &&
git clone https://github.com/ros/urdf_parser_py.git -b ros2 &&
git clone https://github.com/ros-controls/control_msgs.git -b foxy-devel &&
cd /home/$USERNAME/march/.ros2_foxy &&
colcon build --packages-select urdfdom_py xacro control_msgs"
check_error
