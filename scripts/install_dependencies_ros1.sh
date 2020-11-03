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

# Install March specific ROS 1 dependencies
print_info "Update ROS dependencies list..."
schroot -d "/home/$USERNAME" -c ros -- zsh -c "rosdep update"
check_error

print_info "Install March specific ROS 1 dependencies..."
sudo schroot -d "/home/$USERNAME" -c ros -- zsh -c "sudo chmod -R 755 /opt"
schroot -d "/home/$USERNAME" -c ros -- zsh -c "source /opt/ros/melodic/setup.zsh; rosdep install -y --from-paths /home/$USERNAME/march/ros1/src --ignore-src"
check_error

