#!/usr/bin/env bash

# This file is used to create the commands in the chroot environment. This file is separated
# from the setup file, because changes in the commands should not require the setup script
# to be run again.
# 
# It defines a total of six commands

USERNAME=$(whoami)
SCHROOT_LOCATION=""
ROS1_LOCATION="/opt/ros/melodic"
ROS2_LOCATION="/home/$USERNAME/ros2_foxy"
MARCH_LOCATION="/home/$USERNAME/Desktop/march"

sudo rm $SCHROOT_LOCATION/usr/bin/march_* -f

sudo tee <<EOF $SCHROOT_LOCATION/usr/bin/march_build_bridge >/dev/null
#!/usr/bin/env zsh
source $ROS1_LOCATION/setup.zsh &&
source $ROS2_LOCATION/install/setup.zsh &&
source $MARCH_LOCATION/ros1/install/setup.zsh &&
source $MARCH_LOCATION/ros2/install/local_setup.zsh &&
cd $ROS2_LOCATION &&
export CC=gcc &&
export CXX=g++ &&
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure &&
cd $MARCH_LOCATION
EOF
sudo chmod 755 $SCHROOT_LOCATION/usr/bin/march_build_bridge

sudo tee <<EOF $SCHROOT_LOCATION/usr/bin/march_build_ros1 >/dev/null
#!/usr/bin/env zsh
source $ROS1_LOCATION/setup.zsh &&
cd $MARCH_LOCATION/ros1 &&
export CC=gcc &&
export CXX=g++ &&
colcon build --symlink-install --cmake-force-configure
EOF
sudo chmod 755 $SCHROOT_LOCATION/usr/bin/march_build_ros1

sudo tee <<EOF $SCHROOT_LOCATION/usr/bin/march_run_ros1 >/dev/null
#!/usr/bin/env zsh
source $ROS1_LOCATION/setup.zsh &&
cd $MARCH_LOCATION/ros1 &&
source install_isolated/setup.zsh &&
roslaunch march_launch march_ros2_simulation.launch
EOF
sudo chmod 755 $SCHROOT_LOCATION/usr/bin/march_run_ros1

sudo tee <<EOF $SCHROOT_LOCATION/usr/bin/march_run_bridge >/dev/null
#!/usr/bin/env zsh
source $ROS1_LOCATION/setup.zsh &&
source $ROS2_LOCATION/install/setup.zsh &&
export ROS_MASTER_URI=http://localhost:11311 &&
cd $ROS2_LOCATION
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
EOF
sudo chmod 755 $SCHROOT_LOCATION/usr/bin/march_run_bridge

sudo tee <<EOF $SCHROOT_LOCATION/usr/bin/march_build_ros2 >/dev/null
#!/usr/bin/env zsh
source $ROS2_LOCATION/install/setup.zsh &&
cd $MARCH_LOCATION/ros2 &&
export CC=gcc &&
export CXX=g++ &&
colcon build --symlink-install --cmake-force-configure
EOF
sudo chmod 755 $SCHROOT_LOCATION/usr/bin/march_build_ros2

sudo tee <<EOF $SCHROOT_LOCATION/usr/bin/march_run_ros2 >/dev/null
#!/usr/bin/env zsh
source $ROS2_LOCATION/install/setup.zsh &&
cd $MARCH_LOCATION/ros2 &&
source install/setup.zsh &&
ros2 launch march_launch march_ros2_simulation.launch.py
EOF
sudo chmod 755 $SCHROOT_LOCATION/usr/bin/march_run_ros2
