#!/usr/bin/env bash

# This file is used to create the commands in the chroot environment. This file is separated
# from the setup file, because changes in the commands should not require the setup script
# to be run again.
# 
# It defines a total of six commands

USERNAME=$(whoami)
ROS_LOCATION="/srv/chroot/ros"

sudo rm $ROS_LOCATION/usr/bin/march_* -f

sudo tee <<EOF $ROS_LOCATION/usr/bin/march_build_bridge >/dev/null
#!/usr/bin/env zsh
source /opt/ros/melodic/setup.zsh &&
source /home/$USERNAME/march/.ros2_foxy/install/setup.zsh &&
source /home/$USERNAME/march/ros1/install/setup.zsh &&
source /home/$USERNAME/march/ros2/install/local_setup.zsh &&
cd /home/$USERNAME/march/.ros2_foxy &&
export CC=gcc &&
export CXX=g++ &&
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure &&
cd /home/$USERNAME/march
EOF
sudo chmod 755 $ROS_LOCATION/usr/bin/march_build_bridge

sudo tee <<EOF $ROS_LOCATION/usr/bin/march_build_ros1 >/dev/null
#!/usr/bin/env zsh
source /opt/ros/melodic/setup.zsh &&
cd /home/$USERNAME/march/ros1 &&
export CC=gcc &&
export CXX=g++ &&
colcon build --symlink-install --cmake-force-configure
EOF
sudo chmod 755 $ROS_LOCATION/usr/bin/march_build_ros1

sudo tee <<EOF $ROS_LOCATION/usr/bin/march_run_ros1 >/dev/null
#!/usr/bin/env zsh
source /opt/ros/melodic/setup.zsh &&
cd /home/$USERNAME/march/ros1 &&
source install/setup.zsh &&
roslaunch march_launch march_ros2_simulation.launch
EOF
sudo chmod 755 $ROS_LOCATION/usr/bin/march_run_ros1

sudo tee <<EOF $ROS_LOCATION/usr/bin/march_run_bridge >/dev/null
#!/usr/bin/env zsh
source /opt/ros/melodic/setup.zsh &&
source /home/$USERNAME/march/.ros2_foxy/install/setup.zsh &&
export ROS_MASTER_URI=http://localhost:11311 &&
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
EOF
sudo chmod 755 $ROS_LOCATION/usr/bin/march_run_bridge

sudo tee <<EOF $ROS_LOCATION/usr/bin/march_build_ros2 >/dev/null
#!/usr/bin/env zsh
source /home/$USERNAME/march/.ros2_foxy/install/setup.zsh &&
cd /home/$USERNAME/march/ros2 &&
export CC=gcc &&
export CXX=g++ &&
colcon build --symlink-install --cmake-force-configure
EOF
sudo chmod 755 $ROS_LOCATION/usr/bin/march_build_ros2

sudo tee <<EOF $ROS_LOCATION/usr/bin/march_run_ros2 >/dev/null
#!/usr/bin/env zsh
source /home/$USERNAME/march/.ros2_foxy/install/setup.zsh &&
cd /home/$USERNAME/march/ros2 &&
source install/setup.zsh &&
ros2 launch march_launch march_ros2_simulation.launch.py
EOF
sudo chmod 755 $ROS_LOCATION/usr/bin/march_run_ros2
