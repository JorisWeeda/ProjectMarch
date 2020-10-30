#!/usr/bin/env bash

USERNAME=$(whoami)
ROS1_LOCATION="/srv/chroot/ros1"
ROS2_LOCATION="/srv/chroot/ros2"

sudo rm $ROS1_LOCATION/usr/bin/march_* -f
sudo rm $ROS2_LOCATION/usr/bin/march_* -f

sudo tee <<EOF $ROS2_LOCATION/usr/bin/march_build_bridge >/dev/null
#!/usr/bin/env zsh
source /opt/ros/melodic/local_setup.zsh;
source /opt/ros/foxy/local_setup.zsh;
source /home/$USERNAME/march/.ros2_foxy/install/local_setup.zsh;
source /home/$USERNAME/march/ros1/install_isolated/local_setup.zsh;
source /home/$USERNAME/march/ros2/install/local_setup.zsh;
cd /home/$USERNAME/march/.ros2_foxy;
export CC=gcc;
export CXX=g++;
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure;
cd /home/$USERNAME/march
EOF
sudo chmod 755 $ROS2_LOCATION/usr/bin/march_build_bridge

sudo tee <<EOF $ROS1_LOCATION/usr/bin/march_build_ros1 >/dev/null
#!/usr/bin/env zsh
source /opt/ros/melodic/local_setup.zsh;
cd /home/$USERNAME/march/ros1;
export CC=gcc;
export CXX=g++;
catkin_make_isolated --install
EOF
sudo chmod 755 $ROS1_LOCATION/usr/bin/march_build_ros1

sudo tee <<EOF $ROS1_LOCATION/usr/bin/march_run_ros1 >/dev/null
#!/usr/bin/env zsh
source /opt/ros/melodic/setup.zsh;
cd /home/$USERNAME/march/ros1;
source install_isolated/setup.zsh;
roslaunch march_launch march_ros2_simulation.launch
EOF
sudo chmod 755 $ROS1_LOCATION/usr/bin/march_run_ros1

sudo tee <<EOF $ROS2_LOCATION/usr/bin/march_run_bridge >/dev/null
#!/usr/bin/env zsh
source /opt/ros/melodic/local_setup.zsh;
source /home/$USERNAME/march/ros1/install_isolated/local_setup.zsh;
source /opt/ros/foxy/local_setup.zsh;
source /home/$USERNAME/march/.ros2_foxy/install/local_setup.zsh;
source /home/$USERNAME/march/ros2/install/local_setup.zsh;
export ROS_MASTER_URI=http://localhost:11311;
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
EOF
sudo chmod 755 $ROS2_LOCATION/usr/bin/march_run_bridge

sudo tee <<EOF $ROS2_LOCATION/usr/bin/march_build_ros2 >/dev/null
#!/usr/bin/env zsh
source /opt/ros/foxy/local_setup.zsh;
source /home/$USERNAME/march/.ros2_foxy/install/local_setup.zsh;
cd /home/$USERNAME/march/ros2;
export CC=gcc;
export CXX=g++;
colcon build --symlink-install --cmake-force-configure
EOF
sudo chmod 755 $ROS2_LOCATION/usr/bin/march_build_ros2

sudo tee <<EOF $ROS2_LOCATION/usr/bin/march_run_ros2 >/dev/null
#!/usr/bin/env zsh
source /opt/ros/foxy/local_setup.zsh;
source /home/$USERNAME/march/.ros2_foxy/install/local_setup.zsh;
cd /home/$USERNAME/march/ros2;
source install/local_setup.zsh;
ros2 launch march_launch march_ros2_simulation.launch.py
EOF
sudo chmod 755 $ROS2_LOCATION/usr/bin/march_run_ros2
