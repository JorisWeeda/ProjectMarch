# Ensure that X11 can be accessed from the chroot environment
xhost +local

# Start ROS 1, ROS 2 and the bridge
setsid gnome-terminal -- schroot -d "/home/$USERNAME" -c ros -- zsh -c "march_run_ros1"
setsid gnome-terminal -- schroot -d "/home/$USERNAME" -c ros -- zsh -c "march_run_ros2"
setsid gnome-terminal -- schroot -d "/home/$USERNAME" -c ros -- zsh -c "march_run_bridge"
