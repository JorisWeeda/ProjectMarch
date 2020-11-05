# Ensure that X11 can be accessed from the chroot environment
xhost +local

# Start ROS 1, ROS 2 and the bridge
setsid gnome-terminal -- schroot -d "/home/$USERNAME" -c ros -- zsh -c "export DISPLAY=:0 && march_run_ros1"
sleep 2
setsid gnome-terminal -- schroot -d "/home/$USERNAME" -c ros -- zsh -c "export DISPLAY=:0 && march_run_bridge"
sleep 2
setsid gnome-terminal -- schroot -d "/home/$USERNAME" -c ros -- zsh -c "export DISPLAY=:0 && march_run_ros2"
