# Ensure that X11 can be accessed from the chroot environment
xhost +local;

setsid schroot -d "/home/$USERNAME" -c ros1 -- zsh -c "march_run_ros1"
setsid schroot -d "/home/$USERNAME" -c ros2 -- zsh -c "march_run_ros2"
setsid schroot -d "/home/$USERNAME" -c ros1 -- zsh -c "march_run_bridge"
