#!/usr/bin/env bash

function print_error ()
{
    echo -e "\033[1;91m$1\033[0;0m"
}

function print_info ()
{
    echo -e "\033[0;93m[MARCH] $1\033[0;0m"
}

function print_info_bold ()
{
    echo -e "\033[1;32m[MARCH] $1\033[0;0m"
}

function check_error ()
{
    ERROR_CODE=$?
    if [ $ERROR_CODE -ne 0 ]
    then
        print_error "[MARCH] Installation failed! Exit code: $ERROR_CODE"
        exit 1
    fi
}

echo -e "\033[1;34m

⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⠏⠉⠉⠙⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⠀⠀⠀⠀⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣶⣆⣀⣜⡦⣀⡠⠊⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⡛⠯⠑⠈⠙⣷⠀⠀⠀⠀⠀⢀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⢠⠴⠒⠋⠁⠀⠀⠀⠀⡆⣿⡀⠀⠀⢀⠔⠁⠀⠀⠀⠀⠀⠀⣴⠶⠶⠦⣤⡀⠀⣴⠶⠶⠶⢤⡀⠀⠀⢀⣤⠴⠶⠶⢤⡀⠀⠀⠀⠀⢰⡆⠀⢰⠶⠶⠶⠶⠆⠀⠀⣠⡴⠶⠶⢦⣄⠰⠶⠶⡶⠶⠶
⠀⠀⠀⠀⠀⠀⠀⠀⡼⠀⢠⡤⡄⠀⠀⠀⢰⢧⡏⢣⠀⡰⠋⢠⠂⠀⠀⠀⠀⠀⠀⣿⠀⠀⠀⠈⣧⠀⣿⠀⠀⠀⠀⢻⠀⢠⡟⠁⠀⠀⠀⠀⠙⣆⠀⠀⠀⢸⡇⠀⢸⠀⠀⠀⠀⠀⠀⡾⠁⠀⠀⠀⠀⠈⠀⠀⠀⡇⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⡏⣿⣷⠀⠀⠀⠸⣸⠇⠈⠟⠀⡰⠁⠀⠀⠀⠀⠀⠀⠀⣿⣀⣀⣀⡴⠇⠀⣿⣀⣀⣀⡴⠏⠀⢸⠀⠀⠀⠀⠀⠀⠀⣿⠀⠀⠀⢸⡇⠀⢸⠶⠶⠶⠶⠀⢸⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⡇⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⡇⢰⠃⣿⣿⠀⠀⠀⣇⡿⠀⠀⢀⡜⠁⠀⠀⠀⠀⠀⠀⠀⠀⣿⠉⠉⠉⠀⠀⠀⣿⠀⠀⠘⢧⠀⠀⠘⣧⠀⠀⠀⠀⠀⣠⠏⠀⠀⠀⢸⡇⠀⢸⠀⠀⠀⠀⠀⠀⢳⡄⠀⠀⠀⠀⢀⠀⠀⠀⡇⠀⠀
⠀⠀⠀⠀⠀⠀⠀⢠⠀⡘⠀⣿⣿⣶⣶⣤⣼⣇⣱⠤⠞⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠿⠀⠀⠀⠀⠀⠀⠻⠀⠀⠀⠈⠷⠀⠀⠈⠙⠶⠶⠶⠚⠁⠀⠙⠶⠶⠛⠀⠀⠸⠶⠶⠶⠶⠆⠀⠀⠙⠳⠶⠶⠖⠋⠀⠀⠀⠇⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠘⠀⠃⠀⣿⣿⣿⢫⣵⣮⣭⣽⣧⣤⣤⣄⣀⣀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡄⠀⠀⠀⠀⢠⡀⠀⠀⠀⠀⢀⡀⠀⠀⠀⠀⣀⣀⣀⣀⣀⠀⠀⠀⠀⢀⣀⣤⣀⡀⠀⢀⣀⡀⠀⠀⠀⣀⣀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⠿⠿⣯⢿⣿⣿⣿⣽⠿⠟⠛⠿⢿⣿⣿⣷⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⡄⠀⠀⢠⣿⡇⠀⠀⠀⢀⣾⣿⡀⠀⠀⠀⣿⡟⠛⠛⢿⣷⠀⢠⣾⡿⠟⠛⠻⠿⠃⢸⣿⡇⠀⠀⠀⣿⣿⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⡇⠀⣸⣶⣤⠉⠉⠁⠒⠒⠒⢂⡕⢺⣟⡟⠋⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⡄⢠⣿⢿⣷⠀⠀⢀⣾⡿⢻⣷⡀⠀⠀⣿⡇⢤⣤⣾⡿⠀⣾⣿⠁⠀⠀⠀⠀⠀⢸⣿⣷⣶⣶⣦⣿⣿⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⡼⠀⣰⣿⡿⠃⠀⠀⠀⠀⠀⠀⡞⢀⣿⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⢰⣿⡇⢻⣷⣾⡏⢸⣿⡄⢀⣾⣿⠥⠤⢿⣷⠀⠀⣿⡇⠘⣿⣯⠀⠀⢻⣿⣆⠀⠀⠀⣀⠀⢸⣿⡏⠉⠉⠉⣿⣿⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⢰⠃⣰⣿⡟⠁⠀⠀⠀⠀⠀⠀⢰⢁⣾⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⠀⠀⢻⡟⠀⠈⣿⡇⣼⡿⠁⠀⠀⠈⣿⣧⠀⣿⡇⠀⠘⣿⣧⠀⠀⠙⢿⣿⣶⣿⠿⠃⢸⣿⡇⠀⠀⠀⣿⣿⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⢀⡠⣠⣶⣝⠏⠀⠀⠀⠀⠀⠀⠀⠀⡎⣼⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⢀⠞⢁⣼⡻⣿⠿⠀⠀⠀⠀⠀⠀⠀⠀⠰⣸⣿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀  \033[4;91mWorkspace Installer\033[24;34m⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⡴⢃⣴⣿⡿⠛⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⢼⣿⣿⣿⣦⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⢀⢊⣴⣿⠟⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠙⠛⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⣴⣿⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠘⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠹⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
\033[0;0m
"

# User configuation
USERNAME=$(whoami)
WORKSPACE_PATH=$(dirname "$(readlink -f "$0")")

if [ "$(lsb_release -si)" != "Ubuntu" ]
then
    print_error "This script is intended to run on Ubuntu machines. This is $(lsb_release -is). Are you sure you want to continue? (y/N)"
    read SURE_CONTINUE
    if [ $SURE_CONTINUE != "y" ]
    then
        print_error "[MARCH] Installation failed!"
        exit 1
    fi
fi

# Install debootstrap and schroot
print_info "Installing required packages 'debootstrap' and 'schroot'..."
print_info "Requesting root permissions..."
#sudo apt update
check_error
#sudo apt install -y debootstrap schroot
check_error

################################
# CREATION OF SCHROOT PROFILES #
################################

print_info "Creating schroot profiles..."
# Create ROS 1 and ROS 2 chroot and installation folders that will be shared across
# both ROS chroots
ROS1_LOCATION="/srv/chroot/ros1"
sudo mkdir -p $ROS1_LOCATION/opt/ros/melodic
check_error
sudo chmod -R 755 $ROS1_LOCATION
check_error
ROS2_LOCATION="/srv/chroot/ros2"
sudo mkdir -p $ROS2_LOCATION/opt/ros/foxy
check_error
sudo chmod -R 755 $ROS2_LOCATION
check_error

# Create a schroot profile for ROS 1
SCHROOT_ROS1="/etc/schroot/ros1"
sudo mkdir -p $SCHROOT_ROS1
check_error
sudo chmod 755 $SCHROOT_ROS1
check_error

# Write the config file that refers to all the files of schroot
sudo tee <<EOF $SCHROOT_ROS1/config >/dev/null
# Filesystems to mount inside the chroot.
FSTAB="$SCHROOT_ROS1/mount"

# Files to copy from the host system into the chroot.
COPYFILES="$SCHROOT_ROS1/copyfiles"

# System databases to copy into the chroot
NSSDATABASES="$SCHROOT_ROS1/nssdatabases"
EOF
check_error

# Define which files should be copied over to the chroot jail
sudo tee <<EOF $SCHROOT_ROS1/copyfiles >/dev/null
/etc/hosts
/etc/resolv.conf
/etc/localtime
/etc/locale.gen
EOF

# Create an empty nssdatabases file
sudo touch $SCHROOT_ROS1/nssdatabases
check_error

# Create the mount file that defines which folders should be mounted in the chroot jail
sudo tee <<EOF $SCHROOT_ROS1/mount >/dev/null
# mount.defaults: static file system information for chroots.
# Note that the mount point will be prefixed by the chroot path
# (CHROOT_PATH)
#
# <file system>	<mount point>	<type>	<options>	<dump>	<pass>
proc		/proc		proc	defaults	0	0
/dev		/dev		none	rw,bind		0	0
/dev/pts	/dev/pts	none	rw,bind		0	0
tmpfs		/dev/shm	tmpfs	defaults	0	0
/sys		/sys		none	rw,bind		0	0
/tmp		/tmp		none	rw,bind		0	0
$WORKSPACE_PATH	/home/$USERNAME/march	none	rw,bind	0	0
$ROS2_LOCATION/opt/ros/foxy	/opt/ros/foxy	none	rw,bind	0	0
EOF
check_error

# Set the file permissions to 744 for all chroot configuration files
sudo chmod 744 $SCHROOT_ROS1/*
check_error

# Create ROS 2 schroot profile by copying the files from ROS 1 and changing some values with sed
SCHROOT_ROS2="/etc/schroot/ros2"
sudo mkdir -p $SCHROOT_ROS2
check_error
sudo chmod 755 $SCHROOT_ROS2
check_error
sudo cp $SCHROOT_ROS1/* $SCHROOT_ROS2
check_error
sudo sed -i 's/ros1/ros2/g' $SCHROOT_ROS2/config
check_error
sudo sed -i 's/ros2/ros1/g' $SCHROOT_ROS2/mount
check_error
sudo sed -i 's/foxy/melodic/g' $SCHROOT_ROS2/mount
check_error

###############################
# CREATION OF SCHROOT CONFIGS #
###############################
print_info "Creating schroot configs..."
cd /etc/schroot/chroot.d
check_error

# Add the ROS 1 config
sudo tee <<EOF ros1.conf >/dev/null
[ros1]
description=Ubuntu 18.04 (Bionic) with ROS Melodic
type=directory
directory=$ROS1_LOCATION
users=$USER
root-groups=root
profile=ros1
personality=linux
EOF
check_error

# Add the ROS 2 config
sudo tee <<EOF ros2.conf >/dev/null
[ros2]
description=Ubuntu 20.04 (Focal) with ROS 2 Foxy
type=directory
directory=$ROS2_LOCATION
users=$USER
root-groups=root
profile=ros2
personality=linux
EOF
check_error

# Set the configs to rw only for root
sudo chmod 600 ros1.conf
check_error
sudo chmod 600 ros2.conf
check_error

################################
# CREATION OF STARTING SCRIPTS #
################################
print_info "Creating startup scripts..."
cd $WORKSPACE_PATH

# Create the startup script and copy it to start_ros2.sh
tee <<EOF /tmp/.start_ros1.sh >/dev/null
xhost +local:
schroot --automatic-session -c ros1
EOF
check_error

chmod +x /tmp/.start_ros1.sh
check_error
cp /tmp/.start_ros1.sh /tmp/.start_ros2.sh
check_error
sed -i 's/ros1/ros2/g' /tmp/.start_ros2.sh
check_error

##############################################
# DOWNLOADING UBUNTU DISTRIBUTIONS IN CHROOT #
##############################################

# Download the files for Ubuntu Bionic in the ROS 1 chroot
#sudo debootstrap --variant=buildd --arch=amd64 bionic $ROS1_LOCATION http://archive.ubuntu.com/ubuntu/
check_error
# Download the files for Ubuntu Focal in the ROS 2 chroot
#sudo debootstrap --variant=buildd --arch=amd64 focal $ROS2_LOCATION http://archive.ubuntu.com/ubuntu/
check_error

############################
# INSTALLING UBUNTU BIONIC #
############################

# Define package locations
sudo tee <<EOF $ROS1_LOCATION/etc/apt/sources.list >/dev/null
deb http://archive.ubuntu.com/ubuntu bionic main
deb http://archive.ubuntu.com/ubuntu bionic universe
deb http://archive.ubuntu.com/ubuntu bionic restricted
deb http://archive.ubuntu.com/ubuntu bionic multiverse
deb http://packages.ros.org/ros/ubuntu bionic main
EOF
check_error

# Add a non-root user with sudo privilege
sudo schroot --automatic-session -c ros1 -- bash -c "if id -u $USERNAME; then useradd $USERNAME && usermod -a -G sudo; fi; chown -R $USERNAME:$USERNAME /home/$USERNAME"
check_error

# Install required packages
sudo schroot --automatic-session -c ros1 -- bash -c "apt update && apt upgrade -y && apt install -y lsb-release sudo curl gpg"
check_error

# Add key from ROS 1 and install ROS melodic
sudo schroot --automatic-session -c ros1 -- bash -c "apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && apt update && apt install ros-melodic-desktop-full"
check_error

# Add automatic sourcing to the .bashrc file of the user
sudo schroot --automatic-session -c ros1 -- bash -c "echo 'source /opt/ros/melodic/setup.bash' > /home/$USERNAME/.bashrc"
check_error

# Install dependencies for building ROS 1 packages
sudo schroot --automatic-session -c ros1 -- bash -c "apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python3-colcon-common-extensions"
check_error
sudo schroot --automatic-session -c ros1 -- bash -c "rosdep init"

# Install March specific ROS 1 dependencies
schroot --automatic-session -c ros1 -- bash -c "rosdep update"
check_error
schroot --automatic-session -c ros1 -- bash -c "rosdep install -y --from-paths /home/$USERNAME/march/ros1/src --ignore-src"
check_error

# Setup has been succesful, unhide the startup scripts
mv /tmp/.start_ros1.sh start_ros1.sh
check_error
mv /tmp/.start_ros2.sh start_ros2.sh
check_error

print_info_bold "Installation succesful!"
exit 0
