#!/usr/bin/env bash

# These functions are often used to print text in a certain color
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

# Check if the exit code is not 0. If it is not, quit the installer and print
# the related exit code.
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
⠀⠀⠀⠀⢀⠞⢁⣼⡻⣿⠿⠀⠀⠀⠀⠀⠀⠀⠀⠰⣸⣿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀  \033[4;91mWorkspace Installer\033[24;34m⠀
⠀⠀⠀⡴⢃⣴⣿⡿⠛⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⢼⣿⣿⣿⣦⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀      
⠀⢀⢊⣴⣿⠟⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠙⠛⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⣴⣿⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠘⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠹⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
\033[0;0m
"

# User configuation
USERNAME=$(whoami)
WORKSPACE_PATH="$(dirname "$(readlink -f "$0")")"
cd "$WORKSPACE_PATH"
cd ..
WORKSPACE_PATH="$(pwd)"

# Quit if script is executed as root
if [ $(id -u) -eq 0 ]
then
    print_error "This script should NOT run as root."
    exit 1
fi

if [ "$(lsb_release -si)" != "Ubuntu" ]
then
    print_error "This script is intended to run on Ubuntu machines. This is $(lsb_release -is). Are you sure you want to continue? (y/N)"
    read SURE_CONTINUE
    if [ $SURE_CONTINUE != "y" ]
    then
        print_error "[MARCH] Installation cancelled!"
        exit 1
    fi
fi

# Install debootstrap and schroot
print_info "Installing required packages 'debootstrap' and 'schroot'..."
print_info "Requesting root permissions..."
sudo apt update
check_error
sudo apt install -y debootstrap schroot
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
sudo chmod 755 $ROS1_LOCATION
check_error
ROS2_LOCATION="/srv/chroot/ros2"
sudo mkdir -p $ROS2_LOCATION/opt/ros/foxy
check_error
sudo chmod 755 $ROS2_LOCATION
check_error

# Set permissions of /opt and child folders
sudo chmod -R 755 $ROS1_LOCATION/opt/ros
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
/etc/passwd
/etc/shadow
/etc/group
/etc/sudoers
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

##############################################
# DOWNLOADING UBUNTU DISTRIBUTIONS IN CHROOT #
##############################################

# Go to the workspace
cd $WORKSPACE_PATH

# Create symbolic link if focal is not yet recognized
print_info "Creating focal symlink for debootstrap..."
sudo ln -s gutsy /usr/share/debootstrap/scripts/focal

print_info "Installing minimal version of Ubuntu Bionic..."
# Download the files for Ubuntu Bionic in the ROS 1 chroot
sudo debootstrap --variant=buildd --arch=amd64 bionic $ROS1_LOCATION http://archive.ubuntu.com/ubuntu/
check_error

print_info "Installing minimal version of Ubuntu Focal..."
# Download the files for Ubuntu Focal in the ROS 2 chroot
sudo debootstrap --variant=buildd --arch=amd64 focal $ROS2_LOCATION http://archive.ubuntu.com/ubuntu/
check_error

#####################################
# INSTALLING ROS 1 ON UBUNTU BIONIC #
#####################################

# Define package locations
sudo tee <<EOF $ROS1_LOCATION/etc/apt/sources.list >/dev/null
deb http://archive.ubuntu.com/ubuntu bionic main
deb http://archive.ubuntu.com/ubuntu bionic universe
deb http://archive.ubuntu.com/ubuntu bionic restricted
deb http://archive.ubuntu.com/ubuntu bionic multiverse
EOF
check_error

# Configure the home directory of the user
print_info "Creating user in Ubuntu Bionic..."
sudo schroot -d "/home/$USERNAME" -c ros1 -- bash -c "mkdir -p /home/$USERNAME/march; chown -R $USERNAME:$USERNAME /home/$USERNAME" 
check_error

# Install required packages
print_info "Installing basic packages of Ubuntu Bionic..."
sudo schroot -d "/home/$USERNAME" -c ros1 -- bash -c "apt update && apt upgrade -y && apt install -y lsb-release sudo curl gpg zsh git"
check_error

# Add key from ROS 1 
print_info "Add ROS 1 signing key..."
sudo schroot -d "/home/$USERNAME" -c ros1 -- zsh -c "apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"
check_error

# Add ROS 1 to package locations
sudo tee <<EOF $ROS1_LOCATION/etc/apt/sources.list >/dev/null
deb http://archive.ubuntu.com/ubuntu bionic main
deb http://archive.ubuntu.com/ubuntu bionic universe
deb http://archive.ubuntu.com/ubuntu bionic restricted
deb http://archive.ubuntu.com/ubuntu bionic multiverse
deb http://packages.ros.org/ros/ubuntu bionic main
EOF
check_error

# Install ROS Melodic
print_info "Installing ROS 1 Melodic..."
sudo schroot -d "/home/$USERNAME" -c ros1 -- zsh -c "apt update && apt install -y ros-melodic-desktop-full"
check_error

# Add automatic sourcing to the .bashrc file of the user
sudo schroot -d "/home/$USERNAME" -c ros1 -- zsh -c "echo 'source /opt/ros/melodic/setup.zsh' > /home/$USERNAME/.zshrc && chown $USERNAME:$USERNAME /home/$USERNAME/.zshrc && chmod 755 /home/$USERNAME/.zshrc"
check_error

# Install dependencies for building ROS 1 packages
print_info "Install ROS 1 building dependencies..."
sudo schroot -d "/home/$USERNAME" -c ros1 -- zsh -c "apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python3-colcon-common-extensions python-pip python3-lark-parser"
check_error
sudo schroot -d "/home/$USERNAME" -c ros1 -- zsh -c "rosdep init"

# Install March specific ROS 1 dependencies
print_info "Update ROS dependencies list..."
schroot -d "/home/$USERNAME" -c ros1 -- zsh -c "rosdep update"
check_error

print_info "Install March specific ROS 1 dependencies..."
sudo schroot -d "/home/$USERNAME" -c ros1 -- zsh -c "sudo chmod -R 755 /opt"
schroot -d "/home/$USERNAME" -c ros1 -- zsh -c "source /opt/ros/melodic/setup.zsh; rosdep install -y --from-paths /home/$USERNAME/march/ros1/src --ignore-src"
check_error

print_info "Creating commands..."
bash -c "$WORKSPACE_PATH/scripts/.create_commands.sh"
check_error

print_info "Set ROS 1 shell prefix..."
schroot -d "/home/$USERNAME" -c ros1 -- zsh -c "echo \"
export DISPLAY=:0;
export precmd_functions='';
export PS1='ROS 1> ' \" > /home/$USERNAME/.zshrc"
check_error

# Update compiler
print_info "Updating compiler to GCC 9..."
sudo schroot -d "/home/$USERNAME" -c ros1 -- zsh -c "
apt update && \
apt install build-essential software-properties-common -y && \
add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
apt update && \
apt install gcc-snapshot -y && \
apt update && \
apt upgrade -y && \
apt install gcc-9 g++-9 -y && \
update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-9 && \
update-alternatives --config gcc"

print_info "Build March ROS 1 for the first time..."
schroot -d "/home/$USERNAME" -c ros1 -- zsh -c "march_build_ros1"
check_error

#####################################
# INSTALLING ROS 2 ON UBUNTU FOCAL #
#####################################

# Define package locations
sudo tee <<EOF $ROS2_LOCATION/etc/apt/sources.list >/dev/null
deb http://archive.ubuntu.com/ubuntu focal main
deb http://archive.ubuntu.com/ubuntu focal universe
deb http://archive.ubuntu.com/ubuntu focal restricted
deb http://archive.ubuntu.com/ubuntu focal multiverse
EOF
check_error

# Configure the home directory of the user
print_info "Creating user in Ubuntu Focal..."
sudo schroot -d "/home/$USERNAME" -c ros2 -- bash -c "mkdir -p /home/$USERNAME/march; chown -R $USERNAME:$USERNAME /home/$USERNAME"
check_error

# Install required packages
print_info "Installing basic packages of Ubuntu Focal..."
sudo schroot -d "/home/$USERNAME" -c ros2 -- bash -c "apt update && apt upgrade -y && apt install -y lsb-release sudo curl gnupg zsh python3-pip locales && pip3 install -U argcomplete"
check_error

# Set locale to UTF8 supported locale
print_info "Settings locale to en_US.UTF-8..."
sudo schroot -d "/home/$USERNAME" -c ros2 -- bash -c "locale && sudo locale-gen en_US en_US.UTF-8 && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && export LANG=en_US.UTF-8 && locale"
check_error

# Add key from ROS 2
print_info "Add ROS 2 signing key..."
sudo schroot -d "/home/$USERNAME" -c ros2 -- zsh -c "curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -"
check_error

# Add ROS 2 to package locations
sudo tee <<EOF $ROS2_LOCATION/etc/apt/sources.list >/dev/null
deb http://archive.ubuntu.com/ubuntu focal main
deb http://archive.ubuntu.com/ubuntu focal universe
deb http://archive.ubuntu.com/ubuntu focal restricted
deb http://archive.ubuntu.com/ubuntu focal multiverse
deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu focal main
EOF
check_error

# Install dependencies for building ROS 2 packages
print_info "Install ROS 2 building dependencies..."
sudo schroot -d "/home/$USERNAME" -c ros2 -- zsh -c "sudo chmod -R 755 /opt"
sudo schroot -d "/home/$USERNAME" -c ros2 -- zsh -c "apt update && apt install -y ros-foxy-rcl-logging-spdlog libboost-all-dev build-essential cmake git libbullet-dev python3-colcon-common-extensions python3-flake8 python3-pip python3-pytest-cov python3-rosdep python3-setuptools python3-vcstool python-yaml wget && python3 -m pip install -U argcomplete flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest-repeat pytest-rerunfailures pytest && apt install --no-install-recommends -y libasio-dev libtinyxml2-dev libcunit1-dev"
check_error

print_info "Install the source files from ROS 2 in order to install the bridge..."
schroot -d "/home/$USERNAME" -c ros2 -- zsh -c "mkdir -p /home/$USERNAME/march/.ros2_foxy/src && cd /home/$USERNAME/march/.ros2_foxy && wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos"
check_error
function import_ros2_repo
{
    schroot -d "/home/$USERNAME" -c ros2 -- zsh -c "cd /home/$USERNAME/march/.ros2_foxy && vcs import src < ros2.repos && rm ros2.repos"
    ERROR_CODE=$?
    if [ $ERROR_CODE -ne 0 ]
    then
        print_error "[MARCH] Downloading ROS 2 repos failed! Retry! Exit code: $ERROR_CODE"
        import_ros2_repo
    fi
}
import_ros2_repo

# Install ROS dependencies that are necessary for building ROS 2
print_info "Install ROS 2 source dependencies..."
sudo schroot -d "/home/$USERNAME" -c ros2 -- zsh -c "rosdep init"
schroot -d "/home/$USERNAME" -c ros2 -- zsh -c "cd /home/$USERNAME/march/.ros2_foxy && rosdep update && rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys \"console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers\""
check_error

print_info "Adding Python symlink to Python 3"
sudo schroot -d "/home/$USERNAME" -c ros2 -- zsh -c "ln -s python2 /usr/bin/python"

# Update compiler
print_info "Updating compiler to GCC 9..."
schroot -d "/home/$USERNAME" -c ros2 -- zsh -c "
sudo apt update && \
sudo apt install build-essential software-properties-common -y && \
sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
sudo apt update && \
sudo apt install gcc-snapshot -y && \
sudo apt update && \
sudo apt upgrade -y && \
sudo apt install gcc-9 g++-9 -y && \
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-9 && \
sudo update-alternatives --config gcc"

# Building ROS 2 (takes a long time)
print_info "Building ROS 2... (THIS TAKES A LONG TIME)"
schroot -d "/home/$USERNAME" -c ros2 -- zsh -c "cd /home/$USERNAME/march/.ros2_foxy && colcon build --symlink-install --packages-skip ros1_bridge"

# Install March specific ROS 2 dependencies
print_info "Install March specific ROS 2 dependencies..."
schroot -d "/home/$USERNAME" -c ros2 -- zsh -c "source /home/$USERNAME/march/.ros2_foxy/install/setup.zsh && rosdep install -y --from-paths /home/$USERNAME/march/ros2/src --ignore-src"
check_error

# Add the build and run commands of ROS 2
print_info "Set ROS 2 shell prefix..."
schroot -d "/home/$USERNAME" -c ros2 -- zsh -c "echo \"
export DISPLAY=:0;
export precmd_functions='';
export PS1='ROS 2> ';
\" > /home/$USERNAME/.zshrc"
check_error

print_info "Build March ROS 2 for the first time..."
schroot -d "/home/$USERNAME" -c ros2 -- zsh -c "march_build_ros2"
check_error

#######################
# BUILDING THE BRIDGE #
#######################

print_info "Build the ROS 1 bridge for the first time..."
sudo schroot -d "/home/$USERNAME" -c ros1 -- zsh -c "apt update && apt full-upgrade -y"
schroot -d "/home/$USERNAME" -c ros1 -- zsh -c "march_build_bridge"
check_error

################################
# CREATION OF STARTING SCRIPTS #
################################
print_info "Creating startup scripts..."
cd $WORKSPACE_PATH/scripts

# Create the startup script and copy it to start_ros2.sh
tee <<EOF start_ros1.sh >/dev/null
xhost +local:
schroot -d "/home/$USERNAME" -c ros1 -- zsh
EOF
check_error

# Give the scripts execute permissions
chmod +x start_ros1.sh
check_error
cp start_ros1.sh start_ros2.sh
check_error
sed -i 's/ros1/ros2/g' start_ros2.sh
check_error

print_info_bold "Installation succesful!"
exit 0
