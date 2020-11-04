#!/usr/bin/env bash

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
⠀⠀⠀⡴⢃⣴⣿⡿⠛⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⢼⣿⣿⣿⣦⣄
⠀⢀⢊⣴⣿⠟⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠙⠛⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⣴⣿⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠘⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠹⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
\033[0;0m
"

##############################
# Project March setup script #
##############################
#
# This script aims to create a reproducible and uniform build environment for
# everybody that is working on the exoskeleton.
# This is done by creating a "chroot" environment that has an Ubuntu 18.04 installation
# in it. In this environment, ROS 1 Melodic is installed from the repositories and
# ROS 2 Foxy is installed from source. This enables the execution of ROS 1 and ROS 2
# in a single environment, with the help of a bridge that passes messages between the
# two.
# This script automatically:
#   - installs schroot and debootstrap for the chroot environment
#   - configures the schroot profiles for the chroot environment
#   - installs ROS 1
#   - installs the dependencies for the March ROS 1 packages
#   - builds the March ROS 1 packages
#   - builds ROS 2
#   - builds the dependencies for the March ROS 2 packages
#   - builds the March ROS 2 packages
#   - builds the bridge between ROS 1 and ROS 2


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

function sudo_schroot_zsh ()
{
    sudo schroot -d "/home/$USERNAME" -c ros -- zsh -c "$1"
    check_error
}

function schroot_zsh ()
{
    schroot -d "/home/$USERNAME" -c ros -- zsh -c "$1"
    check_error
}

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
    print_error "This script is intended to run on Ubuntu machines, because it relies on the apt package manager. This is $(lsb_release -is). Are you sure you want to continue? (y/N)"
    read SURE_CONTINUE
    if [ $SURE_CONTINUE != "y" ]
    then
        print_error "[MARCH] Installation cancelled!"
        exit 1
    fi
fi

# Install debootstrap and schroot. If you want to run this on a machine on non-apt
# package manager systems, you will probably have to change these lines.
print_info "Installing required packages 'debootstrap' and 'schroot'..."
print_info "Requesting root permissions..."
sudo apt update
check_error
sudo apt install -y debootstrap schroot
check_error

################################
# CREATION OF SCHROOT PROFILES #
################################

print_info "Creating schroot profile..."
# Create a chroot for ROS
ROS_LOCATION="/srv/chroot/ros"
sudo mkdir -p $ROS_LOCATION/opt/ros/melodic
check_error
sudo chmod 755 $ROS_LOCATION
check_error

# Set permissions of /opt and child folders
sudo chmod -R 755 $ROS_LOCATION/opt/ros
check_error

# Create a schroot profile for ROS
SCHROOT_ROS="/etc/schroot/ros"
sudo mkdir -p $SCHROOT_ROS
check_error
sudo chmod 755 $SCHROOT_ROS
check_error

# Write the config file that refers to all the files of schroot
sudo tee <<EOF $SCHROOT_ROS/config >/dev/null
# Filesystems to mount inside the chroot.
FSTAB="$SCHROOT_ROS/mount"

# Files to copy from the host system into the chroot.
COPYFILES="$SCHROOT_ROS/copyfiles"

# System databases to copy into the chroot
NSSDATABASES="$SCHROOT_ROS/nssdatabases"
EOF
check_error

# Define which files should be copied over to the chroot jail
sudo tee <<EOF $SCHROOT_ROS/copyfiles >/dev/null
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
sudo touch $SCHROOT_ROS/nssdatabases
check_error

# Create the mount file that defines which folders should be mounted in the chroot jail
sudo tee <<EOF $SCHROOT_ROS/mount >/dev/null
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
EOF
check_error

# Set the file permissions to 744 for all chroot configuration files
sudo chmod 744 $SCHROOT_ROS/*
check_error

###############################
# CREATION OF SCHROOT CONFIGS #
###############################
print_info "Creating schroot configs..."
cd /etc/schroot/chroot.d
check_error

# Add the ROS 1 config
sudo tee <<EOF ros.conf >/dev/null
[ros]
description=Ubuntu 18.04 (Bionic) with ROS Melodic and ROS 2 Foxy from source
type=directory
directory=$ROS_LOCATION
users=$USER
root-groups=root
profile=ros
personality=linux
EOF
check_error

# Set the configs to rw only for root
sudo chmod 600 ros.conf
check_error

##############################################
# DOWNLOADING UBUNTU DISTRIBUTIONS IN CHROOT #
##############################################

# Go to the workspace
cd $WORKSPACE_PATH

# Create symbolic link if focal is not yet recognized
#print_info "Creating focal symlink for debootstrap..."
#sudo ln -s gutsy /usr/share/debootstrap/scripts/focal

print_info "Installing minimal version of Ubuntu Bionic..."
# Download the files for Ubuntu Bionic in the ROS 2 chroot
sudo debootstrap --variant=buildd --arch=amd64 bionic $ROS_LOCATION http://archive.ubuntu.com/ubuntu/
check_error

#####################################
# INSTALLING ROS 1 ON UBUNTU BIONIC #
#####################################

# Define package locations
sudo tee <<EOF $ROS_LOCATION/etc/apt/sources.list >/dev/null
deb http://archive.ubuntu.com/ubuntu bionic main
deb http://archive.ubuntu.com/ubuntu bionic universe
deb http://archive.ubuntu.com/ubuntu bionic restricted
deb http://archive.ubuntu.com/ubuntu bionic multiverse
EOF
check_error

# Configure the home directory of the user
print_info "Creating user in Ubuntu Bionic..."
sudo schroot -d "/home/$USERNAME" -c ros -- bash -c "mkdir -p /home/$USERNAME/march; chown -R $USERNAME:$USERNAME /home/$USERNAME" 
check_error

# Install required packages
print_info "Installing basic packages of Ubuntu Bionic..."
sudo schroot -d "/home/$USERNAME" -c ros -- bash -c "apt update && apt upgrade -y && apt install -y lsb-release sudo curl gnupg2 zsh git locales"
check_error

# Add key from ROS 1 
print_info "Add ROS 1 signing key..."
sudo_schroot_zsh "apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"

# Add key from ROS 2
print_info "Add ROS 2 signing key..."
sudo_schroot_zsh "curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -"

# Set locale to UTF8 supported locale
print_info "Settings locale to en_US.UTF-8..."
sudo_schroot_zsh "locale && sudo locale-gen en_US en_US.UTF-8 && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && export LANG=en_US.UTF-8 && locale"

# Add ROS 1 to package locations
sudo tee <<EOF $ROS_LOCATION/etc/apt/sources.list >/dev/null
deb http://archive.ubuntu.com/ubuntu bionic main
deb http://archive.ubuntu.com/ubuntu bionic universe
deb http://archive.ubuntu.com/ubuntu bionic restricted
deb http://archive.ubuntu.com/ubuntu bionic multiverse
deb http://packages.ros.org/ros/ubuntu bionic main
EOF
check_error

# Install ROS Melodic
print_info "Installing ROS 1 Melodic..."
sudo_schroot_zsh "apt update && apt install -y ros-melodic-desktop-full"

# Add automatic sourcing to the .bashrc file of the user
sudo_schroot_zsh "echo 'source /opt/ros/melodic/setup.zsh' > /home/$USERNAME/.zshrc && chown $USERNAME:$USERNAME /home/$USERNAME/.zshrc && chmod 755 /home/$USERNAME/.zshrc"

# Install dependencies for building ROS 1 packages
print_info "Install ROS 1 building dependencies..."
sudo_schroot_zsh "apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python3-colcon-common-extensions python-pip python3-lark-parser"
sudo_schroot_zsh "rosdep init"

# Install the ROS 1 dependencies
bash -c "$WORKSPACE_PATH/scripts/install_dependencies_ros1.sh"
check_error

print_info "Creating commands..."
bash -c "$WORKSPACE_PATH/scripts/create_commands.sh"
check_error

print_info "Set ROS shell prefix..."
schroot_zsh "echo \"
export DISPLAY=:0;
export precmd_functions='';
export PS1='ROS > ' \" > /home/$USERNAME/.zshrc"

print_info "Build March ROS 1 for the first time..."
schroot_zsh "march_build_ros1"

#####################################
# INSTALLING ROS 2 ON UBUNTU BIONIC #
#####################################

print_info "Install up-to-date cmake version"
sudo_schroot_zsh "apt update && apt install -y build-essential libssl-dev wget"
schroot_zsh "wget https://github.com/Kitware/CMake/archive/v3.18.4.tar.gz && tar -zxvf v3.18.4.tar.gz && cd CMake-3.18.4 && ./bootstrap && make"
sudo_schroot_zsh "cd CMake-3.18.4 && sudo make install"

# Install dependencies for building ROS 2 packages
print_info "Install ROS 2 building dependencies..."
sudo_schroot_zsh "apt update && apt install -y build-essential git libbullet-dev python3-colcon-common-extensions python3-flake8 python3-pip python3-pytest-cov python3-rosdep python3-setuptools python3-vcstool python3-catkin-pkg python3-rosdistro python3-rospkg python3-rosdep-modules && python3 -m pip install -U argcomplete flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest-repeat pytest-rerunfailures pytest && apt install --no-install-recommends -y libasio-dev libtinyxml2-dev libcunit1-dev"

print_info "Install the source files from ROS 2 in order to install the bridge..."
schroot_zsh "mkdir -p /home/$USERNAME/march/.ros2_foxy/src && cd /home/$USERNAME/march/.ros2_foxy && wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos"
function import_ros2_repo
{
    schroot -d "/home/$USERNAME" -c ros -- zsh -c "cd /home/$USERNAME/march/.ros2_foxy && vcs import src < ros2.repos && rm ros2.repos"
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
schroot_zsh "cd /home/$USERNAME/march/.ros2_foxy && rosdep update && rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys \"console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers\""

# Building ROS 2 (takes a long time)
print_info "Building ROS 2... (THIS TAKES A LONG TIME)"
schroot_zsh "cd /home/$USERNAME/march/.ros2_foxy && colcon build --symlink-install --packages-skip ros_bridge"

# Install the ROS 2 dependencies
bash -c "$WORKSPACE_PATH/scripts/install_dependencies_ros2.sh"
check_error

print_info "Build March ROS 2 for the first time..."
schroot_zsh "march_build_ros2"

#######################
# BUILDING THE BRIDGE #
#######################

print_info "Build the ROS 1 bridge for the first time..."
schroot_zsh "march_build_bridge"

################################
# CREATION OF STARTING SCRIPTS #
################################
print_info "Creating startup scripts..."
cd $WORKSPACE_PATH/scripts

# Create the startup script and copy it to start_ros2.sh
tee <<EOF start_ros.sh >/dev/null
xhost +local:
schroot -d "/home/$USERNAME" -c ros -- zsh
EOF
check_error

# Give the scripts execute permissions
chmod +x start_ros.sh
check_error

print_info_bold "Installation succesful!"
exit 0
