BASE_IMAGE="ros:foxy-ros1-bridge-focal"
BASE_PACKAGES="ros-noetic-ros-base ros-foxy-ros-base"
BUILD_TOOLS="python3-rosdep python3-rosinstall python3-rosinstall-generator build-essential python3-colcon-common-extensions python3-pip python3-pip python3-catkin-pkg python3-catkin-lint clang-tidy clang libssl-dev wget cmake git git-lfs libbullet-dev python3-flake8 python3-pytest-cov python3-setuptools python3-vcstool"
BUILD_TOOLS_NO_REC="libasio-dev libtinyxml2-dev libcunit1-dev"
PYTHON_TOOLS="mock argcomplete pytest-repeat pytest-rerunfailures pytest flakehell"

PYTHON_REQUIREMENTS=$(cat requirements.pip | tr '\n' ' ')
FLAKEHELL_REQUIREMENTS=$(python3 -m flakehell missed | tr '\n' ' ')
ROS_PACKAGES=$(./.scripts/package_list.sh | tr '\n' ' ')

echo "# This file has been automatically generated by .scripts/generate_dockerfile.sh"
echo "# You should NOT change this file manually!"
echo "# If you have added a new dependency, running .scripts/generate_dockerfile.sh from"
echo "# the root of the repository should be enough."
echo "FROM $BASE_IMAGE"
echo "ARG DEBIAN_FRONTEND=noninteractive"
echo "RUN apt update && apt upgrade -y && apt install -y apt-utils && apt install -y $BASE_PACKAGES"
echo "RUN apt update && apt install -y $BUILD_TOOLS && apt install -y --no-install-recommends $BUILD_TOOLS_NO_REC"
echo "RUN python3 -m pip install $PYTHON_TOOLS $PYTHON_REQUIREMENTS $FLAKEHELL_REQUIREMENTS"
echo "RUN bash -c \"source /opt/ros/foxy/local_setup.bash && rosdep update && apt update && apt install -y $ROS_PACKAGES\""
