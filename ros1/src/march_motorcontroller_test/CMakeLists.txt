cmake_minimum_required(VERSION 3.0.2)
project(march_motorcontroller_test)

find_package(catkin REQUIRED COMPONENTS
    march_shared_classes
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
    march_shared_classes
)

catkin_install_python(PROGRAMS scripts/generate_subgait
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
