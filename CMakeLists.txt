# CMakeLists.txt

# License: BSD 3-Clause License
# Copyright (c) 2021, University of Leeds and Harbin Institute of Technology.
# Copyright note valid unless otherwise stated in individual files.
# All rights reserved.

################################################################################
# Set Up the Project
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(package_template VERSION 1.0.0)

## Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)


################################################################################
# Find Dependencies
################################################################################
## Find system libraries
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
)


################################################################################
# Catkin Specific Configuration
################################################################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS 
    include
    ## This is only necessary because Eigen3 sets a non-standard EIGEN3_INCLUDE_DIR variable
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES 
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS 
    roscpp
    sensor_msgs
  DEPENDS
    ## find_package(Eigen3) provides a non standard EIGEN3_INCLUDE_DIR instead of 
    ## Eigen3_INCLUDE_DIRS. Therefore, the DEPEND does not work as expected and 
    ## we need to add the directory to the INCLUDE_DIRS
    # Eigen3
  
    ## Boost is not part of the DEPENDS since it is only used in source files,
    ## Dependees do not depend on Boost when they depend on this package.
)


################################################################################
# Build the Main Library and Binaries
################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # Set manually because Eigen sets a non standard INCLUDE DIR
  ${EIGEN3_INCLUDE_DIR}
  # Set because Boost is an internal dependency, not transitive.
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_core
  SHARED
    src/${PROJECT_NAME}/Algorithm.cpp
)
target_link_libraries(${PROJECT_NAME}_core
  PUBLIC
    ${catkin_LIBRARIES}
)
## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_core
  ${catkin_EXPORTED_TARGETS}
)

## Declare cpp executables
add_executable(${PROJECT_NAME}
    src/${PROJECT_NAME}/${PROJECT_NAME}_node.cpp
    src/${PROJECT_NAME}/PackageTemplate.cpp
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${PROJECT_NAME}_core
    ${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)


################################################################################
# Install the Main Library and Binaries
################################################################################
# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}_core #${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark C++ header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


################################################################################
# Test
################################################################################
if(${CATKIN_ENABLE_TESTING})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    ## Add gtest based cpp test target and link libraries
    catkin_add_gtest(${PROJECT_NAME}-test
        test/gtest_main.cpp
        test/test_algorithm.cpp
    )
    target_link_libraries(${PROJECT_NAME}-test 
        ${PROJECT_NAME}_core
    )
endif()