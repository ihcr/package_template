# package template

## Overview

This is a template: replace, remove, and add where required. Describe here what 
this package does and what it's meant for in a few sentences.

### License

The source code is released under a [BSD 3-Clause license](package_template/LICENSE)


## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-noetic-...
    
Or better, use `rosdep`:

	sudo rosdep install --from-paths src

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen] (linear algebra library)

#### Building

To build from source, clone the latest version from this repository into your 
catkin workspace and compile the package using

	cd catkin_ws/src
    cd ..
    catkin init
    cd catkin_ws/src
	git clone https://github.com/ihcr/package_template.git
	cd ..
	catkin build
    source devel/setup.bash
    roslaunch package_template package_template.launch

### Unit Tests

Run the unit tests with

	catkin test package_template