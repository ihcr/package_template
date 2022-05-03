/* ----------------------------------------------------------------------------
 * Copyright (c) 2021, University of Leeds and Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   package_template_node.hpp
 *  @author Jun Li (junlileeds@gmail.com)
 *  @brief  Source file for package template node
 *  @date   May 02, 2022
 **/

// ROS
#include <ros/ros.h>

#include "package_template/PackageTemplate.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "package_template");
    ros::NodeHandle node_handle("~");

    package_template::PackageTemplate package_template(node_handle);

    ros::spin();
    return 0;
}
