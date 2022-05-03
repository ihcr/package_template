/* ----------------------------------------------------------------------------
 * Copyright (c) 2021, University of Leeds and Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   PackageTemplate.hpp
 *  @author Jun Li (junlileeds@gmail.com)
 *  @brief  Header file for PackageTemplate class 
 *  @date   May 02, 2022
 **/

#pragma once

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>

#include "package_template/Algorithm.hpp"

namespace package_template {

/**
 * @brief Main class for the node to handle the ROS interfacing.
 */
class PackageTemplate
{
public:

    /**
     * @brief Construct a new PackageTemplate object
     * 
     * @param node_handle the ROS node handle.
     */
    PackageTemplate(ros::NodeHandle& node_handle);

    /**
     * @brief Destroy the PackageTemplate object
     */
    virtual ~PackageTemplate();

private:
    /**
     * @brief Reads and verifies the ROS parameters.
     * 
     * @return true if successful.
     */
    bool readParameters();

    /**
     * @brief ROS topic callback method.
     * 
     * @param message the received message.
     */
    void topicCallback(const sensor_msgs::Temperature& message);

    /**
     * @brief ROS service server callback.
     * 
     * @param request the request of the service.
     * @param response the provided response.
     * @return true if successful, false otherwise.
     */
    bool serviceCallback(std_srvs::Trigger::Request& request, 
                         std_srvs::Trigger::Response& response);

    // ROS node handle.
    ros::NodeHandle& node_handle_;

    // ROS topic subscriber.
    ros::Subscriber subscriber_;

    // ROS topic name to subscribe to.
    std::string subscriber_topic_;

    // ROS service server.
    ros::ServiceServer service_server_;

    // Algorithm computation object.
    Algorithm algorithm_;
};

}  // end package_template namespace