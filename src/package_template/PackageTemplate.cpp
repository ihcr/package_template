/* ----------------------------------------------------------------------------
 * Copyright (c) 2021, University of Leeds and Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   PackageTemplate.cpp
 *  @author Jun Li (junlileeds@gmail.com)
 *  @brief  Source file for PackageTemplate class 
 *  @date   May 02, 2022
 **/

#include "package_template/PackageTemplate.hpp"

namespace package_template {

PackageTemplate::PackageTemplate(ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
    if (!readParameters()) {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }
    subscriber_ = node_handle_.subscribe(subscriber_topic_, 1,
                                         &PackageTemplate::topicCallback, this);
    service_server_ = node_handle_.advertiseService("get_average", 
                                                    &PackageTemplate::serviceCallback, this);
    ROS_INFO("Successfully launched node.");
}

PackageTemplate::~PackageTemplate()
{}

bool PackageTemplate::readParameters()
{
    if (!node_handle_.getParam("subscriber_topic", subscriber_topic_))
        return false;
    
    return true;
}

void PackageTemplate::topicCallback(const sensor_msgs::Temperature& message)
{
    algorithm_.addData(message.temperature);
}

bool PackageTemplate::serviceCallback(std_srvs::Trigger::Request& request, 
                                      std_srvs::Trigger::Response& response)
{
    response.success = true;
    response.message = "The average is " + std::to_string(algorithm_.getAverage());
    return true;
}

}  // end package_template namespace