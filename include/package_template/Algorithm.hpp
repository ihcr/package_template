/* ----------------------------------------------------------------------------
 * Copyright (c) 2021, University of Leeds and Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   Algorithm.hpp
 *  @author Jun Li (junlileeds@gmail.com)
 *  @brief  Header file for Algorithm class 
 *  @date   May 02, 2022
 **/

#pragma once

// std
#include <memory>

// sys
#include <Eigen/Dense>

namespace package_template {

/**
 * @brief Class containing the algorithmic part of the package.
 */
class Algorithm
{
public:
    /**
     * @brief Construct a new Algorithm object
     */
    Algorithm();

    /**
     * @brief Destroy the Algorithm object
     */
    virtual ~Algorithm();

    /**
     * @brief Add new measurement data.
     * 
     * @param data the new data.
     */
    void addData(const double data);

    /**
     * @brief Add multiple measurements as once.
     * 
     * @param data new data.
     */
    void addData(const Eigen::VectorXd& data);

    /**
     * @brief Get the computed average of the data.
     * 
     * @return double the average of the data.
     */
    double getAverage() const;

private:
    // Forward declared structure that will contain the data
    struct Data;

    // Pointer to data
    std::unique_ptr<Data> data_;
};

}  // end package_template namespace