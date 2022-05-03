/* ----------------------------------------------------------------------------
 * Copyright (c) 2021, University of Leeds and Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   Algorithm.cpp
 *  @author Jun Li (junlileeds@gmail.com)
 *  @brief  Source file for Algorithm class 
 *  @date   May 02, 2022
 **/

#include "package_template/Algorithm.hpp"

// std
#include <utility>

// boost
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/count.hpp>

namespace package_template {

using namespace boost::accumulators;

struct Algorithm::Data
{
    accumulator_set<double, features<tag::mean, tag::count>> acc;
};

Algorithm::Algorithm()
{
    data_ = std::make_unique<Data>();
}

Algorithm::~Algorithm() = default;

void Algorithm::addData(const double data)
{
    data_->acc(data);
}

void Algorithm::addData(const Eigen::VectorXd& data)
{
    for (auto i = 0; i < data.size(); ++i)
        addData(data[i]);
}

double Algorithm::getAverage() const
{
    return count(data_->acc) ? mean(data_->acc) : 0;
}

}  // end package_template namespace