/* ----------------------------------------------------------------------------
 * Copyright (c) 2021, University of Leeds and Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   TestAlgorithm.cpp
 *  @author Jun Li (junlileeds@gmail.com)
 *  @brief  Test file for Algorithm class
 *  @date   May 02, 2022
 **/

#include "package_template/Algorithm.hpp"

// std
#include <vector>

// sys
#include <Eigen/Dense>

// gtest
#include <gtest/gtest.h>

using namespace package_template;

TEST(Algorithm, getWithoutSet)
{
    Algorithm algorithm;
    const double average = algorithm.getAverage();
    EXPECT_EQ(average, 0.0);
}

TEST(Algorithm, singleDataPoint)
{
  const double input_data = 100.0 * (double)rand() / RAND_MAX;
  Algorithm algorithm;
  algorithm.addData(input_data);
  const double average = algorithm.getAverage();
  EXPECT_NEAR(input_data, average, 1e-10);
}

TEST(Algorithm, singleDataVector)
{
  const double input_value = 100.0 * (double)rand() / RAND_MAX;
  Algorithm algorithm;
  Eigen::VectorXd input_data;
  input_data.resize(2);
  input_data << input_value, 3*input_value;
  algorithm.addData(input_data);
  const double average = algorithm.getAverage();
  EXPECT_NEAR(2*input_value, average, 1e-10);
}

TEST(Algorithm, multipleDataPoints)
{
  std::size_t num_meas = 100;
  std::vector<double> input_data(num_meas);
  double sum = 0.0;
  for (auto& data : input_data) {
    data = 100.0 * (double)rand() / RAND_MAX;
    sum += data;
  }

  Algorithm algorithm;
  for (const auto data : input_data) {
    algorithm.addData(data);
  }
  const double average = algorithm.getAverage();
  EXPECT_NEAR(sum / num_meas, average, 1e-10);
}