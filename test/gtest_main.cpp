/* ----------------------------------------------------------------------------
 * Copyright (c) 2021, University of Leeds and Harbin Institute of Technology.
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   gtest_main.cpp
 *  @author Jun Li (junlileeds@gmail.com)
 *  @brief  Test files
 *  @date   May 02, 2022
 **/

#include <gtest/gtest.h>

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    srand((int)time(0));

    return RUN_ALL_TESTS();
}