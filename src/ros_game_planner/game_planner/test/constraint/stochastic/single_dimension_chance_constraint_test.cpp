/*
 * Copyright (c) 2020, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: Yutaka Shimizu
 */

///////////////////////////////////////////////////////////////////////////////
//
// Test Code For Single Dimension Chance Constraint
//
///////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
#include "utils/types.h"
#include "utils/math_utils.h"
#include "constraint/stochastic/single_dimension_chance_constraint.h"

using namespace game_planner;

TEST(SINGLEDIMENSIONCHANCECONSTRAINT, EVALUATE1)
{
    const int size = 2;
    const double prob = 0.98;
    const int dim = 0;
    double threshold = 1.0;
    bool keep_below = true;
    SingleDimensionChanceConstraint constraint(prob, dim, threshold, keep_below, "input_chance_constraint");

    EXPECT_FALSE(constraint.getIsEquality());

    Eigen::VectorXd input1 = Eigen::VectorXd::Zero(size);

    Eigen::MatrixXd cov = 0.01*Eigen::MatrixXd::Identity(size, size);
    const double value1 = input1(0) - threshold + std::sqrt(2*cov(dim, dim))*math::computeInverseErrorFunction(2*prob-1);
    EXPECT_NEAR(constraint.evaluate(input1, cov), value1, constants::kSmallNumber);

    threshold = -1.0;
    keep_below = false;
    SingleDimensionChanceConstraint constraint2(prob, dim, threshold, keep_below, "input_chance_constraint");
    const double value2 = -input1(0) + threshold + std::sqrt(2*cov(dim, dim))*math::computeInverseErrorFunction(2*prob-1);
    std::cout << "value2: " << value2 << std::endl;
    EXPECT_NEAR(constraint2.evaluate(input1, cov), value2, constants::kSmallNumber);
    EXPECT_NEAR(value1, value2, constants::kSmallNumber);
}

TEST(SINGLEDIMENSIONCHANCECONSTRAINT, EVALUATE2)
{
    const int size = 2;
    const double prob = 0.98;
    const int dim = 0;
    const double threshold = -1.0;
    const bool keep_below = false;
    SingleDimensionChanceConstraint constraint(prob, dim, threshold, keep_below, "input_chance_constraint");

    EXPECT_FALSE(constraint.getIsEquality());

    Eigen::VectorXd input1 = Eigen::VectorXd::Zero(size);
    Eigen::MatrixXd cov = 0.1*Eigen::MatrixXd::Identity(size, size);
    const double value1 = -input1(0) + threshold + std::sqrt(2*cov(dim, dim))*math::computeInverseErrorFunction(2*prob-1);
    EXPECT_NEAR(constraint.evaluate(input1, cov), value1, constants::kSmallNumber);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "single_dimension_chance_constraint_test");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}

