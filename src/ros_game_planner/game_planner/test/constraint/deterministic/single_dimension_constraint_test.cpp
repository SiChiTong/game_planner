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
// Test Code For Single Dimension Constraint
//
///////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
#include "utils/types.h"
#include "constraint/deterministic/single_dimension_constraint.h"

using namespace game_planner;

TEST(SINGLEDIMENSIONCONSTRAINT, EVALUATE)
{
    const int dim = 0;
    const double threshold = 1.0;
    const bool keep_below = true;
    SingleDimensionConstraint constraint(dim ,threshold, keep_below, "input_constraint");

    EXPECT_FALSE(constraint.getIsEquality());

    Eigen::VectorXd input1 = Eigen::VectorXd::Zero(2);
    EXPECT_NEAR(constraint.evaluate(input1), -1.0, constants::kSmallNumber);
    double value;
    EXPECT_TRUE(constraint.isSatisfied(0, input1, &value));
    EXPECT_NEAR(value, -1.0, constants::kSmallNumber);

    Eigen::VectorXd input2 = Eigen::VectorXd::Zero(2);
    input2(0) = 1.0;
    EXPECT_NEAR(constraint.evaluate(input2), 0.0, constants::kSmallNumber);
    EXPECT_TRUE(constraint.isSatisfied(0, input2, &value));
    EXPECT_NEAR(value, 0.0, constants::kSmallNumber);

    Eigen::VectorXd input3 = Eigen::VectorXd::Zero(2);
    input3(1) = 1.0;
    EXPECT_NEAR(constraint.evaluate(input3), -1.0, constants::kSmallNumber);
    EXPECT_TRUE(constraint.isSatisfied(0, input3, &value));
    EXPECT_NEAR(value, -1.0, constants::kSmallNumber);

    Eigen::VectorXd input4 = Eigen::VectorXd::Zero(2);
    input4(0) = 5.0;
    EXPECT_NEAR(constraint.evaluate(input4), 4.0, constants::kSmallNumber);
    EXPECT_FALSE(constraint.isSatisfied(0, input4, &value));
    EXPECT_NEAR(value, 4.0, constants::kSmallNumber);
}

TEST(SINGLEDIMENSIONCONSTRAINT, QUADRATICIZE)
{
    const int dim = 0;
    const double threshold = 1.0;
    const bool keep_below = true;
    SingleDimensionConstraint constraint(dim, threshold, keep_below, "input_constraint");

    const int size = 2;
    Eigen::MatrixXd hess = Eigen::MatrixXd::Zero(size, size);
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(size);

    Eigen::VectorXd input1 = Eigen::VectorXd::Zero(size);
    constraint.quadraticize(0, input1, hess, grad);

    for(size_t i=0; i<size; ++i)
    {
        EXPECT_NEAR(grad(i), 0.0, constants::kSmallNumber);
        for(size_t j=0; j<size; ++j)
            EXPECT_NEAR(hess(i,j), 0.0, constants::kSmallNumber);
    }

    Eigen::VectorXd input2 = Eigen::VectorXd::Zero(size);
    input2(0) = 5.0;
    constraint.quadraticize(0, input2, hess, grad);
    EXPECT_NEAR(grad(dim), (input2(dim)-threshold)*constants::kDefaultMu, constants::kSmallNumber);
    EXPECT_NEAR(hess(dim,dim), constants::kDefaultMu, constants::kSmallNumber);
}

TEST(SINGLEDIMENSIONCONSTRAINT, EVALUATE2)
{
    const int dim = 0;
    const double threshold = 1.0;
    const bool keep_below = true;
    SingleDimensionConstraint constraint(dim ,threshold, keep_below, "input_constraint");

    Eigen::VectorXd input1 = Eigen::VectorXd::Zero(2);
    EXPECT_NEAR(constraint.getMu(0, input1), 0.0, constants::kSmallNumber);
    EXPECT_NEAR(constraint.getGlobalMu(), constants::kDefaultMu, constants::kSmallNumber);
    EXPECT_NEAR(constraint.getLambda(0), constants::kDefaultLambda, constants::kSmallNumber);

    Eigen::VectorXd input2 = Eigen::VectorXd::Zero(2);
    input2(0) = 5.0;
    EXPECT_NEAR(constraint.getMu(0, input2), constants::kDefaultMu, constants::kSmallNumber);
    EXPECT_NEAR(constraint.getGlobalMu(), constants::kDefaultMu, constants::kSmallNumber);

    const double scale = 2.0;
    for(unsigned int i=0; i<time::kNumTimeSteps; ++i)
        EXPECT_NEAR(constraint.getLambda(i), constants::kDefaultLambda, constants::kSmallNumber);
    constraint.scaleLambdas(scale);
    for(unsigned int i=0; i<time::kNumTimeSteps; ++i)
        EXPECT_NEAR(constraint.getLambda(i), constants::kDefaultLambda*scale, constants::kSmallNumber);

    const double increment_value = 2.0;
    constraint.incrementLambda(0, increment_value);
    EXPECT_NEAR(constraint.getLambda(0), constants::kDefaultLambda + increment_value*constants::kDefaultMu, constants::kSmallNumber);
    constraint.scaleLambdas(scale);
    EXPECT_NEAR(constraint.getLambda(0), scale*(constants::kDefaultLambda+increment_value*constants::kDefaultMu), constants::kSmallNumber);

    ALConstraint::scaleMu(scale);
    EXPECT_NEAR(constraint.getMu(0, input1), constants::kDefaultMu*scale, constants::kSmallNumber);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "single_dimension_constraint_test");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
