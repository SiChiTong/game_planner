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
// Test Code for Obstacle Linear Constraint
//
///////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
#include "utils/types.h"
#include "constraint/deterministic/obstacle_linear_constraint.h"

using namespace game_planner;

TEST(OBSTACLELINEARCONSTRAINT, EVALUATE)
{
    const std::pair<int, int> position_idx = {0, 1};

    // Obstacle
    const double length = 8.0; // x direction
    const double width  = 4.0; // y direction
    const double obs_x = 0.0;
    const double obs_y = 0.0;
    const double obs_theta = 0.0;
    RectangularObstacle obs(obs_x, obs_y, obs_theta, length, width);

    ObstacleLinearConstraint constraint(obs, position_idx, "obstacle_linear_constraint");

    Eigen::VectorXd input1 = Eigen::VectorXd::Zero(5);
    input1(0) = 6.0;
    double value1 = constraint.evaluate(input1);
    EXPECT_NEAR(value1, -2.0, constants::kSmallNumber);

    Eigen::VectorXd input2 = Eigen::VectorXd::Zero(5);
    input2(0) = 2.0;
    double value2 = constraint.evaluate(input2);
    EXPECT_NEAR(value2, 2.0, constants::kSmallNumber);

    Eigen::VectorXd input3 = Eigen::VectorXd::Zero(5);
    input3(0) = 3.0;
    double cost_value3 = constraint.evaluate(input3);
    EXPECT_NEAR(cost_value3, 1.0, constants::kSmallNumber);

    Eigen::VectorXd input4 = Eigen::VectorXd::Zero(5);
    input4(0) = -2.0;
    double cost_value4 = constraint.evaluate(input4);
    EXPECT_NEAR(cost_value4, 2.0, constants::kSmallNumber);

    Eigen::VectorXd input5 = Eigen::VectorXd::Zero(5);
    input5(0) = -3.0;
    double cost_value5 = constraint.evaluate(input5);
    EXPECT_NEAR(cost_value5, 1.0, constants::kSmallNumber);

    Eigen::VectorXd input6 = Eigen::VectorXd::Zero(5);
    input6(0) = 2.0;
    input6(1) = 2.0;
    double cost_value6 = constraint.evaluate(input6);
    EXPECT_NEAR(cost_value6, 0.0, constants::kSmallNumber);

    Eigen::VectorXd input7 = Eigen::VectorXd::Zero(5);
    input7(0) = 2.0;
    input7(1) = 1.0;
    double cost_value7 = constraint.evaluate(input7);
    EXPECT_NEAR(cost_value7, 1.0, constants::kSmallNumber);

    Eigen::VectorXd input8 = Eigen::VectorXd::Zero(5);
    input8(0) = 6.0;
    input8(1) = 6.0;
    double cost_value8 = constraint.evaluate(input8);
    EXPECT_NEAR(cost_value8, -2*sqrt(5), constants::kSmallNumber);
}

TEST(OBSTACLELINEARCONSTRAINT, QUADRATIZE1)
{
    const std::pair<int, int> position_idx = {0, 1};

    // Obstacle
    const double length = 8.0; // x direction
    const double wide = 4.0; // y direction
    const double obs_x = 0.0;
    const double obs_y = 0.0;
    const double obs_theta = M_PI / 3;

    RectangularObstacle obs(obs_x, obs_y, obs_theta, length, wide);
    ObstacleLinearConstraint constraint(obs, position_idx, "obstacle_linear_cost");

    Eigen::VectorXd grad = Eigen::VectorXd::Zero(5);
    Eigen::MatrixXd hess = Eigen::MatrixXd::Zero(5, 5);
    Eigen::VectorXd input1 = Eigen::VectorXd::Zero(5);
    input1(0) = 6.0;
    constraint.quadraticize(0,input1, hess, grad);

    for(unsigned int i=0; i<hess.rows(); ++i)
    {
        EXPECT_NEAR(grad(i), 0.0, constants::kSmallNumber);
        for(unsigned int j=0; j<hess.cols(); ++j)
            EXPECT_NEAR(hess(i,j), 0.0, constants::kSmallNumber);
    }

    Eigen::VectorXd input2 = Eigen::VectorXd::Zero(5);
    input2(0) = 2.0;
    constraint.quadraticize(0, input2, hess, grad);

    EXPECT_NEAR(-(grad(0)/grad(1)), std::sqrt(3), constants::kSmallNumber);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "obstacle_linear_constraint_test");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}