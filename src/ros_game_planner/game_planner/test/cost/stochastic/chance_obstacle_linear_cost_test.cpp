/*
 * Copyright (c) 2019, The Regents of the University of California (Regents).
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
// Obstacle Avoidance Linear Chance Cost
// This cost only deals with the Rectangular shaped obstacle
//
///////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
#include "utils/types.h"
#include "utils/obstacle/rectangular_obstacle.h"
#include "cost/deterministic/cost.h"
#include "cost/deterministic/obstacle_linear_cost.h"
#include "cost/stochastic/chance_obstacle_linear_cost.h"

using namespace game_planner;

TEST(CHANCE_OBSTACLE_LINEAR_COST, EVALUATE1)
{
    const double weight = 1.0;
    const double prob = 0.95;
    const std::pair<int, int> position_idx = {0, 1};

    // Obstacle
    const double length = 8.0; // x direction
    const double wide   = 4.0; // y direction
    const double obs_x = 0.0;
    const double obs_y = 0.0;
    const double obs_theta = 0.0;

    RectangularObstacle obs(obs_x, obs_y, obs_theta, length, wide);
    ChanceObstacleLinearCost obs_linear_cost(weight, prob, obs, position_idx, "obstacle_linear_cost");

    Eigen::VectorXd grad = Eigen::VectorXd::Zero(5);
    Eigen::MatrixXd hess = Eigen::MatrixXd::Zero(5, 5);
    Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(5, 5);
    Eigen::VectorXd input1 = Eigen::VectorXd::Zero(5);
    input1(0) = 6.0;
    double cost1 = obs_linear_cost.evaluate(input1, cov);
    EXPECT_NEAR(cost1, 0.0, constants::kSmallNumber);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "chance_obstacle_linear_cost_test");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}