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
// Test Code for Obstacle Constraint
//
///////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
#include "utils/types.h"
#include "constraint/deterministic/obstacle_constraint.h"

using namespace game_planner;

TEST(OBSTACLECONSTRAINT, EVALUATE)
{
    const double obs_x = 1.0;
    const double obs_y = 1.0;
    const double obs_theta = 0.0;
    const double obs_radius = 3.0;
    const Eigen::MatrixXd obs_cov = Eigen::MatrixXd::Zero(2, 2);
    CircularObstacle obs(obs_x, obs_y, obs_theta, obs_cov, obs_radius);
    const std::pair<int, int> idxs = {0, 1};
    const double threshold1 = 1.0 + obs_radius;
    const std::string name = "obstacle_cost";

    ObstacleConstraint constraint(obs, idxs, threshold1, name);

    const int size = 5;
    Eigen::VectorXd input1 = Eigen::VectorXd::Zero(size);
    EXPECT_NEAR(constraint.evaluate(0, input1), threshold1 - std::sqrt(2), constants::kSmallNumber);

    Eigen::VectorXd input2 = Eigen::VectorXd::Zero(size);
    input2(0) = 5.0;
    EXPECT_NEAR(constraint.evaluate(0, input2), threshold1 - std::sqrt(17.0), constants::kSmallNumber);
}

TEST(OBSTACLECONSTRAINT, QUADRATICIZE)
{
    const double obs_x = 1.0;
    const double obs_y = 1.0;
    const double obs_theta = 0.0;
    const double obs_radius = 3.0;
    const Eigen::MatrixXd obs_cov = Eigen::MatrixXd::Zero(2, 2);
    CircularObstacle obs(obs_x, obs_y, obs_theta, obs_cov, obs_radius);
    const std::pair<int, int> idxs = {0, 1};
    const double threshold1 = 1.0 + obs_radius;
    const std::string name = "obstacle_cost";

    ObstacleConstraint constraint(obs, idxs, threshold1, name);

    const int size = 3;
    Eigen::VectorXd input1 = Eigen::VectorXd::Zero(size);
    Eigen::MatrixXd hess = Eigen::MatrixXd::Zero(size, size);
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(size);
    constraint.quadraticize(0, input1, hess, grad);

    EXPECT_NEAR(grad(0), 18.2843, constants::kSmallNumber);
    EXPECT_NEAR(grad(1), 18.2843, constants::kSmallNumber);
    EXPECT_NEAR(hess(0,0), -4.14214, constants::kSmallNumber);
    EXPECT_NEAR(hess(1,1), -4.14214, constants::kSmallNumber);
    EXPECT_NEAR(hess(0,1), 14.1421, constants::kSmallNumber);
    EXPECT_NEAR(hess(1,0), 14.1421, constants::kSmallNumber);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "obstacle_constraint_test");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}