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
// This is a test code for chance constraint player cost
//
///////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
#include "cost/player_cost.h"
#include "cost/deterministic/quadratic_polyline2_cost.h"
#include "cost/stochastic/chance_obstacle_linear_cost.h"
#include "cost/deterministic/quadratic_cost.h"
#include "cost/deterministic/obstacle_linear_cost.h"
#include "utils/types.h"

using namespace game_planner;

//Parameters
static constexpr double kCostWeight = 1.0;
static constexpr size_t kNumPlayers = 2;
static constexpr int kVectorDimension = 5;
static constexpr double prob = 0.95;
static constexpr std::pair<int, int> position_idx = {0, 1};

class PlayerChanceCostTest : public ::testing::Test
{
protected:
    void SetUp()
    {
        // Obstacle
        const double length = 8.0; // x direction
        const double width  = 4.0; // y direction
        const double obs_x = 0.0;
        const double obs_y = 0.0;
        const double obs_theta = 0.0;
        RectangularObstacle obs(obs_x, obs_y, obs_theta, length, width);
        player_cost_.addStateChanceCost(std::make_shared<ChanceObstacleLinearCost>(kCostWeight, prob, obs, position_idx));

        // Choose a random state and controls.
        x_ = Eigen::VectorXd::Zero(kVectorDimension);
        x_(0) = 6.0;
        for (size_t ii = 0; ii < kNumPlayers; ii++)
            us_.emplace_back(Eigen::VectorXd::Zero(kVectorDimension));
    }

    // PlayerCost, state, and controls for each player.
    PlayerCost player_cost_;
    Eigen::VectorXd x_;
    std::vector<Eigen::VectorXd> us_;
};

// Test that we evaluate correctly.
TEST_F(PlayerChanceCostTest, Evaluate)
{
    Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(kVectorDimension, kVectorDimension);
    std::vector<Eigen::MatrixXd> control_covs(kNumPlayers);
    for(unsigned int player_id=0; player_id<kNumPlayers; ++player_id)
        control_covs[player_id] = Eigen::MatrixXd::Zero(kVectorDimension, kVectorDimension);

    const double value1 = player_cost_.evaluate(0, x_, us_, cov, control_covs);
    EXPECT_NEAR(value1, 0.0, constants::kSmallNumber);

    const double value2 = player_cost_.evaluate(0, x_, us_);
    EXPECT_NEAR(value2, 0.0, constants::kSmallNumber);

    // Obstacle
    const double length = 8.0; // x direction
    const double width  = 4.0; // y direction
    const double obs_x = 0.0;
    const double obs_y = 0.0;
    const double obs_theta = 0.0;
    RectangularObstacle obs(obs_x, obs_y, obs_theta, length, width);
    player_cost_.addStateCost(std::make_shared<ObstacleLinearCost>(kCostWeight, obs, position_idx));

    const double value3 = player_cost_.evaluate(0, x_, us_);
    EXPECT_NEAR(value3, 0.0, constants::kSmallNumber);

    const double value4 = player_cost_.evaluate(0, x_, us_, cov, control_covs);
    EXPECT_NEAR(value4, 0.0, constants::kSmallNumber);
}

TEST_F(PlayerChanceCostTest, Quadraticize)
{
    player_cost_.addControlCost(0, std::make_shared<QuadraticCost>(kCostWeight, -1));
    player_cost_.addControlCost(1, std::make_shared<QuadraticCost>(kCostWeight, -1));
    Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(kVectorDimension, kVectorDimension);

    std::vector<Eigen::MatrixXd> control_covs(kNumPlayers);
    for(unsigned int player_id=0; player_id<kNumPlayers; ++player_id)
        control_covs[player_id] = Eigen::MatrixXd::Zero(kVectorDimension, kVectorDimension);

    QuadraticCostApproximation result = player_cost_.quadraticize(0, x_, us_, cov, control_covs);

    for(int r=0; r < result.state_.hess_.rows(); r++)
        for(int c=0; c < result.state_.hess_.rows(); c++)
            EXPECT_NEAR(result.state_.hess_(r,c), 0.0, constants::kSmallNumber);

    EXPECT_NEAR(result.state_.grad_(0), 0.0, constants::kSmallNumber);
    EXPECT_NEAR(result.state_.grad_(1), 0.0, constants::kSmallNumber);

    for(unsigned int player_id = 0; player_id < kNumPlayers; ++player_id)
    {
        EXPECT_EQ(result.control_.find(player_id)->second.hess_.rows(), result.control_.find(player_id)->second.grad_.size());
        for(int r=0; r < result.control_.find(player_id)->second.hess_.rows(); r++)
        {
            EXPECT_NEAR(result.control_.find(player_id)->second.grad_(r), 0.0, constants::kSmallNumber);
            for(int c=0; c < result.control_.find(player_id)->second.hess_.cols(); c++)
            {
                if(r!=c)
                    EXPECT_NEAR(result.control_.find(player_id)->second.hess_(r,c), 0.0, constants::kSmallNumber);
                else
                    EXPECT_NEAR(result.control_.find(player_id)->second.hess_(r,c), 1.0, constants::kSmallNumber);
            }
        }
    }

}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "player_chance_cost_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}


