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
 * Authors:  Yutaka Shimizu
 */

///////////////////////////////////////////////////////////////////////////////
//
// TEST Code for ILQ Solver
//
///////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
#include <memory>
#include "dynamics/multi_player_dynamical_system.h"
#include "dynamics/single_player_unicycle_4d.h"
#include "dynamics/single_player_car_6d.h"
#include "cost/player_cost.h"
#include "cost/deterministic/quadratic_polyline2_cost.h"
#include "cost/deterministic/semiquadratic_polyline2_cost.h"
#include "cost/deterministic/quadratic_cost.h"
#include "cost/deterministic/proximity_cost.h"
#include "cost/deterministic/obstacle_cost.h"
#include "solver/lq_solver/lq_feedback_nash_solver.h"
#include "solver/iterative_solver/ilq_solver.h"

using namespace game_planner;

TEST(ILQRSOLVER, SOLVE)
{
    // Car inter-axle distance.
    constexpr double kInterAxleLength = 4.0;  // m

    // Cost weights.
    constexpr double kOmegaCostWeight = 500000.0;
    constexpr double kJerkCostWeight = 500.0;

    //constexpr double kACostWeight = 50.0;
    constexpr double kACostWeight = 5.0;
    constexpr double kP1NominalVCostWeight = 10.0;
    constexpr double kP2NominalVCostWeight = 1.0;

    constexpr double kLaneCostWeight = 25.0;
    constexpr double kLaneBoundaryCostWeight = 100.0;

    constexpr double kMinProximity = 5.0;
    constexpr double kP1ProximityCostWeight = 100.0;
    constexpr double kP1ObstacleCostWeight = 100.0;

    // Heading weight
    constexpr bool kOrientedRight = true;

    // Lane width.
    constexpr double kLaneHalfWidth = 2.5;  // m

    // Nominal speed.
    constexpr double kP1NominalV = 15.0;  // m/s
    constexpr double kP2NominalV = 10.0;  // m/s

    // Initial state.
    constexpr double kP1InitialX = 2.5;    // m
    constexpr double kP1InitialY = -10.0;  // m

    constexpr double kP2InitialX = 2.5;   // m
    constexpr double kP2InitialY = 10.0;  // m

    constexpr double kP1InitialHeading = M_PI_2;  // rad
    constexpr double kP2InitialHeading = M_PI_2;  // rad

    constexpr double kP1InitialSpeed = 10.0;  // m/s
    constexpr double kP2InitialSpeed = 2.0;   // m/s

    // State dimensions.
    using P1 = SinglePlayerCar6D;
    using P2 = SinglePlayerCar6D;

    const int kP1XIdx = P1::kPxIdx;
    const int kP1YIdx = P1::kPyIdx;
    const int kP1HeadingIdx = P1::kThetaIdx;
    const int kP1PhiIdx = P1::kPhiIdx;
    const int kP1VIdx = P1::kVIdx;
    const int kP1AIdx = P1::kAIdx;

    const int kP2XIdx = P1::kNumXDims + P2::kPxIdx;
    const int kP2YIdx = P1::kNumXDims + P2::kPyIdx;
    const int kP2HeadingIdx = P1::kNumXDims + P2::kThetaIdx;
    const int kP2PhiIdx = P1::kNumXDims + P2::kPhiIdx;
    const int kP2VIdx = P1::kNumXDims + P2::kVIdx;
    const int kP2AIdx = P1::kNumXDims + P2::kAIdx;

    // Control dimensions.
    const int kP1OmegaIdx = 0;
    const int kP1JerkIdx = 1;
    const int kP2OmegaIdx = 0;
    const int kP2JerkIdx = 1;

    // Dynamics
    std::shared_ptr<SinglePlayerDynamicalSystem> player1 = std::make_shared<P1>(kInterAxleLength);
    std::shared_ptr<SinglePlayerDynamicalSystem> player2 = std::make_shared<P2>(kInterAxleLength);
    std::vector<std::shared_ptr<SinglePlayerDynamicalSystem>> players = {player1, player2};
    std::shared_ptr<MultiPlayerDynamicalSystem> dynamics = std::make_shared<MultiPlayerDynamicalSystem>(players);

    // Initial State
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(dynamics->getXDim());
    x0(kP1XIdx) = kP1InitialX;
    x0(kP1YIdx) = kP1InitialY;
    x0(kP1HeadingIdx) = kP1InitialHeading;
    x0(kP1VIdx) = kP1InitialSpeed;
    x0(kP2XIdx) = kP2InitialX;
    x0(kP2YIdx) = kP2InitialY;
    x0(kP2HeadingIdx) = kP2InitialHeading;
    x0(kP2VIdx) = kP2InitialSpeed;

    // Player Costs
    std::vector<PlayerCost> player_costs;
    // Set up costs for all players
    player_costs.emplace_back("P1");
    player_costs.emplace_back("P2");
    auto& p1_cost = player_costs[0];
    auto& p2_cost = player_costs[1];

    // Create Two Lanes
    const Polyline2 lane({Eigen::Vector2d(kP1InitialX, -1000), Eigen::Vector2d(kP1InitialX, 1000)});
    const Polyline2 lane2({Eigen::Vector2d(-1.0, -1000), Eigen::Vector2d(-1.0, 1000)});

    // Stay in lines
    const std::shared_ptr<QuadraticPolyline2Cost> p1_lane_cost(new QuadraticPolyline2Cost(kLaneCostWeight, lane2,
                                                                                          {kP1XIdx, kP1YIdx},
                                                                                          "LaneCenter"));
    const std::shared_ptr<SemiquadraticPolyline2Cost> p1_lane_r_cost(new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight,
                                                                                                    lane2,
                                                                                                    {kP1XIdx, kP1YIdx},
                                                                                                    kLaneHalfWidth,
                                                                                                    kOrientedRight,
                                                                                                    "LaneRightBoundary"));
    const std::shared_ptr<SemiquadraticPolyline2Cost> p1_lane_l_cost(new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight,
                                                                                                    lane2,
                                                                                                    {kP1XIdx, kP1YIdx},
                                                                                                    -kLaneHalfWidth,
                                                                                                    !kOrientedRight, "LaneLeftBoundary"));
    const std::shared_ptr<QuadraticPolyline2Cost> p2_lane_cost(new QuadraticPolyline2Cost(kLaneCostWeight, lane,
                                                                                          {kP2XIdx, kP2YIdx},
                                                                                          "LaneCenter"));
    const std::shared_ptr<SemiquadraticPolyline2Cost> p2_lane_r_cost(
            new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, lane,
                                           {kP2XIdx, kP2YIdx}, kLaneHalfWidth,
                                           kOrientedRight, "LaneRightBoundary"));
    const std::shared_ptr<SemiquadraticPolyline2Cost> p2_lane_l_cost(
            new SemiquadraticPolyline2Cost(kLaneBoundaryCostWeight, lane,
                                           {kP2XIdx, kP2YIdx}, -kLaneHalfWidth,
                                           !kOrientedRight, "LaneLeftBoundary"));

    p1_cost.addStateCost(p1_lane_cost);
    p1_cost.addStateCost(p1_lane_l_cost);
    p1_cost.addStateCost(p1_lane_r_cost);

    p2_cost.addStateCost(p2_lane_cost);
    p2_cost.addStateCost(p2_lane_l_cost);
    p2_cost.addStateCost(p2_lane_r_cost);

    // Nominal Speed
    const auto p1_nominal_v_cost = std::make_shared<QuadraticCost>(kP1NominalVCostWeight, kP1VIdx, kP1NominalV, "NominalV");
    p1_cost.addStateCost(p1_nominal_v_cost);

    const auto p2_nominal_v_cost = std::make_shared<QuadraticCost>(kP2NominalVCostWeight, kP2VIdx, kP2NominalV, "NominalV");
    p2_cost.addStateCost(p2_nominal_v_cost);

    // Penalize control effort
    const auto p1_omega_cost = std::make_shared<QuadraticCost>(kOmegaCostWeight, kP1OmegaIdx, 0.0, "Steering");
    const auto p1_jerk_cost = std::make_shared<QuadraticCost>(kJerkCostWeight, kP1JerkIdx, 0.0, "Jerk");
    p1_cost.addControlCost(0, p1_omega_cost);
    p1_cost.addControlCost(0, p1_jerk_cost);

    const auto p2_omega_cost = std::make_shared<QuadraticCost>(kOmegaCostWeight, kP2OmegaIdx, 0.0, "Steering");
    const auto p2_jerk_cost = std::make_shared<QuadraticCost>(kJerkCostWeight, kP2JerkIdx, 0.0, "Jerk");
    p2_cost.addControlCost(1, p2_omega_cost);
    p2_cost.addControlCost(1, p2_jerk_cost);

    // Pairwise Proximity Cost
    const std::shared_ptr<ProximityCost> p1p2_proximity_cost(new ProximityCost(kP1ProximityCostWeight,
                                                                               {kP1XIdx, kP1YIdx},
                                                                               {kP2XIdx, kP2YIdx},
                                                                               kMinProximity, "ProximityP2"));
    p1_cost.addStateCost(p1p2_proximity_cost);

    // Obstacle Cost
    const double obs1_x = -1.0;
    const double obs1_y = 55.0;
    const double obs1_theta = 0.0;
    const double obs1_radius = 1.0;
    const double threshold = obs1_radius;
    const Eigen::MatrixXd obs1_cov = Eigen::MatrixXd::Zero(2, 2);
    CircularObstacle obs1(obs1_x, obs1_y, obs1_theta, obs1_cov, obs1_radius);
    const std::shared_ptr<ObstacleCost> p1_obstacle_cost(new ObstacleCost(kP1ObstacleCostWeight,
                                                                          obs1,
                                                                          {kP1XIdx, kP1YIdx},threshold,
                                                                          "ObstacleAvoidance"));
    p1_cost.addStateCost(p1_obstacle_cost);

    // Create New Log
    std::shared_ptr<SolverLog> log = std::make_shared<SolverLog>();

    // Create Initial Strategy (Zero)
    std::vector<Strategy> initial_strategies;
    for (unsigned int ii = 0; ii < dynamics->getNumPlayers(); ii++)
        initial_strategies.emplace_back(time::kNumTimeSteps, dynamics->getXDim(), dynamics->getUDim(ii));

    // Solver Params
    game_planner::SolverParams params;
    params.max_backtracking_steps = 100;
    params.linesearch = true;
    params.expected_decrease_fraction = 0.1;
    params.initial_alpha_scaling = 0.75;
    params.convergence_tolerance = 0.01;

    //Input and Observation Covariance
    std::vector<Eigen::MatrixXd> input_covariances(dynamics->getNumPlayers());
    std::vector<Eigen::MatrixXd> observation_covariances(dynamics->getNumPlayers());
    for(unsigned int player_id =0; player_id<input_covariances.size(); ++player_id)
    {
        input_covariances[player_id] = Eigen::MatrixXd::Zero(dynamics->getUDim(player_id),
                                                             dynamics->getUDim(player_id));
        observation_covariances[player_id] = Eigen::MatrixXd::Zero(dynamics->getXDim(),
                                                                   dynamics->getXDim());
    }

    std::shared_ptr<Problem> problem = std::make_shared<Problem>(dynamics, player_costs, x0,
                                                                 Eigen::MatrixXd::Zero(x0.size(), x0.size()),
                                                                 input_covariances, observation_covariances);
    ILQSolver solver(problem, params);
    bool success = solver.solve(initial_strategies, log);

    EXPECT_TRUE(success);
    EXPECT_NEAR(log->getFinalOperatingPoint().xs.back()(0), 0.534889, constants::kSmallNumber);
    EXPECT_NEAR(log->getFinalOperatingPoint().xs.back()(1), 120.438, 0.01);
    EXPECT_NEAR(log->getFinalOperatingPoint().xs.back()(2), 1.49527, constants::kSmallNumber);
    EXPECT_NEAR(log->getFinalOperatingPoint().xs.back()(3), -0.00718703, constants::kSmallNumber);
    EXPECT_NEAR(log->getFinalOperatingPoint().xs.back()(4), 16.0978, constants::kSmallNumber);
    EXPECT_NEAR(log->getFinalOperatingPoint().xs.back()(5), 0.463398, constants::kSmallNumber);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "ilq_solver_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}