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
// One Player and using stochastic and deterministic dynamics
// with hard constraints
//
///////////////////////////////////////////////////////////////////////////////
#ifndef SRC_ONE_PLAYER_LANE_CHANGE_H
#define SRC_ONE_PLAYER_LANE_CHANGE_H

#include "solver/iterative_solver/ilq_solver.h"
#include "solver/problem.h"
#include "cost/deterministic/cost.h"
#include "cost/player_cost.h"
#include "cost/deterministic/quadratic_cost.h"
#include "cost/deterministic/quadratic_polyline2_cost.h"
#include "cost/deterministic/proximity_cost.h"
#include "cost/deterministic/semiquadratic_polyline2_cost.h"
#include "cost/deterministic/obstacle_cost.h"
#include "cost/deterministic/obstacle_linear_cost.h"
#include "cost/stochastic/chance_obstacle_linear_cost.h"
#include "constraint/deterministic/obstacle_linear_constraint.h"
#include "constraint/deterministic/single_dimension_constraint.h"
#include "constraint/stochastic/obstacle_linear_chance_constraint.h"
#include "constraint/stochastic/single_dimension_chance_constraint.h"
#include "dynamics/multi_player_dynamical_system.h"
#include "dynamics/single_player_unicycle_4d.h"
#include "dynamics/single_player_car_6d.h"
#include "geometry/polyline2.h"
#include "utils/types.h"
#include "utils/obstacle/rectangular_obstacle.h"
#include "utils/obstacle/circular_obstacle.h"

namespace game_planner
{
    class OnePlayerLaneChange
    {
    public:
        ~OnePlayerLaneChange() = default;
        OnePlayerLaneChange() {}

        void initialize(std::shared_ptr<Problem>& ilq_problem,
                        std::shared_ptr<Problem>& ilqg_problem,
                        std::shared_ptr<std::vector<std::vector<int>>>& pos_dims,
                        std::shared_ptr<std::vector<Polyline2>>& lanes,
                        std::vector<std::shared_ptr<Obstacle>>& obstacles)
        {
            // Car inter-axle distance.
            constexpr double kInterAxleLength = 4.0;  // m

            // Probability
            constexpr double prob = 0.98;

            // Cost weights.
            constexpr double kOmegaCostWeight = 0.1;
            constexpr double kJerkCostWeight = 0.1;

            //constexpr double kACostWeight = 50.0;
            constexpr double kP1NominalVCostWeight = 10.0;

            // Control Constraints
            constexpr double kMaxOmega = 1.0;

            //constexpr double kLaneCostWeight = 25.0;
            //constexpr double kLaneBoundaryCostWeight = 100.0;
            constexpr double kLaneCostWeight = 0.50;
            constexpr double kLaneBoundaryCostWeight = 0.1;

            // Heading weight
            constexpr bool kOrientedRight = true;

            // Lane width.
            constexpr double kLaneHalfWidth = 2.5;  // m

            // Nominal speed.
            constexpr double kP1NominalV = 15.0;  // m/s

            // Initial state.
            constexpr double kP1InitialX = 2.5;    // m
            constexpr double kP1InitialY = -10.0;  // m
            constexpr double kP1InitialHeading = M_PI_2;  // rad
            constexpr double kP1InitialSpeed = 10.0;  // m/s

            // State dimensions.
            using P1 = SinglePlayerCar6D;

            const int kP1XIdx = P1::kPxIdx;
            const int kP1YIdx = P1::kPyIdx;
            const int kP1HeadingIdx = P1::kThetaIdx;
            const int kP1PhiIdx = P1::kPhiIdx;
            const int kP1VIdx = P1::kVIdx;
            const int kP1AIdx = P1::kAIdx;

            // Control dimensions.
            const int kP1OmegaIdx = 0;
            const int kP1JerkIdx = 1;

            // Dynamics
            std::shared_ptr<SinglePlayerDynamicalSystem> player1 = std::make_shared<P1>(kInterAxleLength);
            std::vector<std::shared_ptr<SinglePlayerDynamicalSystem>> players = {player1};
            std::shared_ptr<MultiPlayerDynamicalSystem> dynamics = std::make_shared<MultiPlayerDynamicalSystem>(players);

            // Initial State
            Eigen::VectorXd x0 = Eigen::VectorXd::Zero(dynamics->getXDim());
            x0(kP1XIdx) = kP1InitialX;
            x0(kP1YIdx) = kP1InitialY;
            x0(kP1HeadingIdx) = kP1InitialHeading;
            x0(kP1PhiIdx) = 0.0;
            x0(kP1VIdx) = kP1InitialSpeed;
            x0(kP1AIdx) = 0.0;

            // Initial State Covariance
            Eigen::MatrixXd Sig0 = Eigen::MatrixXd::Zero(x0.size(), x0.size());
            Sig0(0, 0) = 0.4;
            Sig0(1, 1) = 0.4;
            Sig0(2, 2) = 0.1;
            Sig0(3, 3) = 0.01;
            Sig0(4, 4) = 1.0;
            Sig0(5, 5) = 0.01;

            // Player Costs
            std::vector<PlayerCost> ilq_player_costs;
            std::vector<PlayerCost> ilqg_player_costs;
            // Set up costs for all players
            ilq_player_costs.emplace_back("P1");
            ilqg_player_costs.emplace_back("P1");
            auto& ilq_p1_cost = ilq_player_costs[0];
            auto& ilqg_p1_cost = ilqg_player_costs[0];

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
            ilq_p1_cost.addStateCost(p1_lane_cost);
            //ilq_p1_cost.addStateCost(p1_lane_l_cost);
            //ilq_p1_cost.addStateCost(p1_lane_r_cost);
            ilqg_p1_cost.addStateCost(p1_lane_cost);
            //ilqg_p1_cost.addStateCost(p1_lane_l_cost);
            //ilqg_p1_cost.addStateCost(p1_lane_r_cost);

            // Nominal Speed
            const auto p1_nominal_v_cost = std::make_shared<QuadraticCost>(kP1NominalVCostWeight, kP1VIdx, kP1NominalV, "NominalV");
            ilq_p1_cost.addStateCost(p1_nominal_v_cost);
            ilqg_p1_cost.addStateCost(p1_nominal_v_cost);

            // Penalize control effort
            const auto p1_omega_cost = std::make_shared<QuadraticCost>(kOmegaCostWeight, kP1OmegaIdx, 0.0, "Steering");
            const auto p1_jerk_cost = std::make_shared<QuadraticCost>(kJerkCostWeight, kP1JerkIdx, 0.0, "Jerk");
            ilq_p1_cost.addControlCost(0, p1_omega_cost);
            ilq_p1_cost.addControlCost(0, p1_jerk_cost);
            ilqg_p1_cost.addControlCost(0, p1_omega_cost);
            ilqg_p1_cost.addControlCost(0, p1_jerk_cost);

            // Constraints for Control
            const auto p1_omega_max_constraint = std::make_shared<SingleDimensionConstraint>(P1::kOmegaIdx, kMaxOmega,
                                                                                             true, "Max_Omega_Constraint");
            const auto p1_omega_min_constraint = std::make_shared<SingleDimensionConstraint>(P1::kOmegaIdx, -kMaxOmega,
                                                                                             false, "Min_Omega_Constraint");
            const auto p1_omega_max_chance_constraint = std::make_shared<SingleDimensionChanceConstraint>(prob, P1::kOmegaIdx, kMaxOmega,
                                                                                                          true, "Max_Omega_Constraint");
            const auto p1_omega_min_chance_constraint = std::make_shared<SingleDimensionChanceConstraint>(prob, P1::kOmegaIdx, -kMaxOmega,
                                                                                                          false, "Min_Omega_Constraint");
            ilq_p1_cost.addControlConstraint(0, p1_omega_max_constraint);
            ilq_p1_cost.addControlConstraint(0, p1_omega_min_constraint);
            ilqg_p1_cost.addControlChanceConstraint(0, p1_omega_max_chance_constraint);
            ilqg_p1_cost.addControlChanceConstraint(0, p1_omega_min_chance_constraint);

            // Obstacle Cost
            const double obs1_x = -1.0;
            const double obs1_y = 55.0;
            const double obs1_theta = M_PI/2;
            const double obs1_length = 4.0;
            const double obs1_width = 1.0;
            const Eigen::MatrixXd obs1_cov = Eigen::MatrixXd::Zero(2, 2);
            RectangularObstacle obs1(obs1_x, obs1_y, obs1_theta, obs1_length, obs1_width);

            const std::shared_ptr<ObstacleLinearChanceConstraint> p1_obstacle_chance_constraint(new ObstacleLinearChanceConstraint(prob, obs1,
                                                                                                                 {kP1XIdx, kP1YIdx},
                                                                                                                 "ChanceObstacleAvoidance"));
            const std::shared_ptr<ObstacleLinearConstraint> p1_obstacle_constraint(new ObstacleLinearConstraint(obs1,
                                                                                                          {kP1XIdx, kP1YIdx},
                                                                                                          "ObstacleAvoidance"));
            ilq_p1_cost.addStateConstraint(p1_obstacle_constraint);
            ilqg_p1_cost.addStateChanceConstraint(p1_obstacle_chance_constraint);

            // Input and observation covariances
            std::vector<Eigen::MatrixXd> input_covariances(dynamics->getNumPlayers());
            std::vector<Eigen::MatrixXd> observation_covariances(dynamics->getNumPlayers());
            for(unsigned int player_id =0; player_id<input_covariances.size(); ++player_id)
            {
                input_covariances[player_id] = Eigen::MatrixXd::Zero(dynamics->getUDim(player_id),
                                                                     dynamics->getUDim(player_id));
                observation_covariances[player_id] = Eigen::MatrixXd::Zero(dynamics->getXDim(),
                                                                           dynamics->getXDim());
            }

            input_covariances[0](0,0) = 0.01;
            input_covariances[0](1,1) = 0.01;

            observation_covariances[0](0,0) = 0.2;
            observation_covariances[0](1,1) = 0.2;
            observation_covariances[0](2,2) = 0.05;
            observation_covariances[0](3,3) = 0.01;
            observation_covariances[0](4,4) = 1.5;
            observation_covariances[0](5,5) = 0.01;
            observation_covariances[0] *= 0.3;

            lanes.reset(new std::vector<Polyline2>{lane, lane2});
            pos_dims.reset(new std::vector<std::vector<int>>{{kP1XIdx, kP1YIdx, kP1HeadingIdx}});
            ilq_problem.reset(new Problem(dynamics, ilq_player_costs, x0, Sig0, input_covariances, observation_covariances));
            ilqg_problem.reset(new Problem(dynamics, ilqg_player_costs, x0, Sig0, input_covariances, observation_covariances));
            obstacles.push_back(std::make_shared<RectangularObstacle>(obs1));
        }
    };
}

#endif //SRC_ONE_PLAYER_LANE_CHANGE_H
