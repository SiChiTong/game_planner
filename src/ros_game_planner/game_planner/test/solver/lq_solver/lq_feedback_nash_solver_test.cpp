#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
#include <memory>
#include "dynamics/multi_player_dynamical_system.h"
#include "dynamics/single_player_unicycle_4d.h"
#include "cost/player_cost.h"
#include "cost/deterministic/quadratic_polyline2_cost.h"
#include "cost/deterministic/quadratic_cost.h"
#include "solver/lq_solver/lq_feedback_nash_solver.h"

using namespace game_planner;

TEST(LQ_FEEDBACK_NASH_SOLVER, SOLVE)
{
    // Parameters
    const double dt = 0.1;
    const size_t num_time_steps = 2;

    /*************************************
     * 1. Dynamics
     *************************************/
    using P1 = SinglePlayerUnicycle4D;
    using P2 = SinglePlayerUnicycle4D;
    std::shared_ptr<MultiPlayerDynamicalSystem> multi_dynamics;
    multi_dynamics.reset(new MultiPlayerDynamicalSystem({std::make_shared<P1>(), std::make_shared<P2>()}));

    Eigen::VectorXd x1 = Eigen::VectorXd::Zero(multi_dynamics->getXDim());
    std::vector<Eigen::VectorXd> us1(2, Eigen::VectorXd::Zero(2));
    us1[0](1) = 0.0;
    us1[1](1) = 0.0;

    Eigen::VectorXd x2 = Eigen::VectorXd::Zero(multi_dynamics->getXDim());
    std::vector<Eigen::VectorXd> us2(2, Eigen::VectorXd::Zero(2));
    us2[0](1) = 1.0;
    us2[1](1) = 1.0;

    LinearDynamics linear_dynamics1 = multi_dynamics->linearize(dt, x1, us1);
    LinearDynamics linear_dynamics2 = multi_dynamics->linearize(dt, x2, us2);

    /*************************************
     * 2. Player Cost
     *************************************/
    PlayerCost player_cost;
    player_cost.addStateCost(std::make_shared<QuadraticCost>(1.0, -1));
    player_cost.addControlCost(0, std::make_shared<QuadraticCost>(1.0, -1));
    player_cost.addControlCost(1, std::make_shared<QuadraticCost>(1.0, -1));

    QuadraticCostApproximation quad_cost01 = player_cost.quadraticize(0, x1, us1);
    QuadraticCostApproximation quad_cost02 = player_cost.quadraticize(0, x2, us1);
    QuadraticCostApproximation quad_cost11 = player_cost.quadraticize(1, x1, us2);
    QuadraticCostApproximation quad_cost12 = player_cost.quadraticize(1, x2, us2);

    /*************************************
    * 3. Solve
    *************************************/
    std::vector<LinearDynamics> linear_dynamics = {linear_dynamics1, linear_dynamics2};
    std::vector<std::vector<QuadraticCostApproximation>> quad_costs = {{quad_cost01, quad_cost02}, {quad_cost11, quad_cost12}};
    LQFeedbackNashSolver solver(multi_dynamics->getXDim(), multi_dynamics->getUDimVec(), num_time_steps);
    std::vector<Strategy> strategies = solver.solve(linear_dynamics, quad_costs, x1);

    // strategies store a strategy for a single player
    for(auto& strategy : strategies)
    {
        std::cout << strategy.Ps.size() << std::endl;
    }
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "lq_feedback_nash_solver_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}