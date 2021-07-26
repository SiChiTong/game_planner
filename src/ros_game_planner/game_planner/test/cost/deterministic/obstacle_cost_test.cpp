#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
#include "cost/deterministic/obstacle_cost.h"
#include "utils/types.h"

using namespace game_planner;

TEST(OBSTACLE_COST, EVALUATE)
{
    const double weight = 1.0;
    const double obs_x = 1.0;
    const double obs_y = 1.0;
    const double obs_theta = 0.0;
    const double obs_radius = 3.0;
    const Eigen::MatrixXd obs_cov = Eigen::MatrixXd::Zero(2, 2);
    CircularObstacle obs(obs_x, obs_y, obs_theta, obs_cov, obs_radius);
    const std::pair<int, int> idxs = {0, 1};
    const double threshold1 = 1.0;
    const std::string name = "obstacle_cost";
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);

    ObstacleCost obs_cost1(weight, obs, idxs, threshold1, name);
    double cost1 = obs_cost1.evaluate(x0);
    EXPECT_NEAR(cost1, 0.0, constants::kSmallNumber);

    Eigen::VectorXd x1(2);
    x1(0) = 0.5;
    x1(1) = 0.5;
    double cost2 = obs_cost1.evaluate(x1);
    std::cout << cost2 << std::endl;
    double dx2 = x1(0) - obs_x;
    double dy2 = x1(1) - obs_y;
    EXPECT_NEAR(cost2, 0.5*weight*std::pow(threshold1-std::sqrt(dx2*dx2+dy2*dy2), 2), constants::kSmallNumber);

    const double weight2 = 10.0;
    const double threshold2 = 3.0;
    ObstacleCost obs_cost2(weight2, obs, idxs, threshold2, name);
    double cost3 = obs_cost2.evaluate(x0);
    EXPECT_NEAR(cost3, 0.5*weight2*std::pow((3-std::sqrt(2)), 2), constants::kSmallNumber);
}

TEST(OBSTACLE_COST, QUADRATICIZE1)
{
    const double weight = 1.0;
    const double obs_x = 1.0;
    const double obs_y = 1.0;
    const double obs_theta = 0.0;
    const double obs_radius = 3.0;
    const Eigen::MatrixXd obs_cov = Eigen::MatrixXd::Zero(2, 2);
    CircularObstacle obs(obs_x, obs_y, obs_theta, obs_cov, obs_radius);
    const std::pair<int, int> idxs = {0, 1};
    const double threshold1 = 1.0;
    const std::string name = "obstacle_cost";
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);

    ObstacleCost obs_cost1(weight, obs, idxs, threshold1, name);
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd hess = Eigen::MatrixXd::Zero(2,2);
    obs_cost1.quadraticize(x0, hess, grad);

    EXPECT_NEAR(grad(0), 0.0, constants::kSmallNumber);
    for(int i=0; i<2; ++i)
    {
        EXPECT_NEAR(grad(i), 0.0, constants::kSmallNumber);
        for(int j=0; j<2; ++j)
            EXPECT_NEAR(hess(i,j), 0.0, constants::kSmallNumber);
    }
}

TEST(OBSTACLE_COST, QUADRATICIZE2)
{
    const double weight = 1.0;
    const double obs_x = 1.0;
    const double obs_y = 1.0;
    const double obs_theta = 0.0;
    const double obs_radius = 3.0;
    const Eigen::MatrixXd obs_cov = Eigen::MatrixXd::Zero(2, 2);
    CircularObstacle obs(obs_x, obs_y, obs_theta, obs_cov, obs_radius);
    const std::pair<int, int> idxs = {0, 1};
    const double threshold1 = 1.0;
    const std::string name = "obstacle_cost";
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(2);
    x0(0) = 0.5;
    x0(1) = 0.5;

    ObstacleCost obs_cost1(weight, obs, idxs, threshold1, name);
    Eigen::VectorXd grad = Eigen::VectorXd::Zero(2);
    Eigen::MatrixXd hess = Eigen::MatrixXd::Zero(2,2);
    obs_cost1.quadraticize(x0, hess, grad);

    const double dx = x0(0) - obs_x;
    const double dy = x0(1) - obs_y;
    const double delta = std::sqrt(dx*dx+dy*dy);
    const double gap = threshold1 - delta;
    EXPECT_NEAR(grad(0), -weight*gap*dx/delta, constants::kSmallNumber);
    EXPECT_NEAR(grad(1), -weight*gap*dy/delta, constants::kSmallNumber);
    EXPECT_NEAR(hess(0,0), hess(1,1), constants::kSmallNumber);
    EXPECT_NEAR(hess(1,0), hess(0,1), constants::kSmallNumber);
    EXPECT_NEAR(hess(0,0), 0.292893, constants::kSmallNumber);
    EXPECT_NEAR(hess(1,0), 0.707107, constants::kSmallNumber);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "quadratic_cost_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}