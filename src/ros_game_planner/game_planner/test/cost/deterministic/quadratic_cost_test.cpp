#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
#include "cost/deterministic/quadratic_cost.h"
#include "utils/types.h"

using namespace game_planner;

TEST(QuadraticCost, Evaluate1)
{
    const double weight = 1.0;
    const int dim = 0;
    const double nominal = 0.0;
    const std::string name = "quadratic";

    QuadraticCost quad_cost(weight, dim, nominal, name);

    EXPECT_EQ(quad_cost.getName(), name);

    Eigen::VectorXd input1 = Eigen::VectorXd::Constant(2, 3.0);
    EXPECT_NEAR(quad_cost.evaluate(input1), 4.5, constants::kSmallNumber);

    Eigen::VectorXd input2 = Eigen::VectorXd::Constant(2, 0.0);
    EXPECT_NEAR(quad_cost.evaluate(input2), 0.0, constants::kSmallNumber);

    Eigen::VectorXd input3 = Eigen::VectorXd::Constant(2, -3.0);
    EXPECT_NEAR(quad_cost.evaluate(input3), 4.5, constants::kSmallNumber);
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "quadratic_cost_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
