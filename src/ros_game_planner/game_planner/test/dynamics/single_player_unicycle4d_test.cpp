#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
#include "dynamics/single_player_unicycle_4d.h"
#include "utils/types.h"

using namespace game_planner;

TEST(SinglePlayerUnicycle4D, Evaluate1)
{
    const double dt = 0.1;
    SinglePlayerUnicycle4D dynamics{};

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(dynamics.getXDim());
    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(dynamics.getUDim());

    Eigen::VectorXd x1 = dynamics.evaluate(x0, u0) * dt;
    EXPECT_NEAR(x1(0), 0.0, constants::kSmallNumber);
    EXPECT_NEAR(x1(1), 0.0, constants::kSmallNumber);
    EXPECT_NEAR(x1(2), 0.0, constants::kSmallNumber);
    EXPECT_NEAR(x1(3), 0.0, constants::kSmallNumber);
}

TEST(SinglePlayerUnicycle4D, Evaluate2)
{
    const double dt = 0.1;
    SinglePlayerUnicycle4D dynamics{};

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(dynamics.getXDim());
    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(dynamics.getUDim());
    u0(1) = 1.0;

    Eigen::VectorXd x1 = dynamics.evaluate(x0, u0) * dt;
    EXPECT_NEAR(x1(0), 0.0, constants::kSmallNumber);
    EXPECT_NEAR(x1(1), 0.0, constants::kSmallNumber);
    EXPECT_NEAR(x1(2), 0.0, constants::kSmallNumber);
    EXPECT_NEAR(x1(3), u0(1)*dt, constants::kSmallNumber);
}

TEST(SinglePlayerUnicycle4D, Linearize)
{
    const double dt = 0.1;
    SinglePlayerUnicycle4D dynamics{};

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(dynamics.getXDim());
    Eigen::VectorXd u0 = Eigen::VectorXd::Zero(dynamics.getUDim());
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dynamics.getXDim(), dynamics.getXDim());
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(dynamics.getXDim(), dynamics.getUDim());
    dynamics.linearize(dt, x0, u0, A, B);

    EXPECT_NEAR(A(0, 3), dt, constants::kSmallNumber);
    EXPECT_NEAR(B(2, 0), dt, constants::kSmallNumber);
    EXPECT_NEAR(B(3, 1), dt, constants::kSmallNumber);


    A = Eigen::MatrixXd::Identity(dynamics.getXDim(), dynamics.getXDim());
    B = Eigen::MatrixXd::Zero(dynamics.getXDim(), dynamics.getUDim());
    Eigen::VectorXd x1 = Eigen::VectorXd::Zero(dynamics.getXDim());
    Eigen::VectorXd u1 = Eigen::VectorXd::Zero(dynamics.getUDim());
    u1(1) = 1.0;
    dynamics.linearize(dt, x1, u1, A, B);

    EXPECT_NEAR(A(0, 3), dt, constants::kSmallNumber);
    EXPECT_NEAR(B(2, 0), dt, constants::kSmallNumber);
    EXPECT_NEAR(B(3, 1), dt, constants::kSmallNumber);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "single_player_unicycle_4d_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}