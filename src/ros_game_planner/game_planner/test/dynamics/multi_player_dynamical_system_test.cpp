#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
#include <memory>
#include "dynamics/multi_player_dynamical_system.h"
#include "dynamics/single_player_unicycle_4d.h"

using namespace game_planner;

TEST(MULTI_PLAYER_DYNAMICAL_SYSTEM, EVALUATE1)
{
    std::shared_ptr<SinglePlayerDynamicalSystem> p1_dynamics = std::make_shared<SinglePlayerUnicycle4D>();
    std::shared_ptr<SinglePlayerDynamicalSystem> p2_dynamics = std::make_shared<SinglePlayerUnicycle4D>();
    std::vector<std::shared_ptr<SinglePlayerDynamicalSystem>> dynamics_vec = {p1_dynamics, p2_dynamics};
    std::shared_ptr<MultiPlayerDynamicalSystem> multi_dynamics;
    multi_dynamics.reset(new MultiPlayerDynamicalSystem(dynamics_vec));

    EXPECT_EQ(multi_dynamics->getXDim(), 8);
    EXPECT_EQ(multi_dynamics->getUDim(0), 2);
    EXPECT_EQ(multi_dynamics->getUDim(1), 2);


    const double dt = 0.1;
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(multi_dynamics->getXDim());
    std::vector<Eigen::VectorXd> us(2, Eigen::VectorXd::Zero(2));
    Eigen::VectorXd x1 = multi_dynamics->evaluate(x0, us) * dt;

    for(int i=0; i<multi_dynamics->getXDim(); ++i)
        EXPECT_NEAR(x0(i), 0.0, constants::kSmallNumber);
}

TEST(MULTI_PLAYER_DYNAMICAL_SYSTEM, EVALUATE2)
{
    using P1 = SinglePlayerUnicycle4D;
    using P2 = SinglePlayerUnicycle4D;
    std::shared_ptr<MultiPlayerDynamicalSystem> multi_dynamics;
    multi_dynamics.reset(new MultiPlayerDynamicalSystem({std::make_shared<P1>(), std::make_shared<P2>()}));

    const double dt = 0.1;
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(multi_dynamics->getXDim());
    std::vector<Eigen::VectorXd> us(2, Eigen::VectorXd::Zero(2));
    us[0](1) = 1.0;
    us[1](1) = 1.0;
    Eigen::VectorXd x1 = multi_dynamics->evaluate(x0, us) * dt;

    EXPECT_NEAR(x1(3), us[0](1)*dt, constants::kSmallNumber);
    EXPECT_NEAR(x1(7), us[0](1)*dt, constants::kSmallNumber);
}

TEST(MULTI_PLAYER_DYNAMICAL_SYSTEM, LINEARIZE)
{
    using P1 = SinglePlayerUnicycle4D;
    using P2 = SinglePlayerUnicycle4D;
    std::shared_ptr<MultiPlayerDynamicalSystem> multi_dynamics;
    multi_dynamics.reset(new MultiPlayerDynamicalSystem({std::make_shared<P1>(), std::make_shared<P2>()}));

    const double dt = 0.1;
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(multi_dynamics->getXDim());
    std::vector<Eigen::VectorXd> us(2, Eigen::VectorXd::Zero(2));
    us[0](1) = 1.0;
    us[1](1) = 1.0;

    LinearDynamics result = multi_dynamics->linearize(dt, x0, us);

    EXPECT_NEAR(result.A(0, 3), dt, constants::kSmallNumber);
    EXPECT_NEAR(result.A(4, 7), dt, constants::kSmallNumber);
    EXPECT_NEAR(result.Bs[0](2, 0), dt, constants::kSmallNumber);
    EXPECT_NEAR(result.Bs[0](3, 1), dt, constants::kSmallNumber);
    EXPECT_NEAR(result.Bs[1](6, 0), dt, constants::kSmallNumber);
    EXPECT_NEAR(result.Bs[1](7, 1), dt, constants::kSmallNumber);
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "multi_dynamical_system_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}