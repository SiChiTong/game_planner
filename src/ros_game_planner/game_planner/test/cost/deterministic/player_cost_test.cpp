#include <gtest/gtest.h>
#include <ros/ros.h>
#include <iostream>
#include "cost/player_cost.h"
#include "cost/deterministic/quadratic_polyline2_cost.h"
#include "cost/deterministic/quadratic_cost.h"
#include "utils/types.h"

using namespace game_planner;

//Parameters
static constexpr double kCostWeight = 1.0;
static constexpr size_t kNumPlayers = 2;
static constexpr int kVectorDimension = 5;

class PlayerCostTest : public ::testing::Test
{
protected:
    void SetUp()
    {
        player_cost_.addStateCost(std::make_shared<QuadraticCost>(kCostWeight, -1));
        player_cost_.addControlCost(0, std::make_shared<QuadraticCost>(kCostWeight, -1));
        player_cost_.addControlCost(1, std::make_shared<QuadraticCost>(kCostWeight, -1));

        // Choose a random state and controls.
        x_ = Eigen::VectorXd::Random(kVectorDimension);
        for (size_t ii = 0; ii < kNumPlayers; ii++)
            us_.emplace_back(Eigen::VectorXd::Random(kVectorDimension));
    }

    // PlayerCost, state, and controls for each player.
    PlayerCost player_cost_;
    Eigen::VectorXd x_;
    std::vector<Eigen::VectorXd> us_;
};

// Test that we evaluate correctly.
TEST_F(PlayerCostTest, EvaluateWorks)
{
    const double value = player_cost_.evaluate(0, x_, us_);
    const double expected = 0.5 * (x_.squaredNorm() + std::accumulate(us_.begin(), us_.end(), 0.0,
                                                                      [](float total, const Eigen::VectorXd& item) {
                                                                          return total + item.squaredNorm();}
    ));

    EXPECT_NEAR(value, expected, constants::kSmallNumber);
}

// Check that we quadraticize correctly when dimension >= 0.
TEST_F(PlayerCostTest, QuadraticizeWorks)
{
    const QuadraticCostApproximation quad = player_cost_.quadraticize(0, x_, us_);

    // Check state Hessian is just kCostWeight on the diagonal.
    EXPECT_TRUE(quad.state_.hess_.diagonal().isApprox(Eigen::VectorXd::Constant(kVectorDimension, kCostWeight),
                                                      constants::kSmallNumber));
    EXPECT_NEAR(quad.state_.hess_.norm(),
                kCostWeight * std::sqrt(static_cast<double>(kVectorDimension)),
                constants::kSmallNumber);


    // Check state gradient.
    EXPECT_TRUE(quad.state_.grad_.isApprox(kCostWeight * x_, constants::kSmallNumber));

    // Check control Hessians.
    for (const auto& pair : quad.control_)
    {
        const auto& R = pair.second.hess_;
        EXPECT_TRUE(
                R.diagonal().isApprox(Eigen::VectorXd::Constant(kVectorDimension, kCostWeight),
                                      constants::kSmallNumber));
        EXPECT_NEAR(R.norm(),
                    kCostWeight * std::sqrt(static_cast<float>(kVectorDimension)),
                    constants::kSmallNumber);
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "player_cost_tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
