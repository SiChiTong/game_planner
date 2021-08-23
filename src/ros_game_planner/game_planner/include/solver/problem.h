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
// Container for dynamics, cost, initial position and operating point
//
///////////////////////////////////////////////////////////////////////////////
//checked:

#ifndef GAME_PLANNER_PROBLEM_H
#define GAME_PLANNER_PROBLEM_H

#include <memory>

#include "cost/player_cost.h"
#include "dynamics/multi_player_dynamical_system.h"
#include "utils/types.h"
#include "utils/operating_point.h"

namespace game_planner
{
    class Problem
    {
    public:
        Problem(const std::shared_ptr<MultiPlayerDynamicalSystem>& dynamics,
                const std::vector<PlayerCost>& player_cost,
                const Eigen::VectorXd& x0,
                const Eigen::MatrixXd& Sig0,
                const std::vector<Eigen::MatrixXd>& input_covs,
                const std::vector<Eigen::MatrixXd>& observation_covs)
                : dynamics_(dynamics), player_costs_(player_cost),
                  x0_(x0), Sig0_(Sig0),
                  input_covs_(input_covs), observation_covs_(observation_covs)
        {
        }

        // Accessors.
        bool isConstrained() const;

        const Eigen::VectorXd& getInitialState() const { return x0_; }

        const Eigen::MatrixXd& getInitialCovariance() const { return Sig0_; }
        const Eigen::MatrixXd& getInputCovariance(const unsigned int& player_id) const { return input_covs_[player_id]; }
        const Eigen::MatrixXd& getObservationCovariance(const unsigned int& player_id) const { return observation_covs_[player_id]; }
        const std::vector<Eigen::MatrixXd>& getInputCovariance() const { return input_covs_; }
        const std::vector<Eigen::MatrixXd>& getObservationCovariance() const { return observation_covs_; }

        std::vector<PlayerCost>& getPlayerCosts() { return player_costs_; }

        PlayerCost& getPlayerCosts(const unsigned int& i) { assert(i<player_costs_.size()); return player_costs_[i]; }

        const std::vector<PlayerCost>& getPlayerCosts() const { return player_costs_; }

        const PlayerCost& getPlayerCosts(const unsigned int& i) const { assert(i<player_costs_.size()); return player_costs_[i]; }

        const std::shared_ptr<const MultiPlayerDynamicalSystem>& getDynamics() const { return dynamics_; }

        unsigned int getNumPlayers() const { return dynamics_->getNumPlayers(); }

        int getXDim() const { return dynamics_->getXDim(); }

        int getUDim(const unsigned int& i) { return dynamics_->getUDim(i); }

        std::vector<int> getUDimVec() const { return dynamics_->getUDimVec(); }

        OperatingPoint initializeOperatingPoint() const
        {
            // 1. Create Operating Point
            OperatingPoint operating_point(time::kNumTimeSteps, getXDim(), getUDimVec());

            // 2. Set Initial State
            operating_point.xs[0] = getInitialState();

            return operating_point;
        }

    private:
        // Dynamics of the problem
        std::shared_ptr<const MultiPlayerDynamicalSystem> dynamics_;

        // Player Costs
        std::vector<PlayerCost> player_costs_;

        // Initial Position
        const Eigen::VectorXd x0_;

        // Initial Covariance
        const Eigen::MatrixXd Sig0_;

        //Question: What are the input covariance and observation covariacne?
        // Input Covariance
        const std::vector<Eigen::MatrixXd> input_covs_;

        // Observation Covariance(Assume we can observe all states)
        const std::vector<Eigen::MatrixXd> observation_covs_;
    };
}

#endif //GAME_PLANNER_PROBLEM_H
