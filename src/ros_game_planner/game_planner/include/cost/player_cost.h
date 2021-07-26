/*
 * Copyright (c) 2019, The Regents of the University of California (Regents).
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
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Container to store all the cost functions for a single player, and keep track
// of which variables (x, u1, u2, ..., uN) they correspond to.
//
///////////////////////////////////////////////////////////////////////////////
#ifndef GAME_PLANNER_PLAYER_COST_H
#define GAME_PLANNER_PLAYER_COST_H

#include "constraint/deterministic/al_constraint.h"
#include "constraint/stochastic/al_chance_constraint.h"
#include "cost/deterministic/cost.h"
#include "cost/stochastic/chance_cost.h"
#include "utils/operating_point.h"
#include "utils/quadratic_cost_approximation.h"
#include "utils/types.h"

namespace game_planner
{
    class PlayerCost
    {
    public:
        ~PlayerCost() = default;

        // Provide default values for all constructor values. If num_time_steps is
        // positive use that to initialize the lambdas.
        explicit PlayerCost(const std::string& name = "",
                            double state_regularization = 0.0,
                            double control_regularization = 0.0)
                : name_(name),
                  state_regularization_(state_regularization),
                  control_regularization_(control_regularization) {}

        // Add new state and control costs for this player.
        void addStateCost(const std::shared_ptr<Cost>& cost);
        void addControlCost(const unsigned int& idx, const std::shared_ptr<Cost>& cost);

        // Add new state and control chance costs for this player.
        void addStateChanceCost(const std::shared_ptr<ChanceCost>& cost);
        void addControlChanceCost(const unsigned int& idx, const std::shared_ptr<ChanceCost>& cost);

        // Add new state and control constraints.
        void addStateConstraint(const std::shared_ptr<ALConstraint>& constraint);
        void addControlConstraint(const unsigned int& idx,
                                  const std::shared_ptr<ALConstraint>& constraint);

        // Add new state and control chance constraints.
        void addStateChanceConstraint(const std::shared_ptr<ALChanceConstraint>& constraint);
        void addControlChanceConstraint(const unsigned int& idx,
                                        const std::shared_ptr<ALChanceConstraint>& constraint);

        // Evaluate this cost at the current time, state, and controls, or
        // integrate over an entire trajectory. The "Offset" here indicates that
        // state costs will be evaluated at the next time step.
        double evaluate(const int& time_id,
                        const Eigen::VectorXd& x,
                        const std::vector<Eigen::VectorXd>& us) const;
        double evaluate(const OperatingPoint& op, const double& time_step) const;
        double evaluate(const OperatingPoint& op) const;

        // Evaluate this deterministic cost and chance cost
        double evaluate(const int& time_id,
                        const Eigen::VectorXd& x,
                        const std::vector<Eigen::VectorXd>& us,
                        const Eigen::MatrixXd& cov,
                        const std::vector<Eigen::MatrixXd>& control_covs) const;

        // Quadraticize this cost at the given time, time step, state, and controls.
        QuadraticCostApproximation quadraticize(const int& time_id,
                                                const Eigen::VectorXd& x,
                                                const std::vector<Eigen::VectorXd>& us) const;

        QuadraticCostApproximation quadraticize(const int& time_id,
                                                const Eigen::VectorXd& x,
                                                const std::vector<Eigen::VectorXd>& us,
                                                const Eigen::MatrixXd& cov,
                                                const std::vector<Eigen::MatrixXd>& control_covs) const;

        // Return empty cost quadraticization except for control costs.
        QuadraticCostApproximation quadraticizeControlCosts(const int& time_id,
                                                            const Eigen::VectorXd& x,
                                                            const std::vector<Eigen::VectorXd>& us) const;

        // Accessors.
        const std::vector<std::shared_ptr<Cost>>& getStateCosts() const { return state_costs_; }
        const std::unordered_multimap<unsigned int, std::shared_ptr<Cost>>& getControlCosts() const { return control_costs_; }
        const std::vector<std::shared_ptr<ChanceCost>>& getStateChanceCosts() const { return state_chance_costs_; }
        const std::unordered_multimap<unsigned int, std::shared_ptr<ChanceCost>>& getControlChanceCosts() const { return control_chance_costs_; }

        const std::vector<std::shared_ptr<ALConstraint>>& getStateConstraints() const
        {
            return state_constraints_;
        }
        const std::unordered_multimap<unsigned int, std::shared_ptr<ALConstraint>>& getControlConstraints() const
        {
            return control_constraints_;
        }
        const std::vector<std::shared_ptr<ALChanceConstraint>>& getStateChanceConstraints() const
        {
            return state_chance_constraints_;
        }
        const std::unordered_multimap<unsigned int, std::shared_ptr<ALChanceConstraint>>& getControlChanceConstraints() const
        {
            return control_chance_constraints_;
        }
        bool isConstrained() const
        {
            return !state_constraints_.empty() || !control_constraints_.empty() ||
                   !state_chance_constraints_.empty() || !control_chance_constraints_.empty() ;
        }

    private:
        // Name to be used with error msgs.
        const std::string name_;

        // State costs and control costs.
        std::vector<std::shared_ptr<Cost>> state_costs_;
        std::unordered_multimap<unsigned int, std::shared_ptr<Cost>> control_costs_;

        // State chance costs and control chance costs.
        std::vector<std::shared_ptr<ChanceCost>> state_chance_costs_;
        std::unordered_multimap<unsigned int, std::shared_ptr<ChanceCost>> control_chance_costs_;

        // State and control constraints
        std::vector<std::shared_ptr<ALConstraint>> state_constraints_;
        std::unordered_multimap<unsigned int, std::shared_ptr<ALConstraint>> control_constraints_;

        // State and control chance constraints
        std::vector<std::shared_ptr<ALChanceConstraint>> state_chance_constraints_;
        std::unordered_multimap<unsigned int, std::shared_ptr<ALChanceConstraint>> control_chance_constraints_;

        // Regularization on costs.
        const double state_regularization_;
        const double control_regularization_;
    };
}

#endif //GAME_PLANNER_PLAYER_COST_H
