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

#include "cost/player_cost.h"

namespace game_planner
{
    namespace
    {
        // Accumulate control costs and constraints into the given quadratic
        // approximation.
        template <typename T, typename F>
        void AccumulateControlCostsBase(const std::unordered_multimap<unsigned int, std::shared_ptr<T>>& costs,
                                        const int& time_id,
                                        const std::vector<Eigen::VectorXd>& us,
                                        const double& regularization,
                                        QuadraticCostApproximation* q,
                                        F f)
        {
            size_t cost_idx = 0;
            for (const auto &pair : costs) //for each player
            {
                const unsigned int player = pair.first;
                const auto &cost = pair.second;

                // If we haven't seen this player yet, initialize R and r to zero.
                auto iter = q->control_.find(player);
                if (iter == q->control_.end())
                {
                    auto inserted_pair = q->control_.emplace(
                            player, SingleCostApproximation(us[player].size(), regularization));

                    // Second element should be true because we definitely won't have any
                    // key collisions.
                    assert(inserted_pair.second);

                    // Update iter to point to where the new R was inserted.
                    iter = inserted_pair.first;
                }

                f(*cost, time_id, us[player], iter->second.hess_, iter->second.grad_);
                cost_idx++;
            }
        }

        //For Control Cost
        void AccumulateControlCosts(const std::unordered_multimap<unsigned int, std::shared_ptr<Cost>>& costs,
                                    const int& time_id,
                                    const std::vector<Eigen::VectorXd>& us,
                                    const double& regularization,
                                    QuadraticCostApproximation* q)
        {
            auto f = [](const Cost& cost,
                        const int& time_id,
                        const Eigen::VectorXd& u,
                        Eigen::MatrixXd& hess,
                        Eigen::VectorXd& grad)
            {
                cost.quadraticize(time_id, u, hess, grad);
            };

            AccumulateControlCostsBase(costs, time_id, us, regularization, q, f);
        }

        //For Constraint Cost
        void AccumulateControlConstraints(const std::unordered_multimap<unsigned int, std::shared_ptr<ALConstraint>>& constraints,
                                          const int& time_id,
                                          const std::vector<Eigen::VectorXd>& us,
                                          const double& regularization,
                                          QuadraticCostApproximation* q)
        {
            auto f = [](const ALConstraint& constraint,
                        const int& time_id,
                        const Eigen::VectorXd& u, // Input
                        Eigen::MatrixXd& hess,
                        Eigen::VectorXd& grad)
            {
                constraint.quadraticize(time_id, u, hess, grad);
            };

            AccumulateControlCostsBase(constraints, time_id, us, regularization, q, f);
        }

        // Accumulate control chance costs and constraints into the given quadratic
        // approximation.
        template <typename T, typename F>
        void AccumulateControlChanceCostsBase(const std::unordered_multimap<unsigned int, std::shared_ptr<T>>& costs,
                                              const int& time_id,
                                              const std::vector<Eigen::VectorXd>& us,
                                              const double& regularization,
                                              const std::vector<Eigen::MatrixXd>& control_covs,
                                              QuadraticCostApproximation* q,
                                              F f)
        {
            size_t cost_idx = 0;
            for (const auto &pair : costs) //for each player
            {
                const unsigned int player = pair.first;
                const auto &cost = pair.second;

                // If we haven't seen this player yet, initialize R and r to zero.
                auto iter = q->control_.find(player);
                if (iter == q->control_.end())
                {
                    auto inserted_pair = q->control_.emplace(
                            player, SingleCostApproximation(us[player].size(), regularization));

                    // Second element should be true because we definitely won't have any
                    // key collisions.
                    assert(inserted_pair.second);

                    // Update iter to point to where the new R was inserted.
                    iter = inserted_pair.first;
                }

                f(*cost, time_id, us[player], control_covs[player], iter->second.hess_, iter->second.grad_);
                cost_idx++;
            }
        }

        //For Control Chance Cost
        void AccumulateControlChanceCosts(const std::unordered_multimap<unsigned int, std::shared_ptr<ChanceCost>>& costs,
                                          const int& time_id,
                                          const std::vector<Eigen::VectorXd>& us,
                                          const double& regularization,
                                          const std::vector<Eigen::MatrixXd>& control_covs,
                                          QuadraticCostApproximation* q)
        {
            auto f = [](const ChanceCost& cost,
                        const int& time_id,
                        const Eigen::VectorXd& u,
                        const Eigen::MatrixXd& control_cov,
                        Eigen::MatrixXd& hess,
                        Eigen::VectorXd& grad)
            {
                cost.quadraticize(time_id, u, control_cov, hess, grad);
            };

            AccumulateControlChanceCostsBase(costs, time_id, us, regularization, control_covs, q, f);
        }

        //For Constraint Cost
        void AccumulateControlChanceConstraints(const std::unordered_multimap<unsigned int, std::shared_ptr<ALChanceConstraint>>& constraints,
                                                const int& time_id,
                                                const std::vector<Eigen::VectorXd>& us,
                                                const double& regularization,
                                                const std::vector<Eigen::MatrixXd>& control_covs,
                                                QuadraticCostApproximation* q)
        {
            auto f = [](const ALChanceConstraint& constraint,
                        const int& time_id,
                        const Eigen::VectorXd& u, // Input
                        const Eigen::MatrixXd& control_cov,
                        Eigen::MatrixXd& hess,
                        Eigen::VectorXd& grad)
            {
                constraint.quadraticize(time_id, u, control_cov, hess, grad);
            };

            AccumulateControlChanceCostsBase(constraints, time_id, us, regularization, control_covs, q, f);
        }
    }

    void PlayerCost::addStateCost(const std::shared_ptr<Cost>& cost)
    {
        state_costs_.emplace_back(cost);
    }

    void PlayerCost::addControlCost(const unsigned int& idx,
                                    const std::shared_ptr<Cost>& cost)
    {
        control_costs_.emplace(idx, cost);
    }

    void PlayerCost::addStateChanceCost(const std::shared_ptr<ChanceCost>& cost)
    {
        state_chance_costs_.emplace_back(cost);
    }

    void PlayerCost::addControlChanceCost(const unsigned int& idx,
                                          const std::shared_ptr<ChanceCost>& cost)
    {
        control_chance_costs_.emplace(idx, cost);
    }

    void PlayerCost::addStateConstraint(const std::shared_ptr<ALConstraint>& constraint)
    {
        state_constraints_.emplace_back(constraint);
    }

    void PlayerCost::addControlConstraint(const unsigned int& idx,
                                          const std::shared_ptr<ALConstraint>& constraint)
    {
        control_constraints_.emplace(idx, constraint);
    }

    void PlayerCost::addStateChanceConstraint(const std::shared_ptr<ALChanceConstraint>& constraint)
    {
        state_chance_constraints_.emplace_back(constraint);
    }

    void PlayerCost::addControlChanceConstraint(const unsigned int& idx,
                                                const std::shared_ptr<ALChanceConstraint>& constraint)
    {
        control_chance_constraints_.emplace(idx, constraint);
    }

    double PlayerCost::evaluate(const int& time_id,
                                const Eigen::VectorXd& x,
                                const std::vector<Eigen::VectorXd>& us) const
    {
        double total_cost = 0.0;

        // State costs.
        for (const auto& cost : state_costs_) total_cost += cost->evaluate(time_id, x);

        // Control costs.
        for (const auto& pair : control_costs_)
        {
            const unsigned int& player = pair.first;
            const auto& cost = pair.second;

            total_cost += cost->evaluate(time_id, us[player]);
        }

        return total_cost;
    }

    double PlayerCost::evaluate(const OperatingPoint& op, const double& time_step) const
    {
        double cost = 0.0;

        for (size_t kk = 0; kk < op.xs.size(); kk++)
        {
            const double instantaneous_cost = evaluate(kk, op.xs[kk], op.us[kk]);
            cost += instantaneous_cost;
        }

        return cost;
    }

    double PlayerCost::evaluate(const OperatingPoint& op) const
    {
        double total_cost = 0.0;
        for (size_t kk = 0; kk < op.xs.size(); kk++) total_cost += evaluate(op, kk);

        return total_cost;
    }

    double PlayerCost::evaluate(const int& time_id,
                                const Eigen::VectorXd& x,
                                const std::vector<Eigen::VectorXd>& us,
                                const Eigen::MatrixXd& cov,
                                const std::vector<Eigen::MatrixXd>& control_covs) const
    {
        assert(x.size() == cov.rows());
        assert(x.size() == cov.cols());
        assert(control_covs.size() == us.size());
        double total_cost = 0.0;

        // evaluate deterministic cost
        total_cost += evaluate(time_id, x, us);

        // evaluate chance cost
        // State chance costs.
        for (const auto& cost : state_chance_costs_) total_cost += cost->evaluate(time_id, x, cov);

        // Control chance costs.
        for (const auto& pair : control_chance_costs_)
        {
            const unsigned int& player = pair.first;
            const auto& cost = pair.second;

            total_cost += cost->evaluate(time_id, us[player], control_covs[player]);
        }

        return total_cost;
    }

    QuadraticCostApproximation PlayerCost::quadraticize(const int& time_id,
                                                        const Eigen::VectorXd& x,
                                                        const std::vector<Eigen::VectorXd>& us) const
    {
        QuadraticCostApproximation q(x.size(), state_regularization_);

        // Accumulate state costs.
        for (const auto& cost : state_costs_)
            cost->quadraticize(time_id, x, q.state_.hess_, q.state_.grad_);

        // Accumulate control costs.
        AccumulateControlCosts(control_costs_, time_id, us, control_regularization_, &q);

        // Accumulate state constraints (including augmented Lagrangian terms scaled
        // by appropriate multipliers).
        for (const auto& constraint : state_constraints_)
            constraint->quadraticize(time_id, x, q.state_.hess_, q.state_.grad_);

        // Accumulate control constraints.
        AccumulateControlConstraints(control_constraints_, time_id, us, control_regularization_, &q);

        return q;
    }

    QuadraticCostApproximation PlayerCost::quadraticize(const int& time_id,
                                                        const Eigen::VectorXd& x,
                                                        const std::vector<Eigen::VectorXd>& us,
                                                        const Eigen::MatrixXd& cov,
                                                        const std::vector<Eigen::MatrixXd>& control_covs) const
    {
        assert(us.size()==control_covs.size());
        // Calculate Deterministic Quadraticization
        QuadraticCostApproximation q = quadraticize(time_id, x, us);

        // Calculate Stochastic Quadraticization
        // Accumulate state costs.
        for (const auto& cost : state_chance_costs_)
            cost->quadraticize(time_id, x, cov, q.state_.hess_, q.state_.grad_);

        // Accumulate control costs.
        AccumulateControlChanceCosts(control_chance_costs_, time_id, us, control_regularization_, control_covs, &q);

        // Accumulate state constraints (including augmented Lagrangian terms scaled
        // by appropriate multipliers).
        for (const auto& constraint : state_chance_constraints_)
            constraint->quadraticize(time_id, x, cov, q.state_.hess_, q.state_.grad_);

        // Accumulate control constraints.
        AccumulateControlChanceConstraints(control_chance_constraints_, time_id, us, control_regularization_, control_covs, &q);

        return q;
    }

    QuadraticCostApproximation PlayerCost::quadraticizeControlCosts(const int& time_id,
                                                                    const Eigen::VectorXd& x,
                                                                    const std::vector<Eigen::VectorXd>& us) const
    {
        QuadraticCostApproximation q(x.size(), state_regularization_);

        // Accumulate control costs.
        AccumulateControlCosts(control_costs_, time_id, us, control_regularization_, &q);

        return q;
    }

}