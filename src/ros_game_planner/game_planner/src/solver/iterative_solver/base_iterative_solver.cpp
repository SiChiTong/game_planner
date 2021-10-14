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
 * Authors:  Yutaka Shimizu
 */
///////////////////////////////////////////////////////////////////////////////
//
// Base Class For Iterative Solver
//
///////////////////////////////////////////////////////////////////////////////
//checked: 2, q

#include "solver/iterative_solver/base_iterative_solver.h"

namespace game_planner
{
    OperatingPoint BaseIterativeSolver::forward(const OperatingPoint &last_operating_point,
                                                const std::vector<Strategy> &current_strategies)
    {
        OperatingPoint operating_point = last_operating_point;

        // Integrate dynamics and populate operating point, one time step at a time.
        // x_0 is a given prior
        Eigen::VectorXd x(last_operating_point.xs[0]);
        for (size_t kk = 0; kk < time::kNumTimeSteps; kk++)
        {
            // Unpack.
            const Eigen::VectorXd delta_x = x - last_operating_point.xs[kk];
            const auto& last_us = last_operating_point.us[kk];
            auto& current_us = operating_point.us[kk];

            // Record state.
            operating_point.xs[kk] = x;

            // Compute and record control for each player.
            
            for (unsigned int jj = 0; jj < problem_->getNumPlayers(); jj++)
            {
                const auto& strategy = current_strategies[jj];
                
                current_us[jj] = strategy(kk, delta_x, last_us[jj]);
            }

            // Integrate dynamics for one time step(dt).
            if (kk < time::kNumTimeSteps - 1)
                x = problem_->getDynamics()->integrate(time::kTimeStep, x, current_us);
        }

        return operating_point;
    }

    std::vector<LinearDynamics> BaseIterativeSolver::computeLinearization(const OperatingPoint& op)
    {
        std::vector<LinearDynamics> linearized_dynamics(op.xs.size());
        computeLinearization(op, linearized_dynamics);

        return linearized_dynamics;
    }

    void BaseIterativeSolver::computeLinearization(const OperatingPoint& op, std::vector<LinearDynamics>& linearized_dynamics)
    {
        if(linearized_dynamics.size()!=op.xs.size())
            linearized_dynamics.resize(op.xs.size());

        // Cast dynamics to appropriate type.
        const auto dyn = static_cast<const MultiPlayerDynamicalSystem*>(problem_->getDynamics().get());

        // Populate one timestep at a time.
        for (size_t kk = 0; kk < op.xs.size(); kk++)
            linearized_dynamics[kk] = dyn->linearize(time::kTimeStep, op.xs[kk], op.us[kk]);
    }

    bool BaseIterativeSolver::checkArmijoCondition(const double& current_merit_function_value,
                                         const double& current_stepsize,
                                         const double& expected_decrease) const
    {
        // Adjust total expected decrease.
        const double scaled_expected_decrease = solver_params_.expected_decrease_fraction * current_stepsize * expected_decrease;
        return (last_merit_function_value_ - current_merit_function_value >= scaled_expected_decrease);
    }


    //Question: I don't understand this function.
    double BaseIterativeSolver::computeExpectedDecrease(const std::vector<Strategy>& strategies,
                                                        const std::vector<Eigen::VectorXd>& delta_xs,
                                                        const std::vector<std::vector<Eigen::VectorXd>>& costates,
                                                        const std::vector<LinearDynamics>& linearization,
                                                        const std::vector<std::vector<QuadraticCostApproximation>>& cost_quadraticization) const
    {
        assert(linearization.size() == time::kNumTimeSteps);
        assert(cost_quadraticization.size() == time::kNumTimeSteps);
        assert(cost_quadraticization[0].size() == problem_->getNumPlayers());

        double expected_decrease = 0.0;
        for (size_t kk = 0; kk < time::kNumTimeSteps; kk++)
        {
            const auto& lin = linearization[kk];

            // Separate x expected decrease per step at each time (saves computation).
            const size_t xdim = problem_->getXDim();
            Eigen::VectorXd expected_decrease_x = Eigen::VectorXd::Zero(xdim);

            for (unsigned int ii = 0; ii < problem_->getNumPlayers(); ii++)
            {
                const auto& quad = cost_quadraticization[kk][ii];
                const auto& costate = costates[kk][ii];
                const auto& neg_ui = strategies[ii].alphas[kk];  // NOTE: could also evaluate delta u on
                // delta x to be more precise.
                

                //Question: what is the meaning of right hand side?
                // Handle control contribution.
                expected_decrease -= neg_ui.transpose() * quad.control_.at(ii).hess_ * (quad.control_.at(ii).grad_ - lin.Bs[ii].transpose() * costate);
            }
        }
        return expected_decrease;
    }


    //Question: What is merit function here?
    double BaseIterativeSolver::computeMeritFunction(const OperatingPoint& current_op,
                                                     const std::vector<std::vector<QuadraticCostApproximation>>& cost_quadraticization)
    {
        assert(cost_quadraticization.size() == time::kNumTimeSteps);
        assert(cost_quadraticization[0].size() == problem_->getNumPlayers());

        // Now, accumulate cost gradients (presuming that this operating point is
        // dynamically feasible so dynamic constraints are all zero).
        double merit = 0.0;
        for (size_t kk = 0; kk < cost_quadraticization.size(); kk++)
        {
            for (unsigned int ii = 0; ii < problem_->getNumPlayers(); ii++)
            {    //Question: why we add the norms of gradients?
                const auto& quad = cost_quadraticization[kk][ii];
                merit += quad.control_.at(ii).grad_.squaredNorm();

                // Don't accumulate state derivs at t0 since x0 can't change.
                //Question: Should we use the chain rule, to calculate the gradient of state w.r.t the control?
                if (kk > 0)
                    merit += quad.state_.grad_.squaredNorm();
            }
        }

        return 0.5 * merit;
    }
}