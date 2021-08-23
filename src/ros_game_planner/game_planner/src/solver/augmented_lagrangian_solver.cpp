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
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Solver that implements an augmented Lagrangian method. For reference on these
// methods, please refer to Chapter 17 of Nocedal and Wright or the ALTRO paper:
// https://bjack205.github.io/assets/ALTRO.pdf.
//
///////////////////////////////////////////////////////////////////////////////
//checked:1 , q

#include "solver/augmented_lagrangian_solver.h"

namespace game_planner
{
    bool AugmentedLagrangianSolver::solve(const std::vector<Strategy> &initial_strategies,
                                          const std::shared_ptr<SolverLog> &solver_log)
    {
        // 1. Solve the unconstrained problem
        bool unconstrained_success = unconstrained_solver_->solve(initial_strategies, solver_log);

        // 2. Check if the problem is constrained or unconstrained
        //    If the problem is unconstrained we will terminate the calculation here
        if (!unconstrained_solver_->getProblem().isConstrained())
        {
            std::cout << "[" << solver_name_ << "]: This problem is unconstrained" << std::endl;
            return unconstrained_success;
        }

        // 3. Start Main Loop and do computation until convergence
        double max_constraint_error = constants::kInfinity;
        while (solver_log->getNumIterates() < params_.max_al_solver_iters&&
               max_constraint_error > params_.constraint_error_tolerance)
        {
            // 4. Check the constraint and update the augmented lagrangian parameter(This can be parallelize)
            max_constraint_error = -constants::kInfinity;
            const OperatingPoint& op = solver_log->getFinalOperatingPoint();
            for (auto& pc : unconstrained_solver_->getProblem().getPlayerCosts())
            {
                for (size_t kk = 0; kk < op.xs.size(); kk++)
                {
                    const auto& x = op.xs[kk];
                    const auto& us = op.us[kk];

                    // Scale each lambda.
                    for (const auto& constraint : pc.getStateConstraints())
                    {
                        const double constraint_error = constraint->evaluate(kk, x);
                        constraint->incrementLambda(kk, constraint_error);
                        max_constraint_error = std::max(max_constraint_error, constraint_error);
                    }

                    for (const auto& pair : pc.getControlConstraints())
                    {
                        const double constraint_error = pair.second->evaluate(kk, us[pair.first]);
                        pair.second->incrementLambda(kk, constraint_error);
                        max_constraint_error = std::max(max_constraint_error, constraint_error);
                    }
                }
            }

            // 5. Scale mu. (This is done for every iteration)
            BaseALConstraint::scaleMu(params_.geometric_mu_scaling);

            // 6. Start New Computation with new parameters
            unconstrained_success = unconstrained_solver_->solve(initial_strategies, solver_log);

            if(!unconstrained_success)
                std::cout << "Calculation False and down scaling the value" << std::endl;

            // 7. If we failed then downscale all lambdas and mus for next iteration.
            if (!unconstrained_success)
            {
                for (auto& pc : unconstrained_solver_->getProblem().getPlayerCosts())
                {
                    for (const auto& constraint : pc.getStateConstraints())
                        constraint->scaleLambdas(params_.geometric_lambda_downscaling);
                    for (const auto& pair : pc.getControlConstraints())
                        pair.second->scaleLambdas(params_.geometric_lambda_downscaling);
                }

                BaseALConstraint::scaleMu(params_.geometric_mu_downscaling);
            }

        } // end while

        if(max_constraint_error > params_.constraint_error_tolerance)
            std::cout << "[" << solver_name_ << "]: Could not satisfy all constraints" << std::endl;

        // 8.Reset all multipliers.
        if (params_.reset_lambdas)
        {
            for (auto& pc : unconstrained_solver_->getProblem().getPlayerCosts())
            {
                for (const auto& constraint : pc.getStateConstraints())
                    constraint->scaleLambdas(constants::kDefaultLambda);
                for (const auto& pair : pc.getControlConstraints())
                    pair.second->scaleLambdas(constants::kDefaultLambda);
            }
        }

        if (params_.reset_mu) BaseALConstraint::getGlobalMu() = constants::kDefaultMu;

        return unconstrained_success;
    }
}
