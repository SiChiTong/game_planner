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
// Class For ILQR Solver
//
///////////////////////////////////////////////////////////////////////////////
#include "solver/iterative_solver/ilq_solver.h"

namespace game_planner
{
    bool ILQSolver::solve(const std::vector<Strategy>& initial_strategies,
                          const std::shared_ptr<SolverLog>& solver_log)
    {
        // Things to keep track of during each iteration.
        size_t num_iterations = 0;
        bool has_converged = false;
        double elapsed = 0.0;

        // 1. Obtain Initial Operating Point (Initial xs[0] is already set)
        OperatingPoint current_operating_point = problem_->initializeOperatingPoint();
        Eigen::VectorXd x0 = problem_->getInitialState();

        // 2. Forward(calculate xs from time 1 to the end)
        current_operating_point = forward(current_operating_point, initial_strategies);

        // 3. Compute total costs for initial trajectory
        std::vector<double> total_costs = computeTotalCosts(current_operating_point);

        // 4. Logging initial trajectory
        solver_log->addSolverIterate(current_operating_point, initial_strategies, total_costs, elapsed, has_converged);

        // Maintain delta_xs and costates for linesearch.
        std::vector<Eigen::VectorXd> delta_xs;
        std::vector<std::vector<Eigen::VectorXd>> costates;
        std::vector<Strategy> current_strategies = initial_strategies;

        // 5. Quadraticize costs before first iteration.
        std::vector<std::vector<QuadraticCostApproximation>> cost_quad = computeCostQuadraticization(current_operating_point);

        // 6. Linearize dynamics about the new operating point
        std::vector<LinearDynamics> linear_dyn = computeLinearization(current_operating_point);

        //Main loop
        while(num_iterations < solver_params_.max_solver_iters && !has_converged)
        {
            // Start loop timer.
            timer_.Tic();

            // New iteration.
            num_iterations++;

            // 7. Solve LQ game.
            current_strategies = lq_solver_->solve(linear_dyn, cost_quad, x0, &delta_xs, &costates);

            // 8. Do Line Search (Update linear_dyn, cost_quad)
            if (!doLineSearch(delta_xs, costates, linear_dyn, cost_quad,
                              current_strategies, current_operating_point, has_converged))
            {
                // Emit warning if exiting early.
                std::cerr << "Solver exited due to linesearch failure." << std::endl;
                return false;
            }

            // Compute total costs and check if we've converged.
            total_costs = computeTotalCosts(current_operating_point);

            // Record loop runtime.
            elapsed += timer_.Toc();

            // Log current iterate.
            solver_log->addSolverIterate(current_operating_point, current_strategies,
                                         total_costs, elapsed, has_converged);
        }

        return true;
    }

    bool ILQSolver::doLineSearch(const std::vector<Eigen::VectorXd>& delta_xs,
                                 const std::vector<std::vector<Eigen::VectorXd>>& costates,
                                 std::vector<LinearDynamics>& linear_dyn,
                                 std::vector<std::vector<QuadraticCostApproximation>>& cost_quad,
                                 std::vector<Strategy>& strategies,
                                 OperatingPoint& current_operating_point,
                                 bool& has_converged)
    {
        // 1. Precompute expected decrease before we do anything else.
        double expected_decrease = computeExpectedDecrease(strategies, delta_xs, costates, linear_dyn, cost_quad);

        // 2. Scale ff term by Line Search Parameters
        scaleByLineSearchParam(solver_params_.initial_alpha_scaling, strategies);

        // 3. Forward
        const OperatingPoint last_operating_point(current_operating_point);
        double current_stepsize = solver_params_.initial_alpha_scaling;
        current_operating_point = forward(last_operating_point, strategies);

        if (!solver_params_.linesearch) return true;

        // Keep reducing alphas until we satisfy the Armijo condition.
        for (size_t ii = 0; ii < solver_params_.max_backtracking_steps; ii++)
        {
            // 4. Quadraticization
            computeCostQuadraticization(current_operating_point, cost_quad);

            // 5. Compute merit function value.
            const double current_merit_function_value = computeMeritFunction(current_operating_point, cost_quad);

            // 6. Check Armijo condition.
            if (checkArmijoCondition(current_merit_function_value, current_stepsize, expected_decrease))
            {
                // Success! Update cached terms and check convergence.
                has_converged = checkConverged(current_merit_function_value);
                last_merit_function_value_ = current_merit_function_value;

                // Update linearized dynamics
                computeLinearization(current_operating_point, linear_dyn);

                return true;
            }

            // 7. Update current_step_size
            current_stepsize *= solver_params_.geometric_alpha_scaling;

            // 8. Scale down the alphas and update operating point. Try again
            scaleByLineSearchParam(solver_params_.geometric_alpha_scaling, strategies);
            current_operating_point = forward(last_operating_point, strategies);
            computeLinearization(current_operating_point, linear_dyn);
        }

        // Output a warning. Solver should revert to last valid operating point.
        std::cerr << "Exceeded maximum number of backtracking steps." << std::endl;
        return false;
    }

    // Compute cost for each player around current operating point
    std::vector<double> ILQSolver::computeTotalCosts(const OperatingPoint& current_op) const
    {
        // Initialize
        std::vector<double> total_costs(problem_->getNumPlayers(), 0.0);

        // Compute Total costs
        // Accumulate costs.
        for (size_t kk = 0; kk < time::kNumTimeSteps; kk++) // For every time step
            for (size_t ii = 0; ii < problem_->getNumPlayers(); ii++) // For each Player
                total_costs[ii] += problem_->getPlayerCosts(ii).evaluate(kk, current_op.xs[kk], current_op.us[kk]);

        return total_costs;
    }

    std::vector<std::vector<QuadraticCostApproximation>> ILQSolver::computeCostQuadraticization(const OperatingPoint& op)
    {
        std::vector<std::vector<QuadraticCostApproximation>> q = initializeQuadraticCostApproximation();
        computeCostQuadraticization(op, q);

        return q;
    }

    void ILQSolver::computeCostQuadraticization(const OperatingPoint &op,
                                                std::vector<std::vector<QuadraticCostApproximation>>& q)
    {
        assert(q.size() == time::kNumTimeSteps);
        assert(q.front().size() == problem_->getNumPlayers());

        for (size_t kk = 0; kk < time::kNumTimeSteps; kk++)
        {
            const auto& x = op.xs[kk];
            const auto& us = op.us[kk];

            // Quadraticize costs.
            for (unsigned int ii = 0; ii < problem_->getNumPlayers(); ii++)
            {
                const PlayerCost& cost = problem_->getPlayerCosts(ii);
                q[kk][ii] = cost.quadraticize(kk, x, us);
            }
        }
    }
}