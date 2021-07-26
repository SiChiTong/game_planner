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
//checked: 2
#ifndef GAME_PLANNER_BASE_ITERATIVE_SOLVER_H
#define GAME_PLANNER_BASE_ITERATIVE_SOLVER_H

#include "utils/types.h"
#include "utils/loop_timer.h"
#include "solver/lq_solver/lq_solver.h"
#include "solver/lq_solver/lq_feedback_nash_solver.h"
#include "solver/problem.h"
#include "solver/solver_log.h"

namespace game_planner
{
    struct SolverParams
    {
        // Consider a solution converged once max elementwise difference is below this
        // tolerance or solver has exceeded a maximum number of iterations.
        double convergence_tolerance = 1e-1;
        size_t max_solver_iters = 1000;

        // Linesearch parameters. If flag is set 'true', then applied initial alpha
        // scaling to all strategies and backs off geometrically at the given rate for
        // the specified number of steps.
        bool linesearch = true;
        double initial_alpha_scaling = 0.5;
        double geometric_alpha_scaling = 0.5;
        size_t max_backtracking_steps = 10;
        double expected_decrease_fraction = 0.1;

        // Solver Specification
        std::string solver_type = "FBNash"; // FFNash, FBNash, FBStackel

        size_t unconstrained_solver_max_iters = 10;

        // State and control regularization.
        double state_regularization = 0.0;
        double control_regularization = 0.0;

        // Max Record Timer Value
        int max_timer_record_num = 10;
    };

    class BaseIterativeSolver
    {
    public:
        virtual ~BaseIterativeSolver() = default;
        virtual bool solve(const std::vector<Strategy>& initial_strategies,
                           const std::shared_ptr<SolverLog>& solver_log) = 0;

        // Accessors.
        Problem& getProblem() { return *problem_; }

        // Check if solver has converged.
        virtual bool checkConverged(double current_merit_function_value) const
        {
            return (last_merit_function_value_ - current_merit_function_value) < solver_params_.convergence_tolerance;
        }


    protected:
        explicit BaseIterativeSolver(const std::shared_ptr<Problem>& problem, const SolverParams& solver_params = SolverParams())
                 : problem_(problem), solver_params_(solver_params),
                   timer_(solver_params.max_timer_record_num),
                   last_merit_function_value_(constants::kInfinity)
        {
            if(solver_params_.solver_type=="FBNash")
                lq_solver_ = std::make_unique<LQFeedbackNashSolver>(problem->getXDim(), problem->getUDimVec(), time::kNumTimeSteps);
            else
                lq_solver_ = std::make_unique<LQFeedbackNashSolver>(problem->getXDim(), problem->getUDimVec(), time::kNumTimeSteps);
        }

        /******************************************
         *************** Functions ****************
         ******************************************/

        // Compute the current operating point based on the current set of
        // strategies and the last operating point.
        OperatingPoint forward(const OperatingPoint& last_operating_point,
                               const std::vector<Strategy>& current_strategies);

        // Scale Line Search Parameter
        static void scaleByLineSearchParam(const double& scaling_value, std::vector<Strategy>& strategies)
        {
            for (auto& strategy : strategies)
                for (auto& alpha : strategy.alphas) alpha *= scaling_value;
        }

        // Populate the given vector with a linearization of the dynamics about
        // the given operating point. Provide version with no operating point for use
        // with feedback linearizable systems.
        std::vector<LinearDynamics> computeLinearization(const OperatingPoint& op);
        void computeLinearization(const OperatingPoint& op, std::vector<LinearDynamics>& linearized_dynamics);

        // Armijo condition check. Returns true if the new operating point satisfies
        // the Armijo condition, and also returns current merit function value.
        bool checkArmijoCondition(const double& current_merit_function_value,
                                  const double& current_stepsize,
                                  const double& expected_decrease) const;

        // Compute current merit function value. Note that to compute the merit
        // function at the given operating point we have to compute a full cost
        // quadraticization there. To do so efficiently, this will overwrite the
        // current cost quadraticization (and presume it has already been used to
        // compute the expected decrease from the last iterate).
        double computeMeritFunction(const OperatingPoint& current_op,
                                    const std::vector<std::vector<QuadraticCostApproximation>>& cost_quadraticization);

        // Compute expected decrease based on current cost quadraticization,
        // (player-indexed) strategies, and (time-indexed) lists of delta states and
        // (also player-indexed) costates.
        double computeExpectedDecrease(const std::vector<Strategy>& strategies,
                                       const std::vector<Eigen::VectorXd>& delta_xs,
                                       const std::vector<std::vector<Eigen::VectorXd>>& costates,
                                       const std::vector<LinearDynamics>& linearization,
                                       const std::vector<std::vector<QuadraticCostApproximation>>& cost_quadraticization) const;

        std::vector<std::vector<QuadraticCostApproximation>> initializeQuadraticCostApproximation()
        {
            return std::vector<std::vector<QuadraticCostApproximation>>(time::kNumTimeSteps,
                                                                        std::vector<QuadraticCostApproximation>
                                                                                (problem_->getNumPlayers(),
                                                                                 QuadraticCostApproximation(problem_->getXDim()))
            );
        }

        /*************************************
         *************** Member **************
         ************************************/

        // Problem
        const std::shared_ptr<Problem> problem_;

        // Solver Params
        const SolverParams solver_params_;

        // Timer
        LoopTimer timer_;

        // Last merit function
        double last_merit_function_value_;

        // Core LQ Solver.
        std::unique_ptr<LQSolver> lq_solver_;
    };
}

#endif //GAME_PLANNER_BASE_ITERATIVE_SOLVER_H
