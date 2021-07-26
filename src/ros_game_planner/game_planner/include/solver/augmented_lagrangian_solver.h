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
#ifndef GAME_PLANNER_AUGMENTED_LAGRANGIAN_SOLVER_H
#define GAME_PLANNER_AUGMENTED_LAGRANGIAN_SOLVER_H

#include "utils/types.h"
#include "utils/loop_timer.h"
#include "constraint/deterministic/al_constraint.h"
#include "solver/iterative_solver/base_iterative_solver.h"
#include "solver/iterative_solver/ilqg_solver.h"
#include "solver/iterative_solver/ilq_solver.h"
#include "solver/lq_solver/lq_solver.h"
#include "solver/lq_solver/lq_feedback_nash_solver.h"
#include "solver/problem.h"
#include "solver/solver_log.h"

namespace game_planner
{
    enum ALIterativeeSolverType
    {
        ILQ = 0,
        ILQG = 1,
    };

    struct ALSolverParams
    {
        // For Iterative Solver
        SolverParams iterative_solver_params_ = SolverParams();

        // For Augmented Lagrangian parameters.
        ALIterativeeSolverType iterative_solver_type_ = ALIterativeeSolverType::ILQ;
        double geometric_mu_scaling = 1.1;
        double geometric_mu_downscaling = 0.5;
        double geometric_lambda_downscaling = 0.5;
        double constraint_error_tolerance = 1e-1;
        unsigned int max_al_solver_iters = 100;

        // Should the solver reset problem/constraint params to their initial values.
        // NOTE: defaults to true.
        bool reset_problem = true;
        bool reset_lambdas = true;
        bool reset_mu = true;
    };

    class AugmentedLagrangianSolver
    {
    public:
        ~AugmentedLagrangianSolver() = default;

        AugmentedLagrangianSolver(const std::shared_ptr<Problem>& problem,
                                  const ALSolverParams& solver_params = ALSolverParams()) : params_(solver_params)
        {
            if(solver_params.iterative_solver_type_ == ALIterativeeSolverType::ILQ)
            {
                unconstrained_solver_ = std::make_unique<ILQSolver>(problem, solver_params.iterative_solver_params_);
                solver_name_ = "AL_ILQ";
            }
            else
            {
                unconstrained_solver_ = std::make_unique<ILQGSolver>(problem, solver_params.iterative_solver_params_);
                solver_name_ = "AL_ILQG";
            }
        }

        // Solve this game. Returns true if converged.
        bool solve(const std::vector<Strategy>& initial_strategies,
                   const std::shared_ptr<SolverLog>& solver_log);

    private:
        //Base Iterative Solver (Can Select from ILQ and ILQG solver)
        std::unique_ptr<BaseIterativeSolver> unconstrained_solver_;
        const ALSolverParams params_;
        std::string solver_name_;
    };
}

#endif //GAME_PLANNER_AUGMENTED_LAGRANGIAN_SOLVER_H
