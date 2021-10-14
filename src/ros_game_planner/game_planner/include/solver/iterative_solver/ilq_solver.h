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
//checked:1, q
#ifndef GAME_PLANNER_ILQ_SOLVER_H
#define GAME_PLANNER_ILQ_SOLVER_H

#include "utils/types.h"
#include "utils/loop_timer.h"
#include "base_iterative_solver.h"
#include "solver/lq_solver/lq_solver.h"
#include "solver/lq_solver/lq_feedback_nash_solver.h"
#include "solver/problem.h"
#include "solver/solver_log.h"

namespace game_planner
{
    class ILQSolver : public BaseIterativeSolver
    {
    public:
        virtual ~ILQSolver() {}
        ILQSolver(const std::shared_ptr<Problem>& problem, const SolverParams& solver_params = SolverParams())
             : BaseIterativeSolver(problem, solver_params)
        {
        }

        bool solve(const std::vector<Strategy>& initial_strategies,
                   const std::shared_ptr<SolverLog>& solver_log);

        //Warm start version solve
        bool solve(const OperatingPoint initial_op,
                   const std::shared_ptr<SolverLog>& solver_log);
 
        //Debugging for trajectory drift
        bool solve(const OperatingPoint initial_op,
                   const std::shared_ptr<SolverLog>& solver_log, size_t al_num_iter);

    protected:
        // Line Search
        bool doLineSearch(const std::vector<Eigen::VectorXd>& delta_xs,
                          const std::vector<std::vector<Eigen::VectorXd>>& costates,
                          std::vector<LinearDynamics>& linear_dyn,
                          std::vector<std::vector<QuadraticCostApproximation>>& cost_quad,
                          std::vector<Strategy>& strategies,
                          OperatingPoint& current_operating_point,
                          bool& has_converged);

        // Compute overall costs and set times of extreme costs.
        std::vector<double> computeTotalCosts(const OperatingPoint& current_op) const;

        // Compute the quadratic cost approximation at the given operating point.
        std::vector<std::vector<QuadraticCostApproximation>> computeCostQuadraticization(const OperatingPoint& op);
        void computeCostQuadraticization(const OperatingPoint& op,
                                         std::vector<std::vector<QuadraticCostApproximation>>& q);

    };
}

#endif //GAME_PLANNER_ILQ_SOLVER_H
