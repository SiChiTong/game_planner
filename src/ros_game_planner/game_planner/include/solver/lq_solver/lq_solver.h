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
 * Authors:  David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Base Class For LQ Solver (Nash and Stackelberg)
//
///////////////////////////////////////////////////////////////////////////////
//checked: 1
#ifndef GAME_PLANNER_LQ_SOLVER_H
#define GAME_PLANNER_LQ_SOLVER_H

#include "utils/linear_dynamics.h"
#include "utils/quadratic_cost_approximation.h"
#include "utils/strategy.h"

namespace game_planner
{
    class LQSolver
    {
    public:
        virtual  ~LQSolver() = default;

        // Main Function to solve the problem
        // Return the strategies for all time index
        virtual std::vector<Strategy> solve(const std::vector<LinearDynamics>& linearization,
                                            const std::vector<std::vector<QuadraticCostApproximation>>& quadraticization,
                                            const Eigen::VectorXd& x0,
                                            std::vector<Eigen::VectorXd>* delta_xs = nullptr,
                                            std::vector<std::vector<Eigen::VectorXd>>* costates = nullptr) = 0;

    protected:
        LQSolver(size_t num_time_steps)
                : num_time_steps_(num_time_steps)
        {
        }

        // Dynamics and number of time steps.
        const size_t num_time_steps_;
    };
}



#endif //GAME_PLANNER_LQ_SOLVER_H
