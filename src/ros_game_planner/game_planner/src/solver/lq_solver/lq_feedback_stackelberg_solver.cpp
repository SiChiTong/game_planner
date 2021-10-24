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
 * Authors: Yutaka Shimizu
 */
///////////////////////////////////////////////////////////////////////////////
//
// Core LQ game solver from Basar and Olsder, "Preliminary Notation for
// Corollary 7.2" (pp. 375). All notation matches the text, though we
// shall assume that `c` (additive drift in dynamics) is always `0`, which
// holds because these dynamics are for delta x, delta us.
// Also, we have modified terms slightly to account for linear terms in the
// stage cost for control, i.e.
//       control penalty i = 0.5 \sum_j du_j^T R_ij (du_j + 2 r_ij)
//
// Solve a time-varying, finite horizon LQ game (finds closed-loop Stackelberg
// feedback strategies for both players).
//
// Assumes that dynamics are given by
//           ``` dx_{k+1} = A_k dx_k + \sum_i Bs[i]_k du[i]_k ```
//
// Returns strategies Ps(Feedback Term), alphas(Feedforward Term).
//
///////////////////////////////////////////////////////////////////////////////
#include "solver/lq_solver/lq_feedback_stackelberg_solver.h"

namespace game_planner
{
    std::vector<Strategy> LQFeedbackStackelbergSolver::solve(const std::vector<LinearDynamics> &linearization,
                                                             const std::vector<std::vector<QuadraticCostApproximation>> &quadraticization,
                                                             const Eigen::VectorXd &x0,
                                                             std::vector<Eigen::VectorXd> *delta_xs,
                                                             std::vector<std::vector<Eigen::VectorXd>> *costates)
    {

        std::vector<Strategy> result;
        return result;
    }

    //debug version solve, check for the condition number
    std::vector<Strategy> LQFeedbackStackelbergSolver::debug_solve(const std::vector<LinearDynamics>& linearization,
                                    const std::vector<std::vector<QuadraticCostApproximation>> &quadraticization,
                                    const Eigen::VectorXd& x0,
                                    const size_t al_iter, const size_t ilqg_iter,
                                    std::vector<Eigen::VectorXd>* delta_xs ,
                                    std::vector<std::vector<Eigen::VectorXd>>* )
    {
        std::vector<Strategy> result;
        return result;

    }
}