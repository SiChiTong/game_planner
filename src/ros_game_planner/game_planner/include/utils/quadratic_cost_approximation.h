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
// Container to store a quadratic approximation of a single player's cost at a
// particular moment in time. That is, each player should have a time-indexed
// set of these QuadraticApproximations.
//
// Notation is taken from Basar and Olsder, Corollary 6.1.
// -- Q is the Hessian with respect to state
// -- l is the gradient with respect to state
// -- Rs[ii] is the Hessian with respect to the control input of player ii
// -- rs[ii] is the gradient with respect to the control input of player ii
//
///////////////////////////////////////////////////////////////////////////////
//checked: 2
#ifndef GAME_PLANNER_QUADRATIC_COST_APPROXIMATION_H
#define GAME_PLANNER_QUADRATIC_COST_APPROXIMATION_H

#include "utils/types.h"
#include <unordered_map>

namespace game_planner
{

    struct SingleCostApproximation
    {
        // Construct from matrix/vector directly.
        SingleCostApproximation(const Eigen::MatrixXd& hessian, const Eigen::VectorXd& gradient)
                : hess_(hessian), grad_(gradient)
        {
            assert(hess_.rows()==hess_.cols());
            assert(hess_.rows()==grad_.size());
        }

        // Construct with zeros.
        SingleCostApproximation(const int& dim, const double& regularization = 0.0)
                : hess_(regularization * Eigen::MatrixXd::Identity(dim, dim)),
                  grad_(Eigen::VectorXd::Zero(dim))
        {}
        
        Eigen::MatrixXd hess_;
        Eigen::VectorXd grad_;
    };

    struct QuadraticCostApproximation
    {
        // Construct from state dimension.
        explicit QuadraticCostApproximation(const int& xdim,
                                            const double& regularization = 0.0)
                : state_(xdim, regularization)
        {}

        SingleCostApproximation state_;
        std::unordered_map<unsigned int, SingleCostApproximation> control_;
    };
}

#endif //GAME_PLANNER_QUADRATIC_COST_APPROXIMATION_H
