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
 * Authors: Yutaka Shimizu
 */

///////////////////////////////////////////////////////////////////////////////
//
// (Time-invariant) single dimension chance constraint,
// i.e., g(x) = (+/-) (x_i - d),
//        Pr(g(x)) >= p
// where d is a threshold and sign is determined by the `keep_below` argument
// (positive is true).
// if positive
//    g(x) = x_i - d (x_i <= d)
// else
//    g(x) = d - x_i (d <= x_i)
//
///////////////////////////////////////////////////////////////////////////////

#ifndef GAME_PLANNER_SINGLE_DIMENSION_CHANCE_CONSTRAINT_H
#define GAME_PLANNER_SINGLE_DIMENSION_CHANCE_CONSTRAINT_H

#include "constraint/stochastic/al_chance_constraint.h"
#include "utils/types.h"

namespace game_planner
{
    class SingleDimensionChanceConstraint : public ALChanceConstraint
    {
    public:
        ~SingleDimensionChanceConstraint() = default;
        SingleDimensionChanceConstraint(const double& prob,
                                        const int& dim,
                                        const double& threshold,
                                        const bool& keep_below,
                                        const std::string& name = "")
                : ALChanceConstraint(prob, false, name),
                  dim_(dim),
                  threshold_(threshold),
                  keep_below_(keep_below) {}

        // Evaluate the cost(This constraint is time invariant)
        double evaluate(const int& time_id,
                        const Eigen::VectorXd& input,
                        const Eigen::MatrixXd& cov) const
        {
            return evaluate(input, cov);
        }

        double evaluate(const Eigen::VectorXd& input, const Eigen::MatrixXd& cov) const;

        // Quadraticize the cost
        void quadraticize(const int& time_id,
                          const Eigen::VectorXd& input,
                          const Eigen::MatrixXd& cov,
                          Eigen::MatrixXd& hess,
                          Eigen::VectorXd& grad) const;

    private:
        // Dimension to constrain, threshold value, and sign of constraint.
        const int dim_;
        const double threshold_;
        const bool keep_below_;
    };
}

#endif //GAME_PLANNER_SINGLE_DIMENSION_CHANCE_CONSTRAINT_H
