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
// Base class for all types of quadratic cost
//
///////////////////////////////////////////////////////////////////////////////
#ifndef GAME_PLANNER_QUADRATIC_COST_H
#define GAME_PLANNER_QUADRATIC_COST_H

#include "utils/types.h"
#include "cost/deterministic/cost.h"

namespace game_planner
{
    class QuadraticCost : public Cost
    {
    public:
        // Construct from a multiplicative weight and the dimension in which to apply
        // the quadratic cost (difference from nominal). If dimension < 0, then
        // applies to all dimensions (i.e. ||input - nominal * ones()||^2).
        QuadraticCost(const double& weight,
                      const int& dim,
                      const double& nominal = 0.0,
                      const std::string& name = "")
                      : Cost(weight, name), dimension_(dim), nominal_(nominal)
        {}

        // Evaluate this cost at the current input.
        double evaluate(const Eigen::VectorXd& input) const;

        double evaluate(const int& time_id, const Eigen::VectorXd& input) const
        {
            return evaluate(input);
        }

        // Quadraticize this cost at the given input, and add to the running
        // sum of gradients and Hessians.
        void quadraticize(const Eigen::VectorXd& input,
                          Eigen::MatrixXd& hess,
                          Eigen::VectorXd& grad) const;

        void quadraticize(const int& time_id,
                          const Eigen::VectorXd& input,
                          Eigen::MatrixXd& hess,
                          Eigen::VectorXd& grad) const
        {
            quadraticize(input, hess, grad);
        }

    private:
        // Dimension in which to apply the quadratic cost.
        const int dimension_;

        // Nominal value in this (or all) dimensions.
        const double nominal_;
    };
}

#endif //GAME_PLANNER_QUADRATIC_COST_H
