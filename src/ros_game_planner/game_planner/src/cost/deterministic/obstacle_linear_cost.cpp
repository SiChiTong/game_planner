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
// Penalizes (thresh - relative distance)^2 between ego position and static
// rectangular obstacle
// Formulation: g*x < m -> g*x - m < 0
// Cost: cost = weight * (g * x - m)
//
///////////////////////////////////////////////////////////////////////////////
#include "cost/deterministic/obstacle_linear_cost.h"

namespace game_planner
{
    double ObstacleLinearCost::evaluate(const Eigen::VectorXd& input) const
    {
        std::tuple<Eigen::Vector2d, double, double> result = math::computeConvexFeasibleSet(input, obs_, xidx_, yidx_);
        const Eigen::Vector2d g = std::get<0>(result);
        const double m = std::get<1>(result);
        Eigen::Vector2d x = Eigen::Vector2d::Zero();
        x << input(xidx_), input(yidx_);

        // Compute weight*(g*x - m)
        return std::max(0.0, weight_ * (g.dot(x) - m));
    }

    void ObstacleLinearCost::quadraticize(const Eigen::VectorXd &input,
                                          Eigen::MatrixXd &hess,
                                          Eigen::VectorXd &grad) const
    {
        // Cost = weight*(g*x - m)
        // Derivative of cost: d(cost)/dx = weight*g

        std::tuple<Eigen::Vector2d, double, double> result = math::computeConvexFeasibleSet(input, obs_, xidx_, yidx_);
        const Eigen::Vector2d g = std::get<0>(result);
        const double m = std::get<1>(result);
        Eigen::Vector2d x = Eigen::Vector2d::Zero();
        x << input(xidx_), input(yidx_);

        double diff = g.dot(x) - m;
        if(diff<constants::kSmallNumber)
            return;

        // Only Update gradient vector as this is a linear cost
        grad(xidx_) += weight_ * g(0);
        grad(yidx_) += weight_ * g(1);
    }
}
