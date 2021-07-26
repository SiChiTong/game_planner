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
// Chance Constraint Cost for Obstacle Avoidance
// Pr(g*x-m<=0) > p
//
///////////////////////////////////////////////////////////////////////////////
#include "cost/stochastic/chance_obstacle_linear_cost.h"

namespace game_planner
{
    double ChanceObstacleLinearCost::evaluate(const Eigen::VectorXd &input, const Eigen::MatrixXd &cov) const
    {
        assert(cov.rows()>xidx_);
        assert(cov.cols()>xidx_);
        assert(cov.rows()>yidx_);
        assert(cov.cols()>yidx_);

        Eigen::Matrix2d position_cov = Eigen::Matrix2d::Zero();
        position_cov << cov(xidx_, xidx_), cov(xidx_, yidx_),
                        cov(yidx_, xidx_), cov(yidx_, yidx_);

        std::tuple<Eigen::Vector2d, double, double> result = math::computeConvexFeasibleSet(input, obs_, xidx_, yidx_);
        const Eigen::Vector2d g = std::get<0>(result);
        const double m = std::get<1>(result);
        double r = std::sqrt(2*g.dot(position_cov*g))*math::computeInverseErrorFunction(2*prob_-1);
        Eigen::Vector2d x = Eigen::Vector2d::Zero();
        x << input(xidx_), input(yidx_);

        return std::max(0.0, weight_*(g.dot(x) - (m-r)));
    }

    void ChanceObstacleLinearCost::quadraticize(const Eigen::VectorXd &input,
                                                const Eigen::MatrixXd &cov,
                                                Eigen::MatrixXd &hess,
                                                Eigen::VectorXd &grad) const
    {
        // Gradient Value is same as non chance constraint linear gradient
        assert(cov.rows()>xidx_);
        assert(cov.cols()>xidx_);
        assert(cov.rows()>yidx_);
        assert(cov.cols()>yidx_);

        Eigen::Matrix2d position_cov = Eigen::Matrix2d::Zero();
        position_cov << cov(xidx_, xidx_), cov(xidx_, yidx_),
                        cov(yidx_, xidx_), cov(yidx_, yidx_);

        std::tuple<Eigen::Vector2d, double, double> result = math::computeConvexFeasibleSet(input, obs_, xidx_, yidx_);
        const Eigen::Vector2d g = std::get<0>(result);
        const double m = std::get<1>(result);
        double r = std::sqrt(2*g.dot(position_cov*g))*math::computeInverseErrorFunction(2*prob_-1);
        Eigen::Vector2d x = Eigen::Vector2d::Zero();
        x << input(xidx_), input(yidx_);

        double diff = g.dot(x) - (m-r);
        if(diff<constants::kSmallNumber)
            return;

        // Only Update gradient vector as this is a linear cost
        grad(xidx_) += weight_ * g(0);
        grad(yidx_) += weight_ * g(1);
    }
}
