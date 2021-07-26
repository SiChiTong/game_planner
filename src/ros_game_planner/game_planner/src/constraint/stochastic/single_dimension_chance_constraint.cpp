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

#include "constraint/stochastic/single_dimension_chance_constraint.h"

namespace game_planner
{
    double SingleDimensionChanceConstraint::evaluate(const Eigen::VectorXd &input,
                                                     const Eigen::MatrixXd &cov) const
    {
        assert(input.size() == cov.rows());
        assert(input.size() == cov.cols());
        assert(input.size() > dim_);

        const double sign = (keep_below_) ? 1.0 : -1.0;
        const double updated_threshold
                            = threshold_ - sign * std::sqrt(2*cov(dim_, dim_))*math::computeInverseErrorFunction(2*prob_-1);
        return sign*(input(dim_) - updated_threshold);
    }

    void SingleDimensionChanceConstraint::quadraticize(const int& time_id,
                                                       const Eigen::VectorXd& input,
                                                       const Eigen::MatrixXd& cov,
                                                       Eigen::MatrixXd& hess,
                                                       Eigen::VectorXd& grad) const
    {
        assert(hess.rows()==input.size());
        assert(hess.cols()==input.size());
        assert(grad.size()==input.size());

        // Compute gradient and Hessian.
        const double sign = (keep_below_) ? 1.0 : -1.0;
        const double updated_threshold
                = threshold_ - sign * std::sqrt(2*cov(dim_, dim_))*math::computeInverseErrorFunction(2*prob_-1);
        const double g = sign*(input(dim_) - updated_threshold);

        bool updated_sign = sgn(threshold_*updated_threshold);
        if(!updated_sign)
            std::cerr << "Updated Threshold has different sign from original ones" << std::endl;

        double dx  = sign;
        double ddx = 0.0;
        adaptALConstraint(time_id, g, &dx, &ddx);

        grad(dim_) += dx;
        hess(dim_, dim_) += ddx;

    }
}