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
// (Time-invariant) single dimension constraint, i.e., g(x) = (+/-) (x_i - d),
// where d is a threshold and sign is determined by the `keep_below` argument
// (positive is true).
// if positive
//    g(x) = x_i - d (x_i <= d)
// else
//    g(x) = d - x_i (d <= x_i)
//
///////////////////////////////////////////////////////////////////////////////

#include "constraint/deterministic/single_dimension_constraint.h"

namespace game_planner
{
    double SingleDimensionConstraint::evaluate(const Eigen::VectorXd &input) const
    {
        return (keep_below_) ? input(dim_) - threshold_ : threshold_ - input(dim_);
    }

    void SingleDimensionConstraint::quadraticize(const int &time_id,
                                                 const Eigen::VectorXd &input,
                                                 Eigen::MatrixXd &hess,
                                                 Eigen::VectorXd &grad) const
    {
        assert(hess.rows()==input.size());
        assert(hess.cols()==input.size());
        assert(grad.size()==input.size());

        // Compute gradient and Hessian.
        const double sign = (keep_below_) ? 1.0 : -1.0;
        const double x = input(dim_);
        const double g = sign * (x - threshold_);

        double dx  = sign;
        double ddx = 0.0;
        adaptALConstraint(time_id, g, &dx, &ddx);

        grad(dim_) += dx;
        hess(dim_, dim_) += ddx;
    }
}
