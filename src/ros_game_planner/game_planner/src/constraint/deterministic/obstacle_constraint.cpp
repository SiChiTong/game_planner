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
// (Time-invariant) static obstacle avoidance constraint  i.e.
//           g(x) = d - ||(px, py) - (obs_x, obs_y)||  <= 0
//
///////////////////////////////////////////////////////////////////////////////

#include "constraint/deterministic/obstacle_constraint.h"

namespace game_planner
{
    double ObstacleConstraint::evaluate(const Eigen::VectorXd &input) const
    {
        const double dx = input(xidx_) - obs_.getX();
        const double dy = input(yidx_) - obs_.getY();
        const double value = threshold_ - std::hypot(dx, dy);

        return value;
    }

    void ObstacleConstraint::quadraticize(const int& time_id,
                                          const Eigen::VectorXd &input,
                                          Eigen::MatrixXd &hess,
                                          Eigen::VectorXd &grad) const
    {
        // Check dimensions.
        assert(input.size()==hess.rows());
        assert(input.size()==hess.cols());
        assert(input.size()==grad.size());

        // Compute Hessian and gradient.
        const double dx = input(xidx_) - obs_.getX();
        const double dy = input(yidx_) - obs_.getY();
        const double dist = std::hypot(dx, dy);
        const double g = threshold_ - dist;

        double grad_x = -dx / dist;
        double grad_y = -dy / dist;
        double hess_xx = -(1.0 - grad_x * grad_x) / dist;
        double hess_yy = -(1.0 - grad_y * grad_y) / dist;
        double hess_xy = grad_x * grad_y / dist;

        adaptALConstraint(time_id, g, &grad_x, &hess_xx, &grad_y, &hess_yy, &hess_xy);

        grad(xidx_) += grad_x;
        grad(yidx_) += grad_y;

        hess(xidx_, xidx_) += hess_xx;
        hess(yidx_, yidx_) += hess_yy;
        hess(xidx_, yidx_) += hess_xy;
        hess(yidx_, xidx_) += hess_xy;
    }


}