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
// Penalizes (thresh - relative distance)^2 between two pairs of state
// dimensions (representing two positions of vehicles whose states have been
// concatenated) whenever relative distance is less than thresh.
//
///////////////////////////////////////////////////////////////////////////////
#include "cost/deterministic/proximity_cost.h"

namespace game_planner
{
    double ProximityCost::evaluate(const Eigen::VectorXd& input) const
    {
        const double dx = input(xidx1_) - input(xidx2_);
        const double dy = input(yidx1_) - input(yidx2_);
        const double delta_sq = dx * dx + dy * dy;

        if (delta_sq >= threshold_sq_) return 0.0;

        const double gap = threshold_ - std::sqrt(delta_sq);
        return 0.5 * weight_ * gap * gap;
    }

    void ProximityCost::quadraticize(const Eigen::VectorXd& input,
                                     Eigen::MatrixXd& hess,
                                     Eigen::VectorXd& grad) const
    {

        // Check dimensions.
        assert(input.size()==hess.rows());
        assert(input.size()==hess.cols());
        assert(input.size()==grad.size());

        // Compute Hessian and gradient.
        const double dx = input(xidx1_) - input(xidx2_);
        const double dy = input(yidx1_) - input(yidx2_);
        const double delta_sq = dx * dx + dy * dy;

        // Catch cost not active.
        if (delta_sq >= threshold_sq_) return;

        const double delta = std::sqrt(delta_sq);
        const double gap = threshold_ - delta;
        const double weight_delta = weight_ / delta;
        const double dx_delta = dx / delta;
        const double dy_delta = dy / delta;

        const double ddx1 = -weight_delta * gap * dx;
        const double ddy1 = -weight_delta * gap * dy;
        const double hess_x1x1 =
                weight_delta * (dx_delta * (gap * dx_delta + dx) - gap);
        const double hess_y1y1 =
                weight_delta * (dy_delta * (gap * dy_delta + dy) - gap);
        const double hess_x1y1 = weight_delta * (dx_delta * (gap * dy_delta + dy));

        grad(xidx1_) += ddx1;
        grad(xidx2_) -= ddx1;

        grad(yidx1_) += ddy1;
        grad(yidx2_) -= ddy1;

        hess(xidx1_, xidx1_) += hess_x1x1;
        hess(xidx1_, xidx2_) -= hess_x1x1;
        hess(xidx2_, xidx1_) -= hess_x1x1;
        hess(xidx2_, xidx2_) += hess_x1x1;

        hess(yidx1_, yidx1_) += hess_y1y1;
        hess(yidx1_, yidx2_) -= hess_y1y1;
        hess(yidx2_, yidx1_) -= hess_y1y1;
        hess(yidx2_, yidx2_) += hess_y1y1;

        hess(xidx1_, yidx1_) += hess_x1y1;
        hess(yidx1_, xidx1_) += hess_x1y1;

        hess(xidx1_, yidx2_) -= hess_x1y1;
        hess(yidx2_, xidx1_) -= hess_x1y1;

        hess(xidx2_, yidx1_) -= hess_x1y1;
        hess(yidx1_, xidx2_) -= hess_x1y1;

        hess(xidx2_, yidx2_) += hess_x1y1;
        hess(yidx2_, xidx2_) += hess_x1y1;
    }
}