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
// Single player dynamics modeling a car. 6 states and 2 control inputs.
// State is [x, y, theta, phi, v, a], control is [omega, j], and dynamics are:
//                     \dot px    = v cos theta
//                     \dot py    = v sin theta
//                     \dot theta = (v / L) * tan phi
//                     \dot phi   = omega
//                     \dot v     = a
//                     \dot a     = j
// Please refer to
// https://www.sciencedirect.com/science/article/pii/S2405896316301215
// for further details.
//
///////////////////////////////////////////////////////////////////////////////

#include "dynamics/single_player_car_6d.h"

namespace game_planner
{
    // Constexprs for state indices.
    const int SinglePlayerCar6D::kNumXDims = 6;
    const int SinglePlayerCar6D::kPxIdx = 0;
    const int SinglePlayerCar6D::kPyIdx = 1;
    const int SinglePlayerCar6D::kThetaIdx = 2;
    const int SinglePlayerCar6D::kPhiIdx = 3;
    const int SinglePlayerCar6D::kVIdx = 4;
    const int SinglePlayerCar6D::kAIdx = 5;

    // Constexprs for control indices.
    const int SinglePlayerCar6D::kNumUDims = 2;
    const int SinglePlayerCar6D::kOmegaIdx = 0;
    const int SinglePlayerCar6D::kJerkIdx = 1;


    // ----------------------------- IMPLEMENTATION ----------------------------- //
    inline Eigen::VectorXd SinglePlayerCar6D::evaluate(const Eigen::VectorXd& x,
                                                       const Eigen::VectorXd& u) const
    {
        /*********************************
         * dx/dt = v*cos(theta)
         * dy/dt = v*sin(theta)
         * d(theta)/dt = (v/L)*tan(theta)
         * d(phi)/dt = u(0) = omega(Note: omega)
         * d(v)/dt = x(5) = a
         * d(a)/dt = u(1) = jerk
         */
        Eigen::VectorXd xdot(xdim_);
        xdot(kPxIdx) = x(kVIdx) * std::cos(x(kThetaIdx));
        xdot(kPyIdx) = x(kVIdx) * std::sin(x(kThetaIdx));
        xdot(kThetaIdx) = (x(kVIdx) / inter_axle_distance_) * std::tan(x(kPhiIdx));
        xdot(kPhiIdx) = u(kOmegaIdx);
        xdot(kVIdx) = x(kAIdx);
        xdot(kAIdx) = u(kJerkIdx);

        return xdot;
    }

    inline void SinglePlayerCar6D::linearize(const double& dt,
                                             const Eigen::VectorXd& x,
                                             const Eigen::VectorXd& u,
                                             Eigen::Ref<Eigen::MatrixXd> A,
                                             Eigen::Ref<Eigen::MatrixXd> B) const
    {
        const double ctheta = std::cos(x(kThetaIdx)) * dt;
        const double stheta = std::sin(x(kThetaIdx)) * dt;
        const double cphi = std::cos(x(kPhiIdx));
        const double tphi = std::tan(x(kPhiIdx));

        A(kPxIdx, kThetaIdx) += -x(kVIdx) * stheta;
        A(kPxIdx, kVIdx) += ctheta;

        A(kPyIdx, kThetaIdx) += x(kVIdx) * ctheta;
        A(kPyIdx, kVIdx) += stheta;

        A(kThetaIdx, kPhiIdx) += x(kVIdx) * dt / (inter_axle_distance_ * cphi * cphi);
        A(kThetaIdx, kVIdx) += tphi * dt / inter_axle_distance_;

        A(kVIdx, kAIdx) += dt;

        B(kPhiIdx, kOmegaIdx) = dt;
        B(kAIdx, kJerkIdx) = dt;
    }
}
