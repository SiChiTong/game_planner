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
// Single player dynamics modeling a unicycle. 4 states and 2 control inputs.
// State is [x, y, theta, v], control is [omega, a], and dynamics are:
//                     \dot px    = v cos theta
//                     \dot py    = v sin theta
//                     \dot theta = omega
//                     \dot v     = a
//
///////////////////////////////////////////////////////////////////////////////
//checked: 2

#include "dynamics/single_player_unicycle_4d.h"

namespace game_planner
{
    // Constexprs for state indices.
    const int SinglePlayerUnicycle4D::kNumXDims = 4;
    const int SinglePlayerUnicycle4D::kPxIdx = 0;
    const int SinglePlayerUnicycle4D::kPyIdx = 1;
    const int SinglePlayerUnicycle4D::kThetaIdx = 2;
    const int SinglePlayerUnicycle4D::kVIdx = 3;

    // Constexprs for control indices.
    const int SinglePlayerUnicycle4D::kNumUDims = 2;
    const int SinglePlayerUnicycle4D::kOmegaIdx = 0;
    const int SinglePlayerUnicycle4D::kAIdx = 1;

    inline Eigen::VectorXd SinglePlayerUnicycle4D::evaluate(const Eigen::VectorXd &x,
                                                            const Eigen::VectorXd &u) const
    {
        Eigen::VectorXd xdot(xdim_);
        xdot(kPxIdx) = x(kVIdx) * std::cos(x(kThetaIdx));
        xdot(kPyIdx) = x(kVIdx) * std::sin(x(kThetaIdx));
        xdot(kThetaIdx) = u(kOmegaIdx);
        xdot(kVIdx) = u(kAIdx);

        return xdot;
    }

    
    inline void SinglePlayerUnicycle4D::linearize(const double& dt,
                                                  const Eigen::VectorXd& x,
                                                  const Eigen::VectorXd& u,
                                                  Eigen::Ref<Eigen::MatrixXd> A,
                                                  Eigen::Ref<Eigen::MatrixXd> B) const
    {
        assert(A.cols() == xdim_);
        assert(A.rows() == xdim_);
        assert(B.cols() == udim_);
        assert(B.rows() == xdim_);

        //Question: Why do we need to multiply by dt here? 

        const double ctheta = std::cos(x(kThetaIdx)) * dt;
        const double stheta = std::sin(x(kThetaIdx)) * dt;

        A(kPxIdx, kThetaIdx) += -x(kVIdx) * stheta;
        A(kPxIdx, kVIdx) += ctheta;

        A(kPyIdx, kThetaIdx) += x(kVIdx) * ctheta;
        A(kPyIdx, kVIdx) += stheta;

        B(kThetaIdx, kOmegaIdx) = dt;
        B(kVIdx, kAIdx) = dt;
    }
}