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
#ifndef GAME_PLANNER_SINGLE_PLAYER_CAR_6D_H
#define GAME_PLANNER_SINGLE_PLAYER_CAR_6D_H

#include "dynamics/single_player_dynamical_system.h"
#include "utils/types.h"

namespace game_planner
{
    class SinglePlayerCar6D : public SinglePlayerDynamicalSystem
    {
    public:
        ~SinglePlayerCar6D() = default;
        SinglePlayerCar6D(const double& inter_axle_distance)
                : SinglePlayerDynamicalSystem(kNumXDims, kNumUDims),
                  inter_axle_distance_(inter_axle_distance) {}

        // Compute time derivative of state.
        Eigen::VectorXd evaluate(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const;

        // Compute a discrete-time Jacobian linearization.
        void linearize(const double& dt,
                       const Eigen::VectorXd& x,
                       const Eigen::VectorXd& u,
                       Eigen::Ref<Eigen::MatrixXd> A,
                       Eigen::Ref<Eigen::MatrixXd> B) const;

        // Position dimensions.
        std::vector<int> getPositionDimension() const { return {kPxIdx, kPyIdx}; }

        int getVelocityDimension() const { return kVIdx; }

        // Constexprs for state indices.
        static const int kNumXDims;
        static const int kPxIdx;
        static const int kPyIdx;
        static const int kThetaIdx;
        static const int kPhiIdx;
        static const int kVIdx;
        static const int kAIdx;

        // Constexprs for control indices.
        static const int kNumUDims;
        static const int kOmegaIdx;
        static const int kJerkIdx;

    private:
        // Inter-axle distance. Determines turning radius.
        const double inter_axle_distance_;
    };
}

#endif //GAME_PLANNER_SINGLE_PLAYER_CAR_6D_H
