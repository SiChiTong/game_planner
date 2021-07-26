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
// Base class for all single-player dynamical systems. Supports (discrete-time)
// linearization.
//
///////////////////////////////////////////////////////////////////////////////
//check: 1
#ifndef GAME_PLANNER_SINGLE_PLAYER_DYNAMICAL_SYSTEM_H
#define GAME_PLANNER_SINGLE_PLAYER_DYNAMICAL_SYSTEM_H

#include <Eigen/Eigen>
#include <vector>
#include <cmath>

namespace game_planner
{
    class SinglePlayerDynamicalSystem
    {
    public:
        virtual ~SinglePlayerDynamicalSystem() {}

        // Compute (dx/dt)
        virtual Eigen::VectorXd evaluate(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const = 0;

        virtual void linearize(const double& dt,
                               const Eigen::VectorXd& x,
                               const Eigen::VectorXd& u,
                               Eigen::Ref<Eigen::MatrixXd> A,
                               Eigen::Ref<Eigen::MatrixXd> B) const = 0;

        // Getter
        virtual std::vector<int> getPositionDimension() const = 0;
        virtual int getVelocityDimension() const = 0;
        int getXDim() const { return xdim_; }
        int getUDim() const { return udim_; }

    protected:
        SinglePlayerDynamicalSystem(const int& xdim, const int& udim) : xdim_(xdim), udim_(udim) {};

        //Dimensions
        const int xdim_;
        const int udim_;
    };
}

#endif //GAME_PLANNER_SINGLE_PLAYER_DYNAMICAL_SYSTEM_H
