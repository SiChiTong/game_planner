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
// Class for multi-player dynamical systems. Supports (discrete-time)
// linearization and integration.
//
///////////////////////////////////////////////////////////////////////////////
//checked: 1
#include "dynamics/multi_player_dynamical_system.h"

namespace game_planner
{
    bool MultiPlayerDynamicalSystem::integrate_using_euler_ = false;

    MultiPlayerDynamicalSystem::MultiPlayerDynamicalSystem(const std::vector<std::shared_ptr<SinglePlayerDynamicalSystem>>& subsystems)
                                                           : xdim_(std::accumulate(
                                                                   subsystems.begin(), subsystems.end(), 0,
                                                                   [](int total,
                                                                   const std::shared_ptr<SinglePlayerDynamicalSystem>& subsystem) {
                                                                   assert(subsystem.get());
                                                                   return total + subsystem->getXDim();
                                                                   })),
                                                             subsystems_(subsystems)
    {
        // Populate subsystem start dimensions.
        subsystem_start_dims_.push_back(0);
        for (const auto& subsystem : subsystems_)
            subsystem_start_dims_.push_back(subsystem_start_dims_.back() + subsystem->getXDim());
    }

    Eigen::VectorXd MultiPlayerDynamicalSystem::evaluate(const Eigen::VectorXd& x,
                                                         const std::vector<Eigen::VectorXd>& us) const
    {
        assert(us.size()==getNumPlayers());

        // Populate 'xdot' one subsystem at a time.
        Eigen::VectorXd xdot(xdim_);
        int dims_so_far = 0;
        for (size_t ii = 0; ii < getNumPlayers(); ii++)
        {
            const auto& subsystem = subsystems_[ii];
            xdot.segment(dims_so_far, subsystem->getXDim()) = subsystem->evaluate(x.segment(dims_so_far, subsystem->getXDim()), us[ii]);
            dims_so_far += subsystem->getXDim();
        }

        return xdot;
    }

    LinearDynamics MultiPlayerDynamicalSystem::linearize(const double& dt,
                                                         const Eigen::VectorXd& x,
                                                         const std::vector<Eigen::VectorXd>& us) const
    {
        assert(us.size()==getNumPlayers());

        // Populate a block-diagonal A, as well as Bs.
        std::vector<int> udims(getNumPlayers(), 0);
        for(unsigned int i=0; i<getNumPlayers(); ++i)
            udims[i] = this->getUDim(i);

        LinearDynamics linearization(getNumPlayers(), getXDim(), udims);

        int dims_so_far = 0;
        for (size_t ii = 0; ii < getNumPlayers(); ii++)
        {
            const auto& subsystem = subsystems_[ii];
            const int xdim = subsystem->getXDim();
            const int udim = subsystem->getUDim();
            subsystem->linearize(dt, x.segment(dims_so_far, xdim), us[ii],
                                 linearization.A.block(dims_so_far, dims_so_far, xdim, xdim),
                                 linearization.Bs[ii].block(dims_so_far, 0, xdim, udim));

            dims_so_far += subsystem->getXDim();
        }

        return linearization;
    }

    Eigen::VectorXd MultiPlayerDynamicalSystem::integrate(const double& time_interval,
                                                          const Eigen::VectorXd& x0,
                                                          const std::vector<Eigen::VectorXd>& us) const
    {
        Eigen::VectorXd x(x0);

        if (integrate_using_euler_)
        {
            x += time_interval * evaluate(x0, us);
        }
        else
        {
            // Number of integration steps and corresponding time step.
            constexpr size_t kNumIntegrationSteps = 2;
            const double dt = time_interval / static_cast<double>(kNumIntegrationSteps);

            // RK4 integration. See https://en.wikipedia.org/wiki/Runge-Kutta_methods
            // for further details.
            for (double t = 0.0; t < time_interval - 0.5 * dt; t += dt)
            {
                const Eigen::VectorXd k1 = dt * evaluate(x, us);
                const Eigen::VectorXd k2 = dt * evaluate(x + 0.5 * k1, us);
                const Eigen::VectorXd k3 = dt * evaluate(x + 0.5 * k2, us);
                const Eigen::VectorXd k4 = dt * evaluate(x + k3, us);

                x += (k1 + 2.0 * (k2 + k3) + k4) / 6.0;
            }
        }

        return x;
    }
}