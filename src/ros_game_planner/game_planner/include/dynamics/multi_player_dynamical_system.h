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
#ifndef GAME_PLANNER_MULTI_PLAYER_DYNAMICAL_SYSTEM_H
#define GAME_PLANNER_MULTI_PLAYER_DYNAMICAL_SYSTEM_H

#include "dynamics/single_player_dynamical_system.h"
#include "utils/linear_dynamics.h"
#include "utils/types.h"
#include <Eigen/Eigen>
#include <vector>
#include <cmath>

namespace game_planner
{
    class MultiPlayerDynamicalSystem
    {
    public:
        virtual ~MultiPlayerDynamicalSystem() = default;

        explicit MultiPlayerDynamicalSystem(const std::vector<std::shared_ptr<SinglePlayerDynamicalSystem>>& subsystems);

        // Compute time derivative of state.
        Eigen::VectorXd evaluate(const Eigen::VectorXd& x,
                                 const std::vector<Eigen::VectorXd>& us) const;

        // Compute a discrete-time Jacobian linearization.
        LinearDynamics linearize(const double& dt,
                                 const Eigen::VectorXd& x,
                                 const std::vector<Eigen::VectorXd>& us) const;

        // Integrate these dynamics forward in time.
        Eigen::VectorXd integrate(const double& time_interval,
                                  const Eigen::VectorXd& x0,
                                  const std::vector<Eigen::VectorXd>& us) const;

        // Getters.
        const std::vector<std::shared_ptr<SinglePlayerDynamicalSystem>>& getSubsystems() const { return subsystems_; }

        unsigned int getNumPlayers() const { return subsystems_.size(); }

        int getSubsystemStartDim(const unsigned int& player_idx) const
        {
            return subsystem_start_dims_[player_idx];
        }

        int getSubsystemXDim(const unsigned int& player_idx) const
        {
            return subsystems_[player_idx]->getXDim();
        }

        int getSubsystemVelocityDimension(const unsigned int& player_idx) const
        {
            return subsystems_[player_idx]->getVelocityDimension();
        }

        int getXDim() const
        {
            return xdim_;
        }

        int getUDim(const unsigned int& player_idx) const
        {
            return subsystems_[player_idx]->getUDim();
        }

        std::vector<int> getUDimVec() const
        {
            std::vector<int> udims(getNumPlayers());
            for(unsigned int i=0; i<udims.size(); ++i)
                udims[i] = subsystems_[i]->getUDim();

            return udims;
        }

        int getTotalUDim() const
        {
            int total = 0;
            for (unsigned int ii = 0; ii < getNumPlayers(); ii++) total += getUDim(ii);
            return total;
        }

        // Integrate using single step Euler or not, see below for more extensive
        // description.
        static void IntegrateUsingEuler() { integrate_using_euler_ = true; }
        static void IntegrateUsingRK4() { integrate_using_euler_ = false; }
        static bool getIntegrationUsesEuler() { return integrate_using_euler_; }

    private:
        // Cumulative sum of dimensions of each subsystem.
        std::vector<int> subsystem_start_dims_;

        // State dimension
        const int xdim_;

        // Whether to use single Euler during integration. Typically this is false but
        // it is typically used either for testing (we only derive Nash typically in
        // this case) or for speed.
        static bool integrate_using_euler_;

        // List of subsystems, each of which controls the affects of a single player.
        const std::vector<std::shared_ptr<SinglePlayerDynamicalSystem>> subsystems_;
    };
}


#endif //GAME_PLANNER_MULTI_PLAYER_DYNAMICAL_SYSTEM_H
