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
// Note: This constraint is only applicable for circular obstacle
//
///////////////////////////////////////////////////////////////////////////////
#ifndef GAME_PLANNER_OBSTACLE_CONSTRAINT_H
#define GAME_PLANNER_OBSTACLE_CONSTRAINT_H

#include "constraint/deterministic/al_constraint.h"
#include "utils/obstacle/circular_obstacle.h"
#include "utils/types.h"

namespace game_planner
{
    class ObstacleConstraint : public ALConstraint
    {
    public:
        ~ObstacleConstraint() = default;

        ObstacleConstraint(const CircularObstacle& obs,
                           const std::pair<int, int>& position_idxs,
                           const double threshold,
                           const std::string& name = "")
                : ALConstraint(false, name),
                  obs_(obs),
                  threshold_(threshold),
                  threshold_sq_(threshold * threshold),
                  xidx_(position_idxs.first),
                  yidx_(position_idxs.second)
        {}

        // Evaluate this constraint at the current input.
        double evaluate(const int& time_id, const Eigen::VectorXd& input) const { return evaluate(input); }

        double evaluate(const Eigen::VectorXd& input) const;

        // Quadraticize this cost at the given input, and add to the running
        // sum of gradients and Hessians.
        void quadraticize(const int& time_id,
                          const Eigen::VectorXd& input,
                          Eigen::MatrixXd& hess,
                          Eigen::VectorXd& grad) const;

    private:
        // Position of the obstacle
        const CircularObstacle obs_;

        // Threshold for minimum squared relative distance.
        const double threshold_, threshold_sq_;

        // Position indices for two vehicles.
        const int xidx_, yidx_;
    };

}

#endif //GAME_PLANNER_OBSTACLE_CONSTRAINT_H
