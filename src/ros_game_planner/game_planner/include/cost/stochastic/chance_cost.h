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
// Base class for all types of chance cost
//
///////////////////////////////////////////////////////////////////////////////
#ifndef GAME_PLANNER_CHANCE_COST_H
#define GAME_PLANNER_CHANCE_COST_H

#include "utils/math_utils.h"
#include "utils/types.h"

namespace game_planner
{
    class ChanceCost
    {
    public:
        virtual ~ChanceCost() = default;

        // Evaluate this cost at the current time and input.
        virtual double evaluate(const int& time_id, const Eigen::VectorXd& input, const Eigen::MatrixXd& cov) const = 0;
        virtual double evaluate(const Eigen::VectorXd& input, const Eigen::MatrixXd& cov) const = 0;

        // Quadraticize this cost at the given time and input, and add to the running
        // sum of gradients and Hessians.
        virtual void quadraticize(const int& time_id,
                                  const Eigen::VectorXd& input,
                                  const Eigen::MatrixXd& cov,
                                  Eigen::MatrixXd& hess,
                                  Eigen::VectorXd& grad) const = 0;

        virtual void quadraticize(const Eigen::VectorXd& input,
                                  const Eigen::MatrixXd& cov,
                                  Eigen::MatrixXd& hess,
                                  Eigen::VectorXd& grad) const = 0;

        // Reset and scale weight.
        void setWeight(const double& weight) { weight_ = weight; }
        void scaleWeight(const double& scale) { weight_ *= scale; }

        // Reset probability
        void setProbability(const double& prob) { prob_ = prob; }

        std::string getName() { return name_; }

    protected:
        explicit ChanceCost(const double& weight,
                            const double& prob,
                            const std::string& name="")
                            : weight_(weight), prob_(prob), name_(name)
        {}

        // Multiplicative weight associated to this cost
        double weight_;

        // Probability for chance constraint
        double prob_;

        // Cost name
        std::string name_;
    };
}

#endif //GAME_PLANNER_CHANCE_COST_H
