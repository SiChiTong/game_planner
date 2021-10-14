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
// Base Augmented Lagrangian Constraint class
//
///////////////////////////////////////////////////////////////////////////////
#ifndef GAME_PLANNER_AL_BASE_CONSTRAINT_H
#define GAME_PLANNER_AL_BASE_CONSTRAINT_H

#include "utils/types.h"

namespace game_planner
{
    class BaseALConstraint
    {
    public:
        virtual ~BaseALConstraint() = default;

        bool isSatisfied(const double& value) const
        {
            return (is_equality_) ? std::abs(value) <= constants::kSmallNumber
                                  : value <= constants::kSmallNumber;
        }

        std::string get_name() const
        {
            return name_;
        }

        static double& getGlobalMu() { return mu_; }
        static void scaleMu(const double& scale) { mu_ *= scale; }

    protected:
        explicit BaseALConstraint(const bool is_equality, const std::string& name)
                 : is_equality_(is_equality),
                   name_(name),
                   lambdas_(time::kNumTimeSteps, constants::kDefaultLambda)
        {}

        // Is this an equality constraint? If not, it is an inequality constraint.
        const bool is_equality_;

        // Constraint Name
        const std::string name_;

        // Multipliers, one per time step. Also a static augmented multiplier for an
        // augmented Lagrangian.
        std::vector<double> lambdas_;
        static double mu_;
    };
}

#endif //GAME_PLANNER_AL_BASE_CONSTRAINT_H
