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
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// (Time-invariant) proximity (inequality) constraint between two vehicles, i.e.
//           g(x) = (+/-) (||(px1, py1) - (px2, py2)|| - d) <= 0
//
// NOTE: The `keep_within` argument specifies the sign of g (true corresponds to
// positive).
// If keep_within
//    g(x) = ||(px1, py1) - (px2, py2)|| - d  (||(px1, py1) - (px2, py2)|| <= d)
// else
//    g(x) = d - ||(px1, py1) - (px2, py2)||  (||(px1, py1) - (px2, py2)|| >= d)
//
///////////////////////////////////////////////////////////////////////////////
#ifndef GAME_PLANNER_PROXIMITY_CONSTRAINT_H
#define GAME_PLANNER_PROXIMITY_CONSTRAINT_H

#include "utils/types.h"
#include "constraint/deterministic/al_constraint.h"

namespace game_planner
{
    class ProximityConstraint : public ALConstraint
    {
    public:
        explicit ProximityConstraint(const std::pair<int, int>& dims1,
                            const std::pair<int, int>& dims2,
                            const double& threshold,
                            const bool& keep_within,
                            const std::string& name = "")
                : ALConstraint(false, name),
                  xidx1_(dims1.first),
                  yidx1_(dims1.second),
                  xidx2_(dims2.first),
                  yidx2_(dims2.second),
                  threshold_(threshold),
                  keep_within_(keep_within)
        {
            assert(threshold_>=0.0);
        }

        // Evaluate this constraint value, i.e., g(x).
        // Note this is a time invariant constraint
        double evaluate(const int& time_id, const Eigen::VectorXd& input) const
        {
            return evaluate(input);
        }

        double evaluate(const Eigen::VectorXd& input) const;

        // Quadraticize the constraint value and its square, each scaled by lambda or
        // mu, respectively (terms in the augmented Lagrangian).
        void quadraticize(const int& time_id,
                          const Eigen::VectorXd& input,
                          Eigen::MatrixXd& hess,
                          Eigen::VectorXd& grad) const;

    private:
        // Position dimension indices for both players.
        const int xidx1_;
        const int yidx1_;
        const int xidx2_;
        const int yidx2_;

        // Nominal distance threshold.
        const double threshold_;

        // Keep within (or without), i.e., orientation of the inequality.
        const bool keep_within_;
    };
}

#endif //GAME_PLANNER_PROXIMITY_CONSTRAINT_H
