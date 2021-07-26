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
// Line segment in 2D.
//
///////////////////////////////////////////////////////////////////////////////
#ifndef GAME_PLANNER_LINE_SEGMENT2_H
#define GAME_PLANNER_LINE_SEGMENT2_H

#include <Eigen/Eigen>
#include <vector>
#include <cmath>
#include <cassert>
#include <iostream>
#include "utils/types.h"

namespace game_planner
{
    class LineSegment2
    {
    public:
        ~LineSegment2() = default;

        LineSegment2(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2)
                : p1_(point1),
                  p2_(point2),
                  length_((point1 - point2).norm()),
                  unit_direction_((point2 - point1) / length_) {
        }

        // Accessors.
        double Length() const { return length_; }

        const Eigen::Vector2d &FirstPoint() const { return p1_; }

        const Eigen::Vector2d &SecondPoint() const { return p2_; }

        const double &FirstPoint(const int &id) const {
            assert(0 <= id && id < 2);
            return p1_(id);
        }

        const double &SecondPoint(const unsigned int &id) const {
            assert(id < 2);
            return p2_(id);
        }

        const Eigen::Vector2d &UnitDirection() const { return unit_direction_; }

        double Heading() const {
            return std::atan2(UnitDirection().y(), UnitDirection().x());
        }

        // Compute which side of this line segment the query point is on.
        // Returns true for the "right" side and false for the "left.""
        bool Side(const Eigen::Vector2d &query) const;

        // Find closest point on this line segment to a given point (and optionally
        // the signed squared distance, where right is positive, and whether or not
        // the closest point is an endpoint).
        Eigen::Vector2d ClosestPoint(const Eigen::Vector2d &query,
                                     bool *is_endpoint = nullptr,
                                     double *signed_squared_distance = nullptr) const;

    private:
        Eigen::Vector2d p1_;
        Eigen::Vector2d p2_;
        double length_;
        Eigen::Vector2d unit_direction_;
    };
}

#endif //GAME_PLANNER_LINE_SEGMENT2_H
