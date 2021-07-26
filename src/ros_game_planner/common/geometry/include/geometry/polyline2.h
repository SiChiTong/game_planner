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
// Polyline2 class for piecewise linear paths in 2D.
//
///////////////////////////////////////////////////////////////////////////////
#ifndef GAME_PLANNER_POLYLINE2_H
#define GAME_PLANNER_POLYLINE2_H

#include "geometry/line_segment2.h"
#include "utils/types.h"

namespace game_planner
{
    class Polyline2
    {
    public:
        Polyline2(const std::vector<Eigen::Vector2d>& points);
        ~Polyline2() = default;


        // Add a new point to the end of the polyline.
        void AddPoint(const Eigen::Vector2d& point);

        // Find closest point on this line segment to a given point, and optionally
        // the line segment that point belongs to (and flag for whether it is a
        // vertex), and the signed squared distance, where right is positive.
        Eigen::Vector2d ClosestPoint(const Eigen::Vector2d& query,
                                     bool* is_vertex = nullptr,
                                     LineSegment2* segment = nullptr,
                                     double* signed_squared_distance = nullptr,
                                     bool* is_endpoint = nullptr) const;

        // Find the point the given distance from the start of the polyline.
        // Optionally returns whether this is a vertex and the line segment which the
        // point belongs to.
        Eigen::Vector2d PointAt(const double& s_position,
                                bool* is_vertex = nullptr,
                                LineSegment2* segment = nullptr,
                                bool* is_endpoint = nullptr) const;

        // Access line segments.
        const std::vector<LineSegment2>& Segments() const { return segments_; }
        const LineSegment2& Segments(const unsigned int& id) const { assert(id<segments_.size()); return segments_[id]; }

        // Compute length.
        double Length() const { return length_; }

    private:
        std::vector<LineSegment2> segments_;
        std::vector<double> cumulative_lengths_;
        double length_;
    };
}

#endif //GAME_PLANNER_POLYLINE2_H