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
//  Quadratic penalty on distance from a given Polyline2.
//
///////////////////////////////////////////////////////////////////////////////
#include "cost/deterministic/quadratic_polyline2_cost.h"

namespace game_planner
{
    double QuadraticPolyline2Cost::evaluate(const Eigen::VectorXd& input) const
    {
        assert(xidx_ < input.size());
        assert(yidx_ < input.size());

        // Compute signed squared distance by finding closest point.
        Eigen::Vector2d current_position(input(xidx_), input(yidx_));
        double signed_squared_distance;
        bool is_endpoint;
        polyline_.ClosestPoint(current_position, nullptr, nullptr,
                               &signed_squared_distance, &is_endpoint);

        if (is_endpoint) {
            // If the is_endpoint flag is raised, we set the signed_squared_distance to
            // 0.0.
            signed_squared_distance = 0.0;
        }

        return 0.5 * weight_ * std::abs(signed_squared_distance);
    }

    void QuadraticPolyline2Cost::quadraticize(const Eigen::VectorXd& input,
                                              Eigen::MatrixXd& hess,
                                              Eigen::VectorXd& grad) const
    {
        assert(xidx_ < input.size());
        assert(yidx_ < input.size());

        assert(input.size()==hess.rows());
        assert(input.size()==hess.cols());
        assert(input.size()==grad.size());

        // Unpack current position and find closest point / segment.
        const Eigen::Vector2d current_position(input(xidx_), input(yidx_));

        bool is_vertex;
        bool is_endpoint;
        LineSegment2 segment(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 1.0));
        const Eigen::Vector2d closest_point = polyline_.ClosestPoint(
                current_position, &is_vertex, &segment, nullptr, &is_endpoint);

        // First check whether the closest point is a endpoint of the polyline.
        if (is_endpoint) return;

        // Handle cases separately depending on whether or not closest point is
        // a vertex of the polyline.
        double ddx = weight_;
        double ddy = weight_;
        double dxdy = 0.0;
        double dx = weight_ * (current_position.x() - closest_point.x());
        double dy = weight_ * (current_position.y() - closest_point.y());

        if (!is_vertex)
        {
            const Eigen::Vector2d relative = current_position - segment.FirstPoint();
            const Eigen::Vector2d& unit_segment = segment.UnitDirection();

            // Handle Hessian first.
            ddx = weight_ * unit_segment.y() * unit_segment.y();
            ddy = weight_ * unit_segment.x() * unit_segment.x();
            dxdy = -weight_ * unit_segment.x() * unit_segment.y();

            // Handle gradient.
            const double w_cross = weight_ * (relative.x() * unit_segment.y() -
                                              relative.y() * unit_segment.x());

            dx = w_cross * unit_segment.y();
            dy = -w_cross * unit_segment.x();
        }

        grad(xidx_) += dx;
        grad(yidx_) += dy;

        hess(xidx_, xidx_) += ddx;
        hess(yidx_, yidx_) += ddy;
        hess(xidx_, yidx_) += dxdy;
        hess(yidx_, xidx_) += dxdy;
    }
}