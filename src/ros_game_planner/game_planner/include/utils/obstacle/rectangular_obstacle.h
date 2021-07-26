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
// Container for Rectangular obstacle information
// Note that rectangular obstacle can consider the obstacle's shape, but
// cannot consider its covariance
//
///////////////////////////////////////////////////////////////////////////////
#ifndef GAME_PLANNER_RECTANGULAR_OBSTACLE_H
#define GAME_PLANNER_RECTANGULAR_OBSTACLE_H

#include "utils/obstacle/obstacle.h"

namespace game_planner
{
    class RectangularObstacle : public Obstacle
    {
    public:
        ~RectangularObstacle() = default;

        RectangularObstacle(const double& x,
                            const double& y,
                            const double& theta,
                            const double& length,
                            const double& width)
                            : Obstacle(x, y, theta, Eigen::MatrixXd::Zero(2,2), ObstacleType::RECTANGULAR), length_(length), width_(width)
        {
            const double half_diag = std::sqrt(length*length+width*width)/2;
            const double angle = std::atan2(width/2.0, length/2.0);
            vertexes_[0].first  = half_diag * std::cos(angle + theta);
            vertexes_[0].second = half_diag * std::sin(angle + theta);
            vertexes_[1].first  = half_diag * std::cos(M_PI - angle + theta);
            vertexes_[1].second = half_diag * std::sin(M_PI - angle + theta);
            vertexes_[2].first  = half_diag * std::cos(M_PI + angle + theta);
            vertexes_[2].second = half_diag * std::sin(M_PI + angle + theta);
            vertexes_[3].first  = half_diag * std::cos(2*M_PI - angle + theta);
            vertexes_[3].second = half_diag * std::sin(2*M_PI - angle + theta);
        }

        std::array<std::pair<double, double>,4> getVertexes() const { return vertexes_; }
        std::pair<double, double> getVertexes(const int& i) const { return vertexes_[i]; }

        double getLength() const { return length_; }
        double getWidth() const { return width_; }

    private:
        std::array<std::pair<double, double>, 4> vertexes_; // Relative Vertex Position
                                                            // from the center of the rectangular
                                                            // Note that (x,y) are the world position

        // For visualization
        const double length_;
        const double width_;
    };
}

#endif //GAME_PLANNER_RECTANGULAR_OBSTACLE_H
