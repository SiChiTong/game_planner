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
// Container for obstacle information(position)
//
///////////////////////////////////////////////////////////////////////////////

#ifndef GAME_PLANNER_OBSTACLE_H
#define GAME_PLANNER_OBSTACLE_H

#include "utils/types.h"

namespace game_planner
{
    enum ObstacleType
    {
        RECTANGULAR = 0,
        CIRCULAR = 1
    };

    class Obstacle
    {
    public:
        // Destructor
        virtual ~Obstacle() = default;

        // getter
        double getX() const { return x_; }
        double getY() const { return y_; }
        double getTheta() const { return theta_; }
        Eigen::MatrixXd getCovariance() const { return cov_; }
        ObstacleType getType() const { return type_; }

        virtual double getLength() const  = 0;
        virtual double getWidth() const  = 0;
        virtual std::array<std::pair<double, double>,4> getVertexes() const  = 0;
        virtual std::pair<double, double> getVertexes(const int& i) const = 0;

    protected:
        // Constructor
        explicit Obstacle(const double& x,
                          const double& y,
                          const double& theta,
                          const Eigen::MatrixXd& cov,
                          const ObstacleType& type)
                          : x_(x), y_(y), theta_(theta), cov_(cov), type_(type) {}

        // Member
        const double x_;
        const double y_;
        const double theta_;
        const Eigen::MatrixXd cov_;
        const ObstacleType type_;
    };
}

#endif //GAME_PLANNER_OBSTACLE_H
