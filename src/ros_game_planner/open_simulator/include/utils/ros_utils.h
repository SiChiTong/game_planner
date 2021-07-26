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
 * Authors:  Yutaka Shimizu
 */

///////////////////////////////////////////////////////////////////////////////
//
// Container for ROS Function
//
///////////////////////////////////////////////////////////////////////////////
#ifndef SRC_ROS_UTILS_H
#define SRC_ROS_UTILS_H

#include "geometry/polyline2.h"
#include "utils/operating_point.h"
#include "utils/obstacle/obstacle.h"

#include <cassert>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ros_utils
{
    /* Visualization Tool */
    visualization_msgs::Marker generateVisualizationPoints(const std::vector<geometry_msgs::Point>& points,
                                                           const std::string& name = "trajectory point",
                                                           const unsigned int& id = 0,
                                                           const double& red = 1.0,
                                                           const double& green = 0.0,
                                                           const double& blue = 0.8);

    visualization_msgs::Marker generateVisualizationLine(const std::vector<geometry_msgs::Point>& points,
                                                         const int& type = visualization_msgs::Marker::LINE_STRIP,
                                                         const std::string& name = "trajectory line",
                                                         const unsigned int& id = 0,
                                                         const double& red = 1.0,
                                                         const double& green = 0.5,
                                                         const double& blue = 0.8);

    std::vector<geometry_msgs::Point> generateLanePoints(const game_planner::Polyline2& lane);

    std::vector<geometry_msgs::Point> generateTrajectoryPoints(const game_planner::OperatingPoint& op,
                                                               const int& x_id,
                                                               const int& y_id);

    void publishOtherVehiclePose(const ros::Publisher& publisher,
                                 const geometry_msgs::Pose2D& pose,
                                 const unsigned int& id);
    void publishObstacle(const ros::Publisher& publisher,
                         const std::shared_ptr<game_planner::Obstacle>& obs,
                         const unsigned int& id);

    void publishLane(const ros::Publisher& publisher,
                     const std::shared_ptr<std::vector<game_planner::Polyline2>>& lanes_);

    /* Covariance Visualization Tool */
    void publishCovariance(const ros::Publisher& publisher,
                           const game_planner::OperatingPoint& op,
                           const std::vector<Eigen::MatrixXd>& covariances,
                           const int& x_id, const int& y_id);
    void makeRightHanded(Eigen::Matrix2d& eigenvectors, Eigen::Vector2d& eigenvalues);
    double computeYawAngelFromMatrix(const Eigen::Matrix2d& matrix);
}

#endif //SRC_ROS_UTILS_H
