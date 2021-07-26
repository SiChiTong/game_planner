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
#include "utils/ros_utils.h"

namespace ros_utils
{
    visualization_msgs::Marker generateVisualizationPoints(const std::vector<geometry_msgs::Point> &points,
                                                           const std::string &name,
                                                           const unsigned int &id,
                                                           const double &red,
                                                           const double &green,
                                                           const double &blue)
    {
        assert(0.0 <= red  && red  <= 1.0);
        assert(0.0 <= green && red <= 1.0);
        assert(0.0 <= blue && blue  <= 1.0);

        visualization_msgs::Marker point_array;
        point_array.lifetime = ros::Duration(-1);
        point_array.header.frame_id = "/world";
        point_array.header.stamp = ros::Time::now();
        point_array.ns = name;
        point_array.action = visualization_msgs::Marker::ADD;
        point_array.pose.orientation.w = 1.0;
        point_array.id = id;
        point_array.type = visualization_msgs::Marker::SPHERE_LIST;
        point_array.scale.x = 0.3;
        point_array.scale.y = 0.3;
        point_array.scale.z = 0.3;
        point_array.color.r = red;
        point_array.color.g = green;
        point_array.color.b = blue;
        point_array.color.a = 1.0f;
        for(const auto& point : points)
            point_array.points.push_back(point);

        return point_array;
    }

    visualization_msgs::Marker generateVisualizationLine(const std::vector<geometry_msgs::Point> &points,
                                                         const int &type,
                                                         const std::string &name,
                                                         const unsigned int &id,
                                                         const double &red,
                                                         const double &green,
                                                         const double &blue)
    {
        assert(0.0 <= red  && red  <= 1.0);
        assert(0.0 <= green && red <= 1.0);
        assert(0.0 <= blue && blue  <= 1.0);

        visualization_msgs::Marker line_array;
        line_array.lifetime = ros::Duration(-1);
        line_array.header.frame_id = "/world";
        line_array.header.stamp = ros::Time::now();
        line_array.ns = name;
        line_array.action = visualization_msgs::Marker::ADD;
        line_array.pose.orientation.w = 1.0;
        line_array.id = id;
        line_array.type = type;
        line_array.scale.x = 0.05;
        line_array.scale.y = 0.0;
        line_array.scale.z = 0.0;
        line_array.color.r = red;
        line_array.color.g = green;
        line_array.color.b = blue;
        line_array.color.a = 1.0f;
        for(const auto& point : points)
            line_array.points.push_back(point);

        return line_array;
    }

    std::vector<geometry_msgs::Point> generateLanePoints(const game_planner::Polyline2& lane)
    {
        std::vector<geometry_msgs::Point> lane_points;
        for(unsigned int i=0; i<lane.Segments().size(); ++i)
        {
            geometry_msgs::Point segment_first_point;
            segment_first_point.x = lane.Segments(i).FirstPoint(0);
            segment_first_point.y = lane.Segments(i).FirstPoint(1);
            lane_points.push_back(segment_first_point);

            geometry_msgs::Point segment_second_point;
            segment_second_point.x = lane.Segments(i).SecondPoint(0);
            segment_second_point.y = lane.Segments(i).SecondPoint(1);
            lane_points.push_back(segment_second_point);
        }

        return lane_points;
    }

    std::vector<geometry_msgs::Point> generateTrajectoryPoints(const game_planner::OperatingPoint& op,
                                                               const int& x_id,
                                                               const int& y_id)
    {
        unsigned int size = op.xs.size();
        std::vector<geometry_msgs::Point> points(size);
        for(unsigned int i=0; i<points.size(); ++i)
        {
            points[i].x = op.xs[i](x_id);
            points[i].y = op.xs[i](y_id);
            points[i].z = 0.0;
        }

        return points;
    }

    void publishOtherVehiclePose(const ros::Publisher& publisher,
                                 const geometry_msgs::Pose2D& pose,
                                 const unsigned int& id)
    {
        visualization_msgs::Marker marker;
        marker.lifetime = ros::Duration(-1);
        marker.header.frame_id = "/world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "other_vehicle";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pose.x;
        marker.pose.position.y = pose.y;
        tf2::Quaternion q;
        q.setRPY(0, 0, pose.theta);
        marker.pose.orientation = tf2::toMsg(q);
        marker.id = id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;

        publisher.publish(marker);
    }

    void publishObstacle(const ros::Publisher& publisher,
                         const std::shared_ptr<game_planner::Obstacle> &obs,
                         const unsigned int &id)
    {
        visualization_msgs::Marker marker;
        marker.lifetime = ros::Duration(-1);
        marker.header.frame_id = "/world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "static_obstacle";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = obs->getX();
        marker.pose.position.y = obs->getY();
        tf2::Quaternion q;
        q.setRPY(0, 0, obs->getTheta());
        marker.pose.orientation = tf2::toMsg(q);
        marker.id = id;

        if(obs->getType() == game_planner::ObstacleType::RECTANGULAR)
            marker.type = visualization_msgs::Marker::CUBE;
        else if(obs->getType() == game_planner::ObstacleType::CIRCULAR)
            marker.type = visualization_msgs::Marker::SPHERE;

        marker.scale.x = obs->getLength();
        marker.scale.y = obs->getWidth();
        marker.scale.z = 1.0;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;

        publisher.publish(marker);
    }

    void publishLane(const ros::Publisher& publisher,
                     const std::shared_ptr<std::vector<game_planner::Polyline2>>& lanes_)
    {
        visualization_msgs::MarkerArray marker_array;
        for(unsigned int i=0; i<lanes_->size(); ++i)
        {
            std::vector<geometry_msgs::Point> lane_points = ros_utils::generateLanePoints(lanes_->at(i));
            visualization_msgs::Marker lane_array = ros_utils::generateVisualizationLine(lane_points,
                                                                                         visualization_msgs::Marker::LINE_LIST,
                                                                                         std::string("Lane"),
                                                                                         i, 0.5, 0.5, 0.55);
            marker_array.markers.push_back(lane_array);
        }

        publisher.publish(marker_array);
    }

    void publishCovariance(const ros::Publisher& publisher,
                           const game_planner::OperatingPoint& op,
                           const std::vector<Eigen::MatrixXd>& covariances,
                           const int& x_id, const int& y_id)
    {
        visualization_msgs::MarkerArray vis_covariances;

        for(size_t i=0; i<covariances.size(); ++i)
        {
            double cov_x  = covariances[i](x_id, x_id);
            double cov_y  = covariances[i](y_id, y_id);
            double cov_xy = covariances[i](x_id, y_id);
            Eigen::Matrix2d cov;
            cov << cov_x, cov_xy,
                   cov_xy, cov_y;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(cov);

            if(eigensolver.info() != Eigen::Success)
                ROS_WARN("[Covariance Transformation]: Eigen Values are invalid");
            else
            {
                Eigen::Vector2d eigenvalues = eigensolver.eigenvalues();
                Eigen::Matrix2d eigenvectors = eigensolver.eigenvectors();
                ros_utils::makeRightHanded(eigenvectors, eigenvalues);
                double theta = ros_utils::computeYawAngelFromMatrix(eigenvectors);

                visualization_msgs::Marker vis_cov;
                vis_cov.lifetime = ros::Duration(-1);
                vis_cov.header.frame_id = "/world";
                vis_cov.header.stamp = ros::Time::now();
                vis_cov.ns = "ILQG";
                vis_cov.action = visualization_msgs::Marker::ADD;
                vis_cov.id = i;
                vis_cov.type = visualization_msgs::Marker::SPHERE;
                vis_cov.color.r = 1.0;
                vis_cov.color.g = 0.0;
                vis_cov.color.b = 0.0;
                vis_cov.color.a = 1.0f;
                vis_cov.pose.position.x = op.xs[i](x_id);
                vis_cov.pose.position.y = op.xs[i](y_id);
                tf2::Quaternion q;
                q.setRPY(0, 0, theta);
                vis_cov.pose.orientation = tf2::toMsg(q);
                vis_cov.scale.x = 2*std::sqrt(eigenvalues(0));
                vis_cov.scale.y = 2*std::sqrt(eigenvalues(1));
                vis_cov.scale.z = 0.001;
                vis_covariances.markers.push_back(vis_cov);
            }
        }
        publisher.publish(vis_covariances);
    }

    // From https://github.com/laas/rviz_plugin_covariance/blob/master/src/covariance_visual.cpp
    // Local function to force the axis to be right handed for 2D. Based on the one from ecl_statistics
    void makeRightHanded(Eigen::Matrix2d& eigenvectors, Eigen::Vector2d& eigenvalues)
    {
        // Note that sorting of eigenvalues may end up with left-hand coordinate system.
        // So here we correctly sort it so that it does end up being righ-handed and normalised.
        Eigen::Vector3d c0;  c0.setZero();  c0.head<2>() = eigenvectors.col(0);  c0.normalize();
        Eigen::Vector3d c1;  c1.setZero();  c1.head<2>() = eigenvectors.col(1);  c1.normalize();
        Eigen::Vector3d cc = c0.cross(c1);
        if (cc[2] < 0)
        {
            eigenvectors << c1.head<2>(), c0.head<2>();
            double e = eigenvalues[0];  eigenvalues[0] = eigenvalues[1];  eigenvalues[1] = e;
        }
        else
        {
            eigenvectors << c0.head<2>(), c1.head<2>();
        }
    }

    double computeYawAngelFromMatrix(const Eigen::Matrix2d& matrix)
    {
        if(std::sqrt(matrix(1,0)*matrix(1,0) + matrix(0,0)*matrix(0,0)) < 1e-6)
            return 0.0;
        else
            return std::atan2(matrix(1,0), matrix(0,0));
    }
}