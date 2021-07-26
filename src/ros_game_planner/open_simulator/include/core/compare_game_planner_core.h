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
// Core class for a game planner
//
///////////////////////////////////////////////////////////////////////////////
#ifndef OPEN_SIMULATOR_OPEN_SIMULATOR_CORE_H
#define OPEN_SIMULATOR_OPEN_SIMULATOR_CORE_H

#include "geometry/line_segment2.h"
#include "solver/augmented_lagrangian_solver.h"
#include "solver/iterative_solver/ilqg_solver.h"
#include "solver/iterative_solver/ilq_solver.h"
#include "solver/problem.h"
#include "cost/deterministic/cost.h"
#include "cost/player_cost.h"
#include "cost/deterministic/quadratic_cost.h"
#include "cost/deterministic/quadratic_polyline2_cost.h"
#include "cost/deterministic/proximity_cost.h"
#include "cost/deterministic/semiquadratic_polyline2_cost.h"
#include "cost/deterministic/obstacle_cost.h"
#include "cost/deterministic/obstacle_linear_cost.h"
#include "constraint/deterministic/al_constraint.h"
#include "constraint/deterministic/obstacle_linear_constraint.h"
#include "dynamics/multi_player_dynamical_system.h"
#include "dynamics/single_player_unicycle_4d.h"
#include "dynamics/single_player_car_6d.h"
#include "geometry/polyline2.h"
#include "utils/types.h"
#include "utils/obstacle/rectangular_obstacle.h"
#include "utils/obstacle/circular_obstacle.h"
#include "utils/ros_utils.h"
#include "scenario/one_player_lane_change.h"
#include "scenario/soft_one_player_lane_change.h"

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

using namespace game_planner;

class CompareGamePlanner
{
public:
    CompareGamePlanner();
    ~CompareGamePlanner() = default;

    /* topic callback */
    void callbackRvizInit(const geometry_msgs::PoseWithCovarianceStampedConstPtr& rviz_pose);
    void timerCallback(const ros::TimerEvent& e);

    /* Publish Vehicle Pose */
    void publishVehiclePose(const geometry_msgs::Pose2D& pose);

    /* Publish Lane and Trajectory*/
    void publishTrajectory(const OperatingPoint& ilq_op,
                           const OperatingPoint& ilqg_op);

private:
    ros::NodeHandle nh_;                 //!< @brief ros node handle
    ros::NodeHandle pnh_;                //!< @brief private ros node handle

    //publisher
    ros::Publisher result_trajectory_pub_;
    ros::Publisher lane_pub_;
    ros::Publisher ilqg_pose_pub_;
    ros::Publisher obstacle_pub_;
    ros::Publisher covariance_pub_;
    ros::Subscriber sub_rviz_ini_;
    tf2_ros::TransformBroadcaster tf_br_;

    // Timer
    ros::Timer timer_;

    // flag
    bool is_simulation_start_;
    bool ilq_solver_success_;
    bool ilqg_solver_success_;

    // link name
    std::string base_link_name_;

    // Problem
    std::shared_ptr<Problem> ilq_problem_;
    std::shared_ptr<Problem> ilqg_problem_;

    // Solver log
    std::shared_ptr<SolverLog> ilq_log_;
    std::shared_ptr<SolverLog> ilqg_log_;

    // Position Dimension
    std::shared_ptr<std::vector<std::vector<int>>> pos_dims_;

    // Lane
    std::shared_ptr<std::vector<Polyline2>> lanes_;

    // Obstacle
    std::vector<std::shared_ptr<Obstacle>> obstacles_;

    // Index
    unsigned long horizon_id_;
};

#endif //OPEN_SIMULATOR_OPEN_SIMULATOR_CORE_H
