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

#include "core/game_planner_core.h"

GamePlanner::GamePlanner() : nh_(""), pnh_("~"), is_simulation_start_(false), solver_success_(false),
                             problem_(nullptr), log_(nullptr), pos_dims_(nullptr),
                             lanes_(nullptr), horizon_id_(0)
{
    // Initialize Problem(Only for Open-loop Simulator)
    TwoPlayersOvetaking scenario;
    scenario.initialize(problem_, pos_dims_, lanes_, obstacles_);

    assert(problem_);

    // Solver Params
    game_planner::ALSolverParams params;
    params.iterative_solver_type_ = ALIterativeeSolverType::ILQG;
    params.iterative_solver_params_.max_backtracking_steps = 100;
    //params.iterative_solver_params_.linesearch = true;
    params.iterative_solver_params_.linesearch = false;
    params.iterative_solver_params_.expected_decrease_fraction = 0.1;
    params.iterative_solver_params_.initial_alpha_scaling = 0.75;
    params.iterative_solver_params_.convergence_tolerance = 0.01;

    // Setup Solver

    AugmentedLagrangianSolver solver(problem_, params);

    // Initial Strategy
    std::vector<Strategy> initial_strategies;
    for (unsigned int ii = 0; ii < problem_->getNumPlayers(); ii++)
        initial_strategies.emplace_back(time::kNumTimeSteps, problem_->getXDim(), problem_->getUDim(ii));

    // Create New Log
    log_.reset(new SolverLog());

    // Solve Problem
    const auto start = std::chrono::system_clock::now();
    solver_success_ = solver.solve(initial_strategies, log_);
    const double calculation_time = std::chrono::duration<double>( std::chrono::system_clock::now() - start).count();
    ROS_INFO("Solver Calculation Time: %f", calculation_time);

    // Parameter
    std::string vehicle1_pose_topic;
    std::string vehicle2_pose_topic;
    pnh_.param("vehicle1_pose", vehicle1_pose_topic, std::string("/vehicle1_pose"));
    pnh_.param("vehicle2_pose", vehicle2_pose_topic, std::string("/vehicle2_pose"));
    pnh_.param("vehicle1_base_link", base_link_name_, std::string("/base_link"));

    // Publisher
    result_trajectory_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("result_trajectory", 1);
    lane_pub_        = pnh_.advertise<visualization_msgs::MarkerArray>("lane", 1);
    other_pose_pub_  = pnh_.advertise<visualization_msgs::Marker>(vehicle2_pose_topic, 1);
    obstacle_pub_    = pnh_.advertise<visualization_msgs::Marker>("obstacle", 1);
    sub_rviz_ini_    = pnh_.subscribe("/initialpose", 10, &GamePlanner::callbackRvizInit, this);

    // Timer
    const double timer_dt = 0.1;
    timer_ = nh_.createTimer(ros::Duration(timer_dt), &GamePlanner::timerCallback, this);
}

void GamePlanner::callbackRvizInit(const geometry_msgs::PoseWithCovarianceStampedConstPtr &rviz_pose)
{
    is_simulation_start_ = true;
}

void GamePlanner::timerCallback(const ros::TimerEvent& e)
{
    if(!solver_success_)
        return;

    OperatingPoint op = log_->getFinalOperatingPoint();
    publishTrajectory(op);

    if(lanes_)
        ros_utils::publishLane(lane_pub_, lanes_);

    if(!obstacles_.empty())
        for(unsigned int i=0; i<obstacles_.size(); ++i)
            ros_utils::publishObstacle(obstacle_pub_, obstacles_[i], i);

    // Pulibsh State
    Eigen::VectorXd x = op.xs[horizon_id_];
    geometry_msgs::Pose2D ego_pose;
    ego_pose.x = x[pos_dims_->at(0)[0]];
    ego_pose.y = x[pos_dims_->at(0)[1]];
    ego_pose.theta = x[pos_dims_->at(0)[2]];
    publishVehiclePose(ego_pose);

    for(unsigned int player_id=1; player_id<problem_->getNumPlayers(); ++player_id)
    {
        geometry_msgs::Pose2D other_pose;
        other_pose.x = x[pos_dims_->at(player_id)[0]];
        other_pose.y = x[pos_dims_->at(player_id)[1]];
        other_pose.theta = x[pos_dims_->at(player_id)[2]];
        ros_utils::publishOtherVehiclePose(other_pose_pub_, other_pose, player_id);
    }

    if(is_simulation_start_)
        horizon_id_ = std::min(horizon_id_+1, op.xs.size()-1);
}

void GamePlanner::publishVehiclePose(const geometry_msgs::Pose2D& pose)
{
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "world";
    transform.child_frame_id = base_link_name_;
    transform.transform.translation.x = pose.x;
    transform.transform.translation.y = pose.y;
    transform.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, pose.theta);
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_br_.sendTransform(transform);
}

void GamePlanner::publishTrajectory(const OperatingPoint &op)
{
    visualization_msgs::MarkerArray marker_array;

    // For ego player
    std::vector<geometry_msgs::Point> p1_points = ros_utils::generateTrajectoryPoints(op, pos_dims_->at(0)[0], pos_dims_->at(0)[1]);
    visualization_msgs::Marker p1_point_array = ros_utils::generateVisualizationPoints(p1_points,
                                                                            std::string("Result trajectory point"),
                                                                            0, 1.0, 0.0, 0.8);
    visualization_msgs::Marker p1_line_array = ros_utils::generateVisualizationLine(p1_points,
                                                                         visualization_msgs::Marker::LINE_STRIP,
                                                                         std::string("trajectory line"),
                                                                         0, 1.0, 0.5, 0.8);
    marker_array.markers.push_back(p1_point_array);
    marker_array.markers.push_back(p1_line_array);

    for(unsigned int player_id=1; player_id<problem_->getNumPlayers(); ++player_id)
    {
        std::vector<geometry_msgs::Point> points = ros_utils::generateTrajectoryPoints(op,
                                                                            pos_dims_->at(player_id)[0],
                                                                            pos_dims_->at(player_id)[1]);
        visualization_msgs::Marker point_array = ros_utils::generateVisualizationPoints(points,
                                                                             std::string("Result trajectory point"),
                                                                             1, 0.0, 0.0, 1.0);
        visualization_msgs::Marker line_array = ros_utils::generateVisualizationLine(points,
                                                                          visualization_msgs::Marker::LINE_STRIP,
                                                                          std::string("trajectory line"),
                                                                          1, 0.0, 0.5, 1.0);
        marker_array.markers.push_back(point_array);
        marker_array.markers.push_back(line_array);
    }

    result_trajectory_pub_.publish(marker_array);
}


