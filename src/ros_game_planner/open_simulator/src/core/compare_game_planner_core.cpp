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

#include "core/compare_game_planner_core.h"

CompareGamePlanner::CompareGamePlanner() : nh_(""), pnh_("~"), is_simulation_start_(false),
                                           ilq_solver_success_(false), ilqg_solver_success_(false),
                                           ilq_problem_(nullptr), ilqg_problem_(nullptr),
                                           ilq_log_(nullptr), ilqg_log_(nullptr),
                                           pos_dims_(nullptr), lanes_(nullptr), horizon_id_(0)
{
    // Initialize Problem(Only for Open-loop Simulator)
    SoftOnePlayerLaneChange scenario;
    scenario.initialize(ilq_problem_, ilqg_problem_, pos_dims_, lanes_, obstacles_);

    assert(ilq_problem_);
    assert(ilqg_problem_);

    // Solver Params
    game_planner::ALSolverParams al_ilq_params;
    al_ilq_params.iterative_solver_type_ = ALIterativeeSolverType::ILQ;
    al_ilq_params.iterative_solver_params_.max_backtracking_steps = 100;
    al_ilq_params.iterative_solver_params_.linesearch = true;
    al_ilq_params.iterative_solver_params_.expected_decrease_fraction = 0.1;
    al_ilq_params.iterative_solver_params_.initial_alpha_scaling = 0.75;
    al_ilq_params.iterative_solver_params_.convergence_tolerance = 0.001;

    game_planner::ALSolverParams al_ilqg_params;
    al_ilqg_params.iterative_solver_type_ = ALIterativeeSolverType::ILQG;
    al_ilqg_params.max_al_solver_iters = 1000;
    al_ilqg_params.iterative_solver_params_.max_backtracking_steps = 100;
    al_ilqg_params.iterative_solver_params_.linesearch = true;
    al_ilqg_params.iterative_solver_params_.expected_decrease_fraction = 0.1;
    al_ilqg_params.iterative_solver_params_.initial_alpha_scaling = 0.75;
    al_ilqg_params.iterative_solver_params_.convergence_tolerance = 0.001;

    // Setup Solver
    AugmentedLagrangianSolver ilq_solver(ilq_problem_, al_ilq_params);
    AugmentedLagrangianSolver ilqg_solver(ilqg_problem_, al_ilqg_params);

    // Initial Strategy
    std::vector<Strategy> initial_strategies;
    for (unsigned int ii = 0; ii < ilq_problem_->getNumPlayers(); ii++)
        initial_strategies.emplace_back(time::kNumTimeSteps, ilq_problem_->getXDim(), ilq_problem_->getUDim(ii));

    // Create New Log
    ilq_log_.reset(new SolverLog());
    ilqg_log_.reset(new SolverLog());

    // Solve Problem
    const auto start = std::chrono::system_clock::now();
    ilq_solver_success_ = ilq_solver.solve(initial_strategies, ilq_log_);
    ilqg_solver_success_ = ilqg_solver.solve(initial_strategies, ilqg_log_);
    const double calculation_time = std::chrono::duration<double>( std::chrono::system_clock::now() - start).count();
    ROS_INFO("Solver Calculation Time: %f", calculation_time);
    if(!ilq_solver_success_)
        ROS_INFO("ILQ Solver Calculation Failed");
    if(!ilqg_solver_success_)
        ROS_INFO("ILQG Solver Calculation Failed");

    // Parameter
    std::string ilq_pose_topic;
    std::string ilqg_pose_topic;
    pnh_.param("ilq_pose", ilq_pose_topic, std::string("/ilq_pose"));
    pnh_.param("ilqg_pose", ilqg_pose_topic, std::string("/ilqg_pose"));
    pnh_.param("ilq_base_link", base_link_name_, std::string("/base_link"));

    // Publisher
    result_trajectory_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("result_trajectory", 1);
    lane_pub_        = pnh_.advertise<visualization_msgs::MarkerArray>("lane", 1);
    ilqg_pose_pub_   = pnh_.advertise<visualization_msgs::Marker>(ilqg_pose_topic, 1);
    obstacle_pub_    = pnh_.advertise<visualization_msgs::Marker>("obstacle", 1);
    covariance_pub_  = pnh_.advertise<visualization_msgs::MarkerArray>("covariances", 1);
    sub_rviz_ini_    = pnh_.subscribe("/initialpose", 10, &CompareGamePlanner::callbackRvizInit, this);

    // Timer
    const double timer_dt = 0.1;
    timer_ = nh_.createTimer(ros::Duration(timer_dt), &CompareGamePlanner::timerCallback, this);
}

void CompareGamePlanner::callbackRvizInit(const geometry_msgs::PoseWithCovarianceStampedConstPtr &rviz_pose)
{
    is_simulation_start_ = true;
}

void CompareGamePlanner::timerCallback(const ros::TimerEvent& e)
{
    if(!ilq_solver_success_ || !ilqg_solver_success_)
        return;

    publishTrajectory(ilq_log_->getFinalOperatingPoint(), ilqg_log_->getFinalOperatingPoint());

    // Publish Covariance
    ros_utils::publishCovariance(covariance_pub_, ilqg_log_->getFinalOperatingPoint(),
                                 ilqg_log_->getFinalCovariance(), pos_dims_->at(0)[0], pos_dims_->at(0)[1]);

    if(lanes_)
        ros_utils::publishLane(lane_pub_, lanes_);

    if(!obstacles_.empty())
        for(unsigned int i=0; i<obstacles_.size(); ++i)
            ros_utils::publishObstacle(obstacle_pub_, obstacles_[i], i);

    // Publish ILQ State
    Eigen::VectorXd ilq_x = ilq_log_->getFinalOperatingPoint().xs[horizon_id_];
    geometry_msgs::Pose2D ego_pose;
    ego_pose.x = ilq_x[pos_dims_->at(0)[0]];
    ego_pose.y = ilq_x[pos_dims_->at(0)[1]];
    ego_pose.theta = ilq_x[pos_dims_->at(0)[2]];
    publishVehiclePose(ego_pose);

    Eigen::VectorXd ilqg_x = ilqg_log_->getFinalOperatingPoint().xs[horizon_id_];
    geometry_msgs::Pose2D ilqg_pose;
    ilqg_pose.x = ilqg_x[pos_dims_->at(0)[0]];
    ilqg_pose.y = ilqg_x[pos_dims_->at(0)[1]];
    ilqg_pose.theta = ilqg_x[pos_dims_->at(0)[2]];
    ros_utils::publishOtherVehiclePose(ilqg_pose_pub_, ilqg_pose, 0);

    if(is_simulation_start_)
        horizon_id_ = std::min(horizon_id_+1, ilq_log_->getFinalOperatingPoint().xs.size()-1);
}

void CompareGamePlanner::publishVehiclePose(const geometry_msgs::Pose2D& pose)
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

void CompareGamePlanner::publishTrajectory(const OperatingPoint& ilq_op,
                                           const OperatingPoint& ilqg_op)
{
    visualization_msgs::MarkerArray marker_array;

    // For ego player
    std::vector<geometry_msgs::Point> p1_points = ros_utils::generateTrajectoryPoints(ilq_op, pos_dims_->at(0)[0], pos_dims_->at(0)[1]);
    visualization_msgs::Marker p1_point_array = ros_utils::generateVisualizationPoints(p1_points,
                                                                                       std::string("ILQ Points"),
                                                                                       0, 1.0, 0.0, 0.8);
    visualization_msgs::Marker p1_line_array = ros_utils::generateVisualizationLine(p1_points,
                                                                                    visualization_msgs::Marker::LINE_STRIP,
                                                                                    std::string("ILQ Trajectory"),
                                                                                    0, 1.0, 0.5, 0.8);
    marker_array.markers.push_back(p1_point_array);
    marker_array.markers.push_back(p1_line_array);

    std::vector<geometry_msgs::Point> p2_points = ros_utils::generateTrajectoryPoints(ilqg_op, pos_dims_->at(0)[0], pos_dims_->at(0)[1]);
    visualization_msgs::Marker p2_point_array = ros_utils::generateVisualizationPoints(p2_points,
                                                                                       std::string("ILQG Points"),
                                                                                       0, 1.0, 1.0, 0.8);
    visualization_msgs::Marker p2_line_array = ros_utils::generateVisualizationLine(p2_points,
                                                                                    visualization_msgs::Marker::LINE_STRIP,
                                                                                    std::string("ILQG Trajectory"),
                                                                                    0, 1.0, 0.7, 0.8);
    marker_array.markers.push_back(p2_point_array);
    marker_array.markers.push_back(p2_line_array);

    result_trajectory_pub_.publish(marker_array);
}

