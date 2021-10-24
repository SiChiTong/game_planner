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
// Container to store solver logs.
//
///////////////////////////////////////////////////////////////////////////////
#ifndef GAME_PLANNER_SOLVER_LOG_H
#define GAME_PLANNER_SOLVER_LOG_H

#include <sys/stat.h>
#include <sys/types.h>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <regex>
#include <vector>
#include "utils/operating_point.h"
#include "utils/strategy.h"
#include "utils/types.h"
#include "utils/quadratic_cost_approximation.h"
#include "utils/linear_dynamics.h"

namespace game_planner
{
    class Uncopyable
    {
    protected:
        Uncopyable() {}
        virtual ~Uncopyable() {}

    private:
        // Declare (but do not define) copy constructor and copy-assign operator
        // so that child classes are not copyable.
        Uncopyable(const Uncopyable& other);
        Uncopyable& operator=(const Uncopyable& other);
    }; //\class Uncopyable

    struct op_iter
    {
        const OperatingPoint op_;
        size_t al_index_iter_;
        size_t ilqg_index_iter;
        std::vector<Strategy> strategy_iter;
        std::vector<std::vector<QuadraticCostApproximation>> cost_quad_iter_;
        std::vector<LinearDynamics> linear_dynamics_iter;

        op_iter(const OperatingPoint& op, const size_t al_iter, const size_t ilqg_iter, const std::vector<Strategy>& current_strategy, const std::vector<std::vector<QuadraticCostApproximation>>& cost_quad, const std::vector<LinearDynamics> linear_dynamics):
         op_(op),al_index_iter_(al_iter),ilqg_index_iter(ilqg_iter), strategy_iter(current_strategy), cost_quad_iter_(cost_quad), linear_dynamics_iter(linear_dynamics)
        {
            
        }
    };
    

    class SolverLog : private Uncopyable
    {
    public:
        ~SolverLog() = default;
        SolverLog() {}

        //Add a new solver iterate for cost_quad
        void addSolverIterate(const OperatingPoint& operating_point,
                              const std::vector<Strategy>& strategies,
                              const std::vector<double>& total_costs,
                              const double& cumulative_runtime,
                              const bool& was_converged,
                              const std::vector<Eigen::MatrixXd>& covariance,
                              const std::vector<std::vector<QuadraticCostApproximation>>& cost_quad,
                              const size_t al_iter, const size_t ilqg_iter,
                              const std::vector<LinearDynamics>& linear_dynamics
                              )
        {
            addSolverIterate(operating_point, strategies, total_costs, cumulative_runtime, was_converged,covariance);
            cost_quad_.push_back(cost_quad);
            op_iter p = op_iter(operating_point,al_iter,ilqg_iter,strategies,cost_quad,linear_dynamics);
            //std::cout<<"op iter's al iter is: "<<p.al_index_iter_<<std::endl;
            collection_iter_.push_back(p);
            //std::cout<<"new op iter's al iter is: "<<collection_iter_.back().al_index_iter_<<std::endl;
            
        }


        // Add a new solver iterate for voariance
        void addSolverIterate(const OperatingPoint& operating_point,
                              const std::vector<Strategy>& strategies,
                              const std::vector<double>& total_costs,
                              const double& cumulative_runtime,
                              const bool& was_converged,
                              const std::vector<Eigen::MatrixXd>& covariance
                              )
        {
            addSolverIterate(operating_point, strategies, total_costs, cumulative_runtime, was_converged);
            covariances_.push_back(covariance);
        }

        void addSolverIterate(const OperatingPoint& operating_point,
                              const std::vector<Strategy>& strategies,
                              const std::vector<double>& total_costs,
                              const double& cumulative_runtime,
                              const bool& was_converged)
        {
            operating_points_.push_back(operating_point);
            strategies_.push_back(strategies);
            total_player_costs_.push_back(total_costs);
            cumulative_runtimes_.push_back(cumulative_runtime);
            was_converged_.push_back(was_converged);
        }

        // Add a whole other log.
        void addLog(const SolverLog& log)
        {
            for (size_t ii = 0; ii < log.getNumIterates(); ii++)
            {
                if(log.covariances_.empty())
                {
                    addSolverIterate(log.operating_points_[ii], log.strategies_[ii],
                                     log.total_player_costs_[ii],
                                     log.cumulative_runtimes_[ii], log.was_converged_[ii]);
                }
                else
                {
                    addSolverIterate(log.operating_points_[ii], log.strategies_[ii],
                                     log.total_player_costs_[ii],
                                     log.cumulative_runtimes_[ii], log.was_converged_[ii], log.covariances_[ii]);
                }
            }
        }

        // Save
        bool makeDirectory(const std::string& directory_name) const;
        bool save(bool only_last_trajectory, const std::string& experiment_name) const;
        bool save(bool only_last_trajectory,
                  const std::vector<std::string>& column_names,
                  const std::string& experiment_name) const;

        // Accessors.
        bool checkConverged() const { return was_converged_.back(); }
        bool checkConverged(size_t idx) const { return was_converged_[idx]; }

        unsigned int getNumPlayers() const { return strategies_[0].size(); }
        size_t getNumIterates() const { return operating_points_.size(); }
        std::vector<double> getTotalCosts() const { return total_player_costs_.back(); }

        const std::vector<Strategy>& getInitialStrategies() const
        {
            return strategies_.front();
        }

        const OperatingPoint& getInitialOperatingPoint() const
        {
            return operating_points_.front();
        }

        const std::vector<Strategy>& getFinalStrategies() const
        {
            return strategies_.back();
        }

        const OperatingPoint& getFinalOperatingPoint() const
        {
            assert(!operating_points_.empty());
            return operating_points_.back();
        }

        const std::vector<Eigen::MatrixXd>& getFinalCovariance() const
        {
            assert(!covariances_.empty());
            return covariances_.back();
        }

        Eigen::VectorXd getState(const size_t& iterate, const size_t& time_index) const
        {
            return operating_points_[iterate].xs[time_index];
        }

        double getState(const size_t& iterate, const size_t& time_index, const int& dim) const
        {
            return operating_points_[iterate].xs[time_index](dim);
        }

        Eigen::VectorXd getControl(const size_t& iterate, const size_t& time_index, const unsigned int& player) const
        {
            return operating_points_[iterate].us[time_index][player];
        }

        double getControl(const size_t& iterate, const size_t& time_index, const unsigned int& player, const int& dim) const
        {
            return operating_points_[iterate].us[time_index][player](dim);
        }

        std::vector<std::vector<QuadraticCostApproximation>> getFinalcost_quad()
        {
            return cost_quad_.back();
        }

        const OperatingPoint get_operating_point_iter(const int num_iter)
        {
            return operating_points_[num_iter];
        }

        const op_iter getcollection(size_t num_iter)
        {
            return collection_iter_[num_iter];
        }

        size_t get_num_collection() {return collection_iter_.size();}

        std::vector<Eigen::MatrixXd> getPs(const size_t& iterate, const size_t& time_index) const;
        std::vector<Eigen::VectorXd> getAlphas(const size_t& iterate, const size_t& time_index) const;
        Eigen::MatrixXd getP(const size_t& iterate, const size_t& time_index, const unsigned int& player) const;
        Eigen::VectorXd getAlpha(const size_t& iterate, const size_t& time_index, const unsigned int& player) const;


    private:
        // Operating points, strategies, total costs, and cumulative runtime indexed
        // by solver iterate.
        std::vector<OperatingPoint> operating_points_;
        std::vector<std::vector<Strategy>> strategies_;
        std::vector<std::vector<double>> total_player_costs_;
        std::vector<double> cumulative_runtimes_;
        std::vector<bool> was_converged_;
        std::vector<std::vector<Eigen::MatrixXd>> covariances_;
        std::vector<std::vector<std::vector<QuadraticCostApproximation>>> cost_quad_;
        std::vector<op_iter> collection_iter_;
        
        
    };
}

#endif //GAME_PLANNER_SOLVER_LOG_H
