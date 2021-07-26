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
#include "solver/solver_log.h"

namespace game_planner
{
    inline std::vector<Eigen::MatrixXd> SolverLog::getPs(const size_t& iterate,
                                                         const size_t& time_index) const
    {
        std::vector<Eigen::MatrixXd> Ps(strategies_[iterate].size());
        for (unsigned int ii = 0; ii < Ps.size(); ii++)
            Ps[ii] = getP(iterate, time_index, ii);
        return Ps;
    }

    inline std::vector<Eigen::VectorXd> SolverLog::getAlphas(const size_t& iterate,
                                                          const size_t& time_index) const
    {
        std::vector<Eigen::VectorXd> alphas(strategies_[iterate].size());
        for (unsigned int ii = 0; ii < alphas.size(); ii++)
            alphas[ii] = getAlpha(iterate, time_index, ii);
        return alphas;
    }

    inline Eigen::MatrixXd SolverLog::getP(const size_t& iterate,
                                        const size_t& time_index,
                                        const unsigned int& player) const
    {
        return strategies_[iterate][player].Ps[time_index];
    }

    inline Eigen::VectorXd SolverLog::getAlpha(const size_t& iterate,
                                            const size_t& time_index,
                                            const unsigned int& player) const
    {
        return strategies_[iterate][player].alphas[time_index];
    }

    bool SolverLog::makeDirectory(const std::string& directory_name) const
    {
        if (mkdir(directory_name.c_str(), 0777) == -1)
            return false;
        return true;
    }

    bool SolverLog::save(bool only_last_trajectory,
                         const std::string& experiment_name) const
    {
        return save(only_last_trajectory, std::vector<std::string>(), experiment_name);
    }

    bool SolverLog::save(bool only_last_trajectory,
                         const std::vector<std::string>& column_names,
                         const std::string& experiment_name) const
    {
        // Making top-level directory
        const std::string dir_name = std::string(ILQGAMES_LOG_DIR) + "/" + experiment_name;

        // Check if the directory exist
        struct stat buffer;
        if(stat(dir_name.c_str(), &buffer) != 0)
        {
            std::cout << "The directory is not exist" << std::endl;
            if (!makeDirectory(dir_name))
            {
                std::cerr << "Cannot Make Directory to " << dir_name << std::endl;
                return false;
            }
        }

        size_t start = 0;
        if (only_last_trajectory) start = operating_points_.size() - 1;

        for (size_t ii = start; ii < operating_points_.size(); ii++)
        {
            const auto& op = operating_points_[ii];
            const std::string sub_dir_name = dir_name + "/" + std::to_string(ii);
            if(stat(sub_dir_name.c_str(), &buffer) != 0)
                if(!makeDirectory(sub_dir_name))
                    return false;

            // Dump initial time.
            std::ofstream file;

            // Output Xs to CSV
            file.open(sub_dir_name + "/xs.csv");
            if(!column_names.empty()){
                for(unsigned int i=0; i<column_names.size()-1; ++i)
                    file << column_names[i] << ",";
                file << column_names[column_names.size()-1] << std::endl;
            }
            for (const auto& x : op.xs) {
                for(int i=0; i<x.size()-1; ++i)
                    file << x(i) << ",";
                file << x(x.size()-1) << std::endl;
            }
            file.close();

            // Dump cumulative runtimes.
            file.open(sub_dir_name + "/cumulative_runtimes.txt");
            file << cumulative_runtimes_[ii] << std::endl;
            file.close();

            // Dump cumulative costs.
            file.open(sub_dir_name + "/cumulative_cost.csv");
            for(size_t player_id=0; player_id<getNumPlayers()-1; ++player_id)
                file << "cost" + std::to_string(player_id) << ",";
            file << "cost" + std::to_string(getNumPlayers()-1) << std::endl;
            for(size_t player_id=0; player_id<getNumPlayers()-1; ++player_id)
                file << total_player_costs_[ii][player_id] << ",";
            file << total_player_costs_[ii][getNumPlayers()-1] << std::endl;
            file.close();

            // Dump us.
            std::vector<std::ofstream> files(operating_points_[ii].us.front().size());
            for (size_t jj = 0; jj < files.size(); jj++)
                files[jj].open(sub_dir_name + "/u" + std::to_string(jj) + ".txt");

            for (size_t kk = 0; kk < op.us.size(); kk++)
            {
                assert(files.size()==op.us[kk].size());
                for (size_t jj = 0; jj < files.size(); jj++)
                    files[jj] << op.us[kk][jj].transpose() << std::endl;

            }
            for (size_t jj = 0; jj < files.size(); jj++)
                files[jj].close();
        }

        return true;
    }


}
