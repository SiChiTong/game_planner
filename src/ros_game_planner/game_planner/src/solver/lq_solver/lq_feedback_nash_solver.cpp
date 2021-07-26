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
// Core LQ game solver from Basar and Olsder, "Preliminary Notation for
// Corollary 6.1" (pp. 279). All notation matches the text, though we
// shall assume that `c` (additive drift in dynamics) is always `0`, which
// holds because these dynamics are for delta x, delta us.
// Also, we have modified terms slightly to account for linear terms in the
// stage cost for control, i.e.
//       control penalty i = 0.5 \sum_j du_j^T R_ij (du_j + 2 r_ij)
//
// Solve a time-varying, finite horizon LQ game (finds closed-loop Nash
// feedback strategies for both players).
//
// Assumes that dynamics are given by
//           ``` dx_{k+1} = A_k dx_k + \sum_i Bs[i]_k du[i]_k ```
//
// Returns strategies Ps, alphas.
//
///////////////////////////////////////////////////////////////////////////////
//checked: 1
#include "solver/lq_solver/lq_feedback_nash_solver.h"

namespace game_planner
{
    std::vector<Strategy> LQFeedbackNashSolver::solve(const std::vector<LinearDynamics> &linearization,
                                                      const std::vector<std::vector<QuadraticCostApproximation>> &quadraticization,
                                                      const Eigen::VectorXd &x0,
                                                      std::vector<Eigen::VectorXd> *delta_xs,
                                                      std::vector<std::vector<Eigen::VectorXd>> *costates)
    {
        assert(linearization.size() == num_time_steps_);
        assert(quadraticization.size() == num_time_steps_);

        // Make sure delta_xs and costates are the right size
        if (delta_xs) assert(costates);
        if (costates) assert(delta_xs);
        if (delta_xs)
        {
            delta_xs->resize(num_time_steps_);
            costates->resize(num_time_steps_);
            for (size_t kk = 0; kk < num_time_steps_; kk++)
            {
                (*delta_xs)[kk].resize(xdim_);
                (*costates)[kk].resize(num_players_);
                for (unsigned int ii = 0; ii < num_players_; ii++)
                    (*costates)[kk][ii].resize(xdim_);
            }
        }

        // List of player-indexed strategies (each of which is a time-indexed
        // affine state error-feedback controller).
        std::vector<Strategy> strategies;
        for (unsigned int ii = 0; ii < num_players_; ii++)
            strategies.emplace_back(num_time_steps_, xdim_, udims_[ii]);

        // Initialize Zs and zetas at the *final time*.   Zs:Z_t^{i} zetas: ζi. The value function at the final time is known.
        for (unsigned int ii = 0; ii < num_players_; ii++)
        {
            Zs_[num_time_steps_ - 1][ii] = quadraticization.back()[ii].state_.hess_;
            zetas_[num_time_steps_ - 1][ii] = quadraticization.back()[ii].state_.grad_;
        }

        // Work backward in time and solve the dynamic program.
        // NOTE: time starts from the second-to-last entry since we'll treat the final
        // entry as a terminal cost as in Basar and Olsder, ch. 6.
        for (int time_id = num_time_steps_ - 2; time_id >= 0; time_id--)
        {
            // Unpack linearization and quadraticization at this time step.
            const auto& lin = linearization[time_id];
            const auto& quad = quadraticization[time_id];

            // Populate coupling matrix S for linear matrix equation to determine X (Ps
            // and alphas).
            // NOTE: S is generally dense and asymmetric, though it is symmetric if all
            // players have the same Z.
            int cumulative_udim_row = 0;
            for (unsigned int ego_player_id = 0; ego_player_id < num_players_; ego_player_id++)
            {
                // Intermediate variable to store B_{t}^[ii]' * Z_{t+1}^[ii].
                const Eigen::MatrixXd BiZi = lin.Bs[ego_player_id].transpose() * Zs_[time_id + 1][ego_player_id];

                int cumulative_udim_col = 0;
                for (unsigned int jj = 0; jj < num_players_; jj++)
                {   
                    //Block of size (p,q), starting at (i,j) matrix.block(i,j,p,q);
                    Eigen::Ref<Eigen::MatrixXd> S_block = S_.block(cumulative_udim_row, cumulative_udim_col,
                                                                   udims_[ego_player_id], udims_[jj]);

                    if (ego_player_id == jj)
                    {
                        // Does player ii's cost depend upon player jj's control?
                        const auto control_iter = quad[ego_player_id].control_.find(ego_player_id);

                        //Question: Why do we need this assert? Why we can't have these two term equal?
                        assert(control_iter != quad[ego_player_id].control_.end());

                        //For ->second,see "https://stackoverflow.com/questions/15451287/what-does-iterator-second-mean"

                        S_block = BiZi * lin.Bs[ego_player_id] + control_iter->second.hess_;
                    }
                    else
                        S_block = BiZi * lin.Bs[jj];

                    // Increment cumulative_udim_col.
                    cumulative_udim_col += udims_[jj];
                }

                // Set appropriate blocks of Y (S X = Y).
                Y_.block(cumulative_udim_row, 0, udims_[ego_player_id], xdim_) = BiZi * lin.A;

                //Block containing n elements, starting at position i *	vector.segment(i,n);
                //Question: Why do we have the quad[ego_player_id].control_.at(ego_player_id).grad_ term?
                Y_.col(xdim_).segment(cumulative_udim_row, udims_[ego_player_id]) =
                        lin.Bs[ego_player_id].transpose() * zetas_[time_id + 1][ego_player_id] + quad[ego_player_id].control_.at(ego_player_id).grad_;

                // Increment cumulative_udim_row.
                cumulative_udim_row += udims_[ego_player_id];
            }

            // Solve linear matrix equality S X = Y.
            // NOTE: not 100% sure that this avoids dynamic memory allocation.
            X_ = S_.householderQr().solve(Y_);

            // Set strategy at current time step.
            for (unsigned int player_id = 0; player_id < num_players_; player_id++)
            {
                strategies[player_id].Ps[time_id] = Ps_[player_id];
                strategies[player_id].alphas[time_id] = alphas_[player_id];
            }

            // Compute F and beta.
            F_ = lin.A;
            beta_ = Eigen::VectorXd::Zero(xdim_);
            for (unsigned int player_id = 0; player_id < num_players_; player_id++)
            {
                F_ -= lin.Bs[player_id] * Ps_[player_id];
                beta_ -= lin.Bs[player_id] * alphas_[player_id];
            }

            // Update Zs and zetas.
            for (unsigned int player_id = 0; player_id < num_players_; player_id++)
            {
                //Looks like there is a typo in the note. (one more transpose in the note). Double check it.
                zetas_[time_id][player_id] = (F_.transpose() * (zetas_[time_id + 1][player_id] + Zs_[time_id + 1][player_id] * beta_)
                                  + quad[player_id].state_.grad_).eval();

                // Use eval() to finalize the value from Eigen Object
                Zs_[time_id][player_id] = (F_.transpose() * Zs_[time_id + 1][player_id] * F_ + quad[player_id].state_.hess_).eval();

                // Add terms for nonzero Rijs.
                for (const auto& Rij_entry : quad[player_id].control_)
                {
                    const unsigned int jj = Rij_entry.first;
                    const Eigen::MatrixXd& Rij = Rij_entry.second.hess_;
                    const Eigen::VectorXd& rij = Rij_entry.second.grad_;
                    zetas_[time_id][player_id] += Ps_[jj].transpose() * (Rij * alphas_[jj] - rij);
                    Zs_[time_id][player_id] += Ps_[jj].transpose() * Rij * Ps_[jj];
                }
            }
        }

        // Maybe compute delta_xs and costates forward in time.
        //These codes looks like doesn't work.
        Eigen::VectorXd x_star = x0;
        Eigen::VectorXd last_x_star;
        for (size_t kk = 0; kk < num_time_steps_; kk++)
        {
            if (delta_xs)
            {
                (*delta_xs)[kk] = x_star;
                for (unsigned int ii = 0; ii < num_players_; ii++)
                {
                    if (kk < num_time_steps_ - 1)
                        (*costates)[kk][ii] = Zs_[kk + 1][ii] * x_star + zetas_[kk + 1][ii];
                    else
                        (*costates)[kk][ii].setZero();
                }
            }

            // Unpack linearization at this time step.
            const auto& lin = linearization[kk];

            // Compute optimal x.
            last_x_star = x_star;
            x_star = lin.A * last_x_star;
            for (unsigned int ii = 0; ii < num_players_; ii++)
                x_star -= lin.Bs[ii] * strategies[ii].alphas[kk];
        }

        return strategies;
    }
}