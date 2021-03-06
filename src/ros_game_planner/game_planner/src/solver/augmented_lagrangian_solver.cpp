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
 * Authors: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// Solver that implements an augmented Lagrangian method. For reference on these
// methods, please refer to Chapter 17 of Nocedal and Wright or the ALTRO paper:
// https://bjack205.github.io/assets/ALTRO.pdf.
//
///////////////////////////////////////////////////////////////////////////////
//checked:1 , q

#include "solver/augmented_lagrangian_solver.h"

namespace game_planner
{
    bool AugmentedLagrangianSolver::solve(const std::vector<Strategy> &initial_strategies,
                                          const std::shared_ptr<SolverLog> &solver_log)
    {
        // 1. Solve the unconstrained problem
        bool unconstrained_success = unconstrained_solver_->solve(initial_strategies, solver_log);

        // 2. Check if the problem is constrained or unconstrained
        //    If the problem is unconstrained we will terminate the calculation here
        if (!unconstrained_solver_->getProblem().isConstrained())
        {
            std::cout << "[" << solver_name_ << "]: This problem is unconstrained" << std::endl;
            return unconstrained_success;
        }

        // 3. Start Main Loop and do computation until convergence
        double max_constraint_error = constants::kInfinity;
        std::string max_constraint_name;
        size_t num_iter = 0;
        
        //std::cout<<"max_constraint_error, first one "<<(max_constraint_error )<<std::endl;
        //Something becomes crazy between iteration 44 and iteration 45
        while (num_iter < 46)//params_.max_al_solver_iters)
        {
            // 4. Check the constraint and update the augmented lagrangian parameter(This can be parallelize)
            //std::cout<<"hello world!"<<std::endl;
            num_iter++;   
            std::cout<<"current iteration is: "<<num_iter<<std::endl;
            max_constraint_error = -constants::kInfinity;
            const OperatingPoint& op = solver_log->getFinalOperatingPoint();
            for (auto& pc : unconstrained_solver_->getProblem().getPlayerCosts())
            {
                for (size_t kk = 0; kk < op.xs.size(); kk++)
                {
                    const auto& x = op.xs[kk];
                    const auto& us = op.us[kk];

                    // Scale each lambda.
                    for (const auto& constraint : pc.getStateConstraints())
                    {
                        const double constraint_error = constraint->evaluate(kk, x);
                        constraint->incrementLambda(kk, constraint_error);

                        //if (constraint->get_name() == "ObsAvoidanceConstraint")
                        //{
                        //    std::cout<<"Obstacle avodiance constraint error is: "<<constraint_error<<std::endl;
                        //}

                        if (constraint_error > max_constraint_error)
                        {
                            max_constraint_error = constraint_error;
                            max_constraint_name  = constraint->get_name();
                        }


                        //max_constraint_error = std::max(max_constraint_error, constraint_error);
                        
                    }

                    for (const auto& pair : pc.getControlConstraints())
                    {
                        const double constraint_error = pair.second->evaluate(kk, us[pair.first]);
                        pair.second->incrementLambda(kk, constraint_error);

                        if (constraint_error > max_constraint_error)
                        {
                            max_constraint_error = constraint_error;
                            max_constraint_name  = pair.second->get_name();
                        }

                        //max_constraint_error = std::max(max_constraint_error, constraint_error);
                    }
                }
                
            }
            //std::cout<<"max_constraint_error, first one "<<(max_constraint_error )<<std::endl;
            //if (max_constraint_error<params_.constraint_error_tolerance) {break;}
            // 5. Scale mu. (This is done for every iteration)
            BaseALConstraint::scaleMu(params_.geometric_mu_scaling);

            // 6. Start New Computation with new parameters
            unconstrained_success = unconstrained_solver_->solve(solver_log->getFinalOperatingPoint(), solver_log, num_iter);
            
            if(!unconstrained_success)
                std::cout << "Calculation False and down scaling the value" << std::endl;

            // 7. If we failed then downscale all lambdas and mus for next iteration.
            if (!unconstrained_success)
            {
                for (auto& pc : unconstrained_solver_->getProblem().getPlayerCosts())
                {
                    for (const auto& constraint : pc.getStateConstraints())
                        constraint->scaleLambdas(params_.geometric_lambda_downscaling);
                    for (const auto& pair : pc.getControlConstraints())
                        pair.second->scaleLambdas(params_.geometric_lambda_downscaling);
                }

                BaseALConstraint::scaleMu(params_.geometric_mu_downscaling);
            }
            
            
            
            
        } // end while


        //std::vector<op_iter> collection_iter = solver_log->getcollection();
        int size = static_cast<int>(solver_log->get_num_collection());


        std::cout<<"The size of collection is: "<<size<<std::endl;
        op_iter final_collection = solver_log->getcollection(size-1);
        op_iter f1_collection = solver_log -> getcollection(size-2);
        op_iter f2_collection = solver_log -> getcollection(size-3);
        //std::vector<std::vector<QuadraticCostApproximation>> final_cost_quad = solver_log->getFinalcost_quad();
        
        for(size_t time_step =0; time_step<time::kNumTimeSteps;time_step++)
        {   
            //std::cout<<"current time step is: "<<time_step<<std::endl;
            //std::cout<<"[1] al index : "<<f1_collection.al_index_iter_<<", ilqg index: "<<f1_collection.ilqg_index_iter<<std::endl;
            //std::cout<<"[2] al index  :"<<f2_collection.al_index_iter_<<", iqlg index: "<<f2_collection.ilqg_index_iter<<std::endl;
            //std::cout<<"[3] al index :"<<final_collection.al_index_iter_<<", ilgq iter: "<<final_collection.ilqg_index_iter<<std::endl;
            //std::cout<<"The difference of alphas for player 1:[1]-[2] "<<std::endl;
            //std::cout<<"Player 0's alpha for [1]"<<std::endl;
            //std::cout<<f1_collection.strategy_iter[0].alphas[time_step]<<std::endl;
            //std::cout<<"Player 0's alpha for [2]"<<std::endl;
            //std::cout<<f2_collection.strategy_iter[0].alphas[time_step]<<std::endl;
            //std::cout<<"Player 0's costquad state grad for [3] "<<std::endl;
            //std::cout<<f1_collection.cost_quad_iter_[time_step][0].state_.grad_<<std::endl;
            //std::cout<<"Player 0's costquad state hessian for [3] "<<std::endl;
            //std::cout<<f1_collection.cost_quad_iter_[time_step][0].state_.hess_<<std::endl;
            //std::cout<<"Player 0's costquad control grad for itself at [3] "<<std::endl;
            //std::cout<<f1_collection.cost_quad_iter_[time_step][0].control_.at(0).grad_<<std::endl;
            //std::cout<<"Player 0's costquad control hessian for itself at [3] "<<std::endl;
            //std::cout<<f1_collection.cost_quad_iter_[time_step][0].control_.at(0).hess_<<std::endl;
            //std::cout<<"player 0's P for [3] "<<std::endl;
            //std::cout<<final_collection.strategy_iter[0].Ps[time_step]<<std::endl;
            //std::cout<<"Player 0's alpha for [3]"<<std::endl;
            //std::cout<<final_collection.strategy_iter[0].alphas[time_step]<<std::endl;
            //std::cout<<"Player 0's costquad state grad for [2] "<<std::endl;
            //std::cout<<f2_collection.cost_quad_iter_[time_step][0].state_.grad_<<std::endl;
            //std::cout<<"Player 0's costquad state hessian for [2] "<<std::endl;
            //std::cout<<f2_collection.cost_quad_iter_[time_step][0].state_.hess_<<std::endl;
            //std::cout<<"Player 0's costquad control grad for itself at [2] "<<std::endl;
            //std::cout<<f2_collection.cost_quad_iter_[time_step][0].control_.at(0).grad_<<std::endl;
            //std::cout<<"Player 0's costquad control hessian for itself at [2] "<<std::endl;
            //std::cout<<f2_collection.cost_quad_iter_[time_step][0].control_.at(0).hess_<<std::endl;
            //std::cout<<"Linear dynamics for [2], A: "<<std::endl;
            //std::cout<<f2_collection.linear_dynamics_iter[time_step].A<<std::endl;
            //std::cout<<"Linear dynamics for [2], B: "<<std::endl;
            //std::cout<<f2_collection.linear_dynamics_iter[time_step].Bs[0]<<std::endl;
            //std::cout<<f2_collection.linear_dynamics_iter[time_step].Bs[1]<<std::endl;




            //std::cout<<"Player 1's alpha for [1]"<<std::endl;
            //std::cout<<f1_collection.strategy_iter[1].alphas[time_step]<<std::endl;
            //std::cout<<"Player 1's alpha for [2]"<<std::endl;
            //std::cout<<f2_collection.strategy_iter[1].alphas[time_step]<<std::endl;
            //std::cout<<"Player 1's apha for [3]"<<std::endl;
            //std::cout<<final_collection.strategy_iter[1].alphas[time_step]<<std::endl;
            //std::cout<<"player 1's P for [3] "<<std::endl;
            //std::cout<<final_collection.strategy_iter[1].Ps[time_step]<<std::endl;
            //std::cout<<"Player 1's costquad state grad for [3] "<<std::endl;
            //std::cout<<f1_collection.cost_quad_iter_[time_step][1].state_.grad_<<std::endl;
            //std::cout<<"Player 1's costquad state hessian for [3] "<<std::endl;
            //std::cout<<f1_collection.cost_quad_iter_[time_step][1].state_.hess_<<std::endl;
            //std::cout<<"Player 1's costquad control grad for itself at [3] "<<std::endl;
            //std::cout<<f1_collection.cost_quad_iter_[time_step][1].control_.at(1).grad_<<std::endl;
            //std::cout<<"Player 1's costquad control hessian for itself at [3] "<<std::endl;
            //std::cout<<f1_collection.cost_quad_iter_[time_step][1].control_.at(1).hess_<<std::endl;            
            //std::cout<<f2_collection.strategy_iter[0].alphas[time_step]-f1_collection.strategy_iter[0].alphas[time_step]<<std::endl;
            //std::cout<<"The difference of controls for player 1:[1]-[2] "<<std::endl;
            //std::cout<<f2_collection.op_.us[time_step][0]-f1_collection.op_.us[time_step][0]<<std::endl;
            //std::cout<<"The difference of controls for player 2:[1]-[2] "<<std::endl;
            //std::cout<<f2_collection.op_.us[time_step][1]-f1_collection.op_.us[time_step][1]<<std::endl;
            //std::cout<<"Player 1's costquad state grad for [2] "<<std::endl;
            //std::cout<<f2_collection.cost_quad_iter_[time_step][1].state_.grad_<<std::endl;
            //std::cout<<"Player 1's costquad state hessian for [2] "<<std::endl;
            //std::cout<<f2_collection.cost_quad_iter_[time_step][1].state_.hess_<<std::endl;
            //std::cout<<"Player 1's costquad control grad for itself at [2] "<<std::endl;
            //std::cout<<f2_collection.cost_quad_iter_[time_step][1].control_.at(1).grad_<<std::endl;
            //std::cout<<"Player 1's costquad control hessian for itself at [2] "<<std::endl;
            //std::cout<<f2_collection.cost_quad_iter_[time_step][1].control_.at(1).hess_<<std::endl;
            //std::cout<<"player 0's P for [3] "<<std::endl;
            //std::cout<<final_collection.strategy_iter[0].Ps[time_step]<<std::endl;
            //std::cout<<"Player 0's alpha for [3]"<<std::endl;
            //std::cout<<final_collection.strategy_iter[0].alphas[time_step]<<std::endl;
            //std::cout<<"Player 1's apha for [3]"<<std::endl;
            //std::cout<<final_collection.strategy_iter[1].alphas[time_step]<<std::endl;
            //std::cout<<"player 1's P for [3] "<<std::endl;
            //std::cout<<final_collection.strategy_iter[1].Ps[time_step]<<std::endl;
            if(time_step == 18)
            {
                
            std::cout<<"[2] al index :"<<f1_collection.al_index_iter_<<", ilgq iter: "<<f1_collection.ilqg_index_iter<<std::endl;
            std::cout<<"current time step is: "<<time_step<<std::endl;
            std::cout<<"player 0's P for [2] "<<std::endl;
            std::cout<<f1_collection.strategy_iter[0].Ps[time_step]<<std::endl;
            std::cout<<"Player 0's alpha for [2]"<<std::endl;
            std::cout<<f1_collection.strategy_iter[0].alphas[time_step]<<std::endl;
            std::cout<<"Player 1's apha for [2]"<<std::endl;
            std::cout<<f1_collection.strategy_iter[1].alphas[time_step]<<std::endl;
            std::cout<<"player 1's P for [2] "<<std::endl;
            std::cout<<f1_collection.strategy_iter[1].Ps[time_step]<<std::endl;


            }
        }

        //std::cout<<"max_constraint_error: "<<max_constraint_error<<"   max_constraint name: "<<max_constraint_name<<std::endl;
        //std::cout<<"constraint_tolerance: "<<params_.constraint_error_tolerance<<std::endl;
                    //print out all the constraint values
        //const auto& op = solver_log->getFinalOperatingPoint();
        const auto& op = f2_collection.op_;
        for(auto& pc : unconstrained_solver_->getProblem().getPlayerCosts())
        {
            for (size_t kk = 0; kk < op.xs.size(); kk++)
            {
                const auto& x = op.xs[kk];
                const auto& us = op.us[kk];
                    
                for (const auto& constraint: pc.getStateConstraints())
                {
                    std::string constraint_name = constraint->get_name();
                    double constraint_value = constraint->evaluate(kk,x);
                    if (constraint_value>=0)
                    {
                        std::cout<<constraint_name<<" at step "<<kk<<"'s value is "<<constraint_value<<std::endl;
                    }
                        //std::cout<<constraint_name<<" at step "<<kk<<"'s value is "<<constraint_value<<std::endl;
                }
            }
        }

        //std::cout<<"max_constraint_error is: "<<max_constraint_error<<std::endl;
        //std::cout<<"Number of iterations is: "<<num_iter<<std::endl;


        if(max_constraint_error > params_.constraint_error_tolerance)
            std::cout << "[" << solver_name_ << "]: Could not satisfy all constraints" << std::endl;

        // 8.Reset all multipliers.
        if (params_.reset_lambdas)
        {
            for (auto& pc : unconstrained_solver_->getProblem().getPlayerCosts())
            {
                for (const auto& constraint : pc.getStateConstraints())
                    constraint->scaleLambdas(constants::kDefaultLambda);
                for (const auto& pair : pc.getControlConstraints())
                    pair.second->scaleLambdas(constants::kDefaultLambda);
            }
        }

        if (params_.reset_mu) BaseALConstraint::getGlobalMu() = constants::kDefaultMu;

        std::cout<<"solver success: "<<unconstrained_success<<std::endl;
        
        return unconstrained_success;
    }
}
