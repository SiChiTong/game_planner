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
 * Authors:  Yutaka Shimizu
 */
///////////////////////////////////////////////////////////////////////////////
//
// Class For Iterative Linear Quadratic Gaussian Solver
//
///////////////////////////////////////////////////////////////////////////////
//checked: 1, q

#include "solver/iterative_solver/ilqg_solver.h"

namespace game_planner
{
    bool ILQGSolver::solve(const std::vector<Strategy>& initial_strategies,
                           const std::shared_ptr<SolverLog>& solver_log)

    {
        //assert(players_input_cov.size() == problem_->getNumPlayers());
        //assert(players_observation_cov.size() == problem_->getNumPlayers());

        // Things to keep track of during each iteration.
        size_t num_iterations = 0;
        bool has_converged = false;
        double elapsed = 0.0;

        // 1. Obtain Initial Operating Point (Initial xs[0] is already set)
        OperatingPoint current_operating_point = problem_->initializeOperatingPoint();
        Eigen::VectorXd x0 = problem_->getInitialState();

        // 2. Forward(calculate xs from time 1 to the end)
        current_operating_point = forward(current_operating_point, initial_strategies);

        // 3. Linearize dynamics about the new operating point
        std::vector<LinearDynamics> linear_dyn = computeLinearization(current_operating_point);

        // 4. Compute Covariance
        std::vector<Eigen::MatrixXd> covariances = computeCovariance(current_operating_point, linear_dyn);


        // 5. Compute total costs for initial trajectory
        std::vector<double> total_costs = computeTotalCosts(current_operating_point, covariances);

        // 6. Logging initial trajectory
        solver_log->addSolverIterate(current_operating_point, initial_strategies, total_costs,
                                     elapsed, has_converged, covariances);

        // Maintain delta_xs and costates for linesearch.
        std::vector<Eigen::VectorXd> delta_xs;
        std::vector<std::vector<Eigen::VectorXd>> costates;
        std::vector<Strategy> current_strategies = initial_strategies;

        // 5. Quadraticize costs before first iteration.
        std::vector<std::vector<QuadraticCostApproximation>> cost_quad
                                            = computeCostQuadraticization(current_operating_point, covariances);

        //Main loop
        //while(num_iterations < solver_params_.max_solver_iters && !has_converged)
        while(num_iterations < solver_params_ .max_solver_iters&& !has_converged)
        {
            // Start loop timer.
            timer_.Tic();

            // New iteration.
            num_iterations++;

            // 7. Solve LQ game.
            current_strategies = lq_solver_->solve(linear_dyn, cost_quad, x0, &delta_xs, &costates);

            // 8. Do Line Search (Update linear_dyn, cost_quad)
            if (!doLineSearch(delta_xs, costates,
                              linear_dyn, cost_quad, current_strategies,
                              current_operating_point, has_converged, covariances))
            {
                // Emit warning if exiting early.
                std::cerr << "Solver exited due to linesearch failure." << std::endl;
                return false;
            }

            // Compute total costs and check if we've converged.
            total_costs = computeTotalCosts(current_operating_point, covariances);

            // Record loop runtime.
            elapsed += timer_.Toc();

            // Log current iterate.
            solver_log->addSolverIterate(current_operating_point, current_strategies,
                                         total_costs, elapsed, has_converged, covariances);
        }

        return true;
    }

    std::vector<Eigen::MatrixXd> ILQGSolver::computeCovariance(const OperatingPoint &op,
                                                               const std::vector<LinearDynamics>& linear_dyn)
    {
        assert(op.xs.size() == linear_dyn.size());

        const int N = op.xs.size();
        const int state_size = linear_dyn.front().A.rows();
        //std::cout<<"state size is"<<state_size<<std::endl;
        //std::cout<<"number of players"<<problem_->getNumPlayers()<<std::endl;

        std::vector<Eigen::MatrixXd> input_covs = problem_->getInputCovariance();
        std::vector<Eigen::MatrixXd> observe_covs = problem_->getObservationCovariance();

        assert(input_covs.size() == problem_->getNumPlayers());
        assert(observe_covs.size() == problem_->getNumPlayers());

        std::vector<Eigen::MatrixXd> covariance(N, Eigen::MatrixXd::Zero(state_size, state_size));
        //covariance[0] = problem_->getInitialCovariance();
        covariance[0] = Eigen::MatrixXd::Identity(state_size,state_size);
        covariance[0](0,1) = 0.5;
        covariance[0] (1,0) = 0.5;
        covariance[0] (0,2) =0.3;
        covariance[0] (2,0) = 0.3;
        covariance[0] (7,8) =0.2;
        covariance[0] (8,7) = 0.2;
        covariance[0] (0,0) = 2;
        covariance[0] (1,1) = 1.1;
        covariance[0](2,2) = 0.3;
        covariance[0](3,3) = 0.4;
        covariance[0] (4,4) = 1.5;
        covariance[0](6,6) =0.7;
        covariance[0] (7,7) =0.4;
        covariance[0] (8,8) = 1.3;
        //std::cout<<"initial covariance"<<std::endl;
        //std::cout<<covariance[0]<<std::endl;
        //throw"trial";
        
        for(unsigned int player_id = 0; player_id < problem_->getNumPlayers(); ++player_id)
        {
            int start_xdim  = problem_->getDynamics()->getSubsystemStartDim(player_id);
            int player_xdim = problem_->getDynamics()->getSubsystemXDim(player_id);
            //std::cout<<"player_xdim is"<<player_xdim<<std::endl;

            int velocity_dim = problem_->getDynamics()->getSubsystemVelocityDimension(player_id);
            //std::cout<<"velocity_dim  is  "<<velocity_dim <<std::endl;
            assert(velocity_dim>=0);
            Eigen::MatrixXd I_eps = 1e-5*Eigen::MatrixXd::Identity(player_xdim,player_xdim);//Constant(player_xdim, player_xdim, 1e-1);
            //std::cout<<I_eps<<std::endl;
            //std::cout<<"input_covs["<<player_id<<"].rows =" << input_covs[player_id].rows() <<std::endl;
            //std::cout<<"input_covs["<<player_id<<"].cols =" << input_covs[player_id].cols() <<std::endl;
            //std::cout<<"linear_dyn[0].Bs["<<player_id<<"].rows =" << linear_dyn[0].Bs[player_id].rows() <<std::endl;
            //std::cout<<"linear_dyn[0].Bs["<<player_id<<"].cols =" << linear_dyn[0].Bs[player_id].cols() <<std::endl;
            //assert(input_covs[player_id].rows() == linear_dyn[0].Bs[player_id].rows());
            //assert(input_covs[player_id].cols() == linear_dyn[0].Bs[player_id].cols());
            observe_covs[player_id] = 0.2*Eigen::MatrixXd::Identity(player_xdim,player_xdim);
            for(int i=0; i<N-1; ++i)
            {
                const Eigen::MatrixXd A = linear_dyn[i].A.block(start_xdim, start_xdim, player_xdim, player_xdim);
                const Eigen::MatrixXd B = linear_dyn[i].Bs[player_id];

                //std::cout<<"A.rows = "<<A.rows()<<std::endl;
                //std::cout<<"A.cols = "<<A.cols()<<std::endl;
                //std::cout<<"covariance[i].rows = "<<covariance[i].rows()<<std::endl;
                //std::cout<<"covariance[i].cols = "<<covariance[i].cols()<<std::endl;
                //std::cout<<"player id"<<player_id<<std::endl;
                //std::cout<<"player_xdim is"<<player_xdim<<std::endl;
                //std::cout<<"start_xdim is"<<start_xdim<<std::endl;
                if(std::isnan(covariance[i](0,0)))

                {
                    throw"something is wrong!";
                }
                
                //Question: A: n_{player} x n_{player}, covariance: n x n, dimession mismatch? why mulitply B?
                //Estimate

                
                    //std::cout << "trial"<< std::endl;
                    //std::cout<<"start_xdim"<<start_xdim<<std::endl;
                    //std::cout<<"player_xdim"<<player_xdim<<std::endl;
                    //std::cout<<"input_covs["<<player_id<<"].rows =" << input_covs[player_id].rows() <<std::endl;

                    Eigen::MatrixXd cov_p = A * covariance[i].block(start_xdim,start_xdim,player_xdim,player_xdim) * A.transpose() + B.block(start_xdim,0,player_xdim,input_covs[player_id].rows()) * input_covs[player_id] * B.block(start_xdim,0,player_xdim,input_covs[player_id].rows()).transpose();
                    //std::cout<<"cov_p"<<std::endl;
                    //std::cout << cov_p << std::endl;
                //}catch (...) {
                    //std::cerr<<"Dimension Mismatch"<<std::endl;
                //}

                //Eigen::MatrixXd cov_p = A * covariance[i] * A.transpose() + B * input_covs[player_id] * B.transpose();


                //Update
                double velocity = op.xs[i](start_xdim+velocity_dim);
                //Question: Why we * std::pow(velocity,2)?
              
                //std::cout<<"oberseve_covs"<<observe_covs[player_id].rows()<<std::endl;
                //std::cout<<"oberseve_covs"<<observe_covs[player_id].cols()<<std::endl;
                //std::cout<<"observe_covs"<<observe_covs[player_id]<<std::endl;

                
                Eigen::MatrixXd kalman_gain = (cov_p+I_eps) * (cov_p + observe_covs[player_id] * std::pow(velocity,2)+I_eps).inverse();
                //std::cout<<"kalman gain"<<std::endl;
                //std::cout<<kalman_gain<<std::endl;
                //throw"Trial!";
                covariance[i+1].block(start_xdim,start_xdim,player_xdim,player_xdim) = cov_p - kalman_gain*cov_p;
                //std::cout<<"final covariance"<<std::endl;
                //std::cout<<covariance[i+1]<<std::endl;
               
            }
        }
        
        //throw"trial!";
       

        return covariance;
    }

    // Line Search
    bool ILQGSolver::doLineSearch(const std::vector<Eigen::VectorXd>& delta_xs,
                                  const std::vector<std::vector<Eigen::VectorXd>>& costates,
                                  std::vector<LinearDynamics>& linear_dyn,
                                  std::vector<std::vector<QuadraticCostApproximation>>& cost_quad,
                                  std::vector<Strategy>& strategies,
                                  OperatingPoint& current_operating_point,
                                  bool& has_converged,
                                  std::vector<Eigen::MatrixXd>& covariances)
    {
        
        
        //For no line search. use small step sizes. update current strategies, operating point,
        //costquad, linear dynamics, last merit function, has_converged
        if (!solver_params_.linesearch)
        {   
            //setup a small stepsize, try: 0.05 to 0.1 to 0.2
            double current_stepsize = 0.2;

            //scale the strategies
            scaleByLineSearchParam(current_stepsize, strategies);

            //Forward to update op
            const OperatingPoint last_operating_point(current_operating_point);

            current_operating_point = forward(last_operating_point, strategies);

            //Update linear dyanmics based on current operating point
            computeLinearization(current_operating_point, linear_dyn);

            //Update cost_quad based on current operating point
                       
            covariances = computeCovariance(current_operating_point, linear_dyn);

            
            computeCostQuadraticization(current_operating_point, covariances, cost_quad);

            //update current merit value
            const double current_merit_function_value = computeMeritFunction(current_operating_point, cost_quad);
            
            //update has_converged and last_merit_value
            has_converged = checkConverged(current_merit_function_value);
            last_merit_function_value_ = current_merit_function_value;

            
            return true;

        }
        
        // 1. Precompute expected decrease before we do anything else.
        double expected_decrease = computeExpectedDecrease(strategies, delta_xs, costates, linear_dyn, cost_quad);
        
        // 2. Scale ff term by Line Search Parameters
        scaleByLineSearchParam(solver_params_.initial_alpha_scaling, strategies);

        // 3. Forward
        const OperatingPoint last_operating_point(current_operating_point);
        double current_stepsize = solver_params_.initial_alpha_scaling;
        //double current_stepsize = 0.1;
        //std::cout<<"current_stepsize"<<current_stepsize;
        current_operating_point = forward(last_operating_point, strategies);

        // Keep reducing alphas until we satisfy the Armijo condition.
        for (size_t ii = 0; ii < solver_params_.max_backtracking_steps; ii++)
        {
            // 4. Compute Covariance regarding current operating point
            covariances = computeCovariance(current_operating_point, linear_dyn);

            // 5. Quadraticization
            computeCostQuadraticization(current_operating_point, covariances, cost_quad);

            // 6. Compute merit function value.
            const double current_merit_function_value = computeMeritFunction(current_operating_point, cost_quad);

            // 7. Check Armijo condition.
            if (checkArmijoCondition(current_merit_function_value, current_stepsize, expected_decrease))
            {
                // Success! Update cached terms and check convergence.
                has_converged = checkConverged(current_merit_function_value);
                last_merit_function_value_ = current_merit_function_value;

                // Update linearized dynamics
                computeLinearization(current_operating_point, linear_dyn);

                return true;
            }

            // 8. Update current_step_size
            current_stepsize *= solver_params_.geometric_alpha_scaling;

            // 9. Scale down the alphas and update operating point and linear dynamics. Try again
            scaleByLineSearchParam(solver_params_.geometric_alpha_scaling, strategies);
            current_operating_point = forward(last_operating_point, strategies);
            computeLinearization(current_operating_point, linear_dyn);
        }

        // Output a warning. Solver should revert to last valid operating point.
        std::cerr << "Exceeded maximum number of backtracking steps." << std::endl;
        return false;
    }

    std::vector<double> ILQGSolver::computeTotalCosts(const OperatingPoint& current_op,
                                                      const std::vector<Eigen::MatrixXd>& covariances) const
    {
        // Initialize
        std::vector<double> total_costs(problem_->getNumPlayers(), 0.0);

        // Compute Total costs
        // Accumulate costs.
        for (size_t kk = 0; kk < time::kNumTimeSteps; kk++) // For every time step
            for (size_t ii = 0; ii < problem_->getNumPlayers(); ii++) // For each Player
                total_costs[ii] += problem_->getPlayerCosts(ii).evaluate(kk, current_op.xs[kk], current_op.us[kk],
                                                                         covariances[kk], problem_->getInputCovariance());

        return total_costs;
    }

    std::vector<std::vector<QuadraticCostApproximation>>
                        ILQGSolver::computeCostQuadraticization(const OperatingPoint& op,
                                                                const std::vector<Eigen::MatrixXd>& covariances)
    {
        std::vector<std::vector<QuadraticCostApproximation>> q = initializeQuadraticCostApproximation();
        computeCostQuadraticization(op, covariances, q);

        return q;
    }

    void ILQGSolver::computeCostQuadraticization(const OperatingPoint &op,
                                                 const std::vector<Eigen::MatrixXd>& covariances,
                                                 std::vector<std::vector<QuadraticCostApproximation>>& q)
    {
        assert(q.size() == time::kNumTimeSteps);
        assert(q.front().size() == problem_->getNumPlayers());

        for (size_t time_step= 0; time_step < time::kNumTimeSteps; time_step++)
        {
            const auto& x = op.xs[time_step];
            const auto& us = op.us[time_step];

            // Quadraticize costs.
            for (unsigned int player_id = 0; player_id < problem_->getNumPlayers(); player_id++)
            {
                const PlayerCost& cost = problem_->getPlayerCosts(player_id);
                q[time_step][player_id] = cost.quadraticize(time_step, x, us, covariances[time_step],
                                                            problem_->getInputCovariance());
            }
        }
    }

    //Warm start version solve.
    bool ILQGSolver::solve(const OperatingPoint initial_op, const std::shared_ptr<SolverLog>& solver_log)

    {
        //assert(players_input_cov.size() == problem_->getNumPlayers());
        //assert(players_observation_cov.size() == problem_->getNumPlayers());

        // Things to keep track of during each iteration.
        size_t num_iterations = 0;
        bool has_converged = false;
        double elapsed = 0.0;

        // 1. Warm start, use the op obtained in last iteration as warm start;
        OperatingPoint current_operating_point = initial_op;
        Eigen::VectorXd x0 = problem_->getInitialState();

        
        // 3. Linearize dynamics about the new operating point
        std::vector<LinearDynamics> linear_dyn = computeLinearization(current_operating_point);

        // 4. Compute Covariance
        std::vector<Eigen::MatrixXd> covariances = computeCovariance(current_operating_point, linear_dyn);


        // 5. Compute total costs for initial trajectory
        std::vector<double> total_costs = computeTotalCosts(current_operating_point, covariances);

        // 6. Logging initial trajectory
        //solver_log->addSolverIterate(current_operating_point, initial_strategies, total_costs,
         //                            elapsed, has_converged, covariances);

        // Maintain delta_xs and costates for linesearch.
        std::vector<Eigen::VectorXd> delta_xs;
        std::vector<std::vector<Eigen::VectorXd>> costates;
        std::vector<Strategy> current_strategies;

        // 5. Quadraticize costs before first iteration.
        std::vector<std::vector<QuadraticCostApproximation>> cost_quad
                                            = computeCostQuadraticization(current_operating_point, covariances);

        //Main loop
        while(num_iterations < solver_params_.max_solver_iters && !has_converged)
        {
            // Start loop timer.
            timer_.Tic();

            // New iteration.
            num_iterations++;

            // 7. Solve LQ game.
            current_strategies = lq_solver_->solve(linear_dyn, cost_quad, x0, &delta_xs, &costates);

            // 8. Do Line Search (Update linear_dyn, cost_quad)
            if (!doLineSearch(delta_xs, costates,
                              linear_dyn, cost_quad, current_strategies,
                              current_operating_point, has_converged, covariances))
            {
                // Emit warning if exiting early.
                std::cerr << "Solver exited due to linesearch failure." << std::endl;
                return false;
            }

            // Compute total costs and check if we've converged.
            total_costs = computeTotalCosts(current_operating_point, covariances);

            // Record loop runtime.
            elapsed += timer_.Toc();

            // Log current iterate.
            solver_log->addSolverIterate(current_operating_point, current_strategies,
                                         total_costs, elapsed, has_converged, covariances);
        }

        return true;
    }

    //Debugging for trajactory shift version solve
    bool ILQGSolver::solve(const OperatingPoint initial_op, const std::shared_ptr<SolverLog>& solver_log, size_t al_num_iter)

    {
        //assert(players_input_cov.size() == problem_->getNumPlayers());
        //assert(players_observation_cov.size() == problem_->getNumPlayers());

        // Things to keep track of during each iteration.
        size_t num_iterations = 0;
        bool has_converged = false;
        double elapsed = 0.0;

        // 1. Warm start, use the op obtained in last iteration as warm start;
        OperatingPoint current_operating_point = initial_op;
        Eigen::VectorXd x0 = problem_->getInitialState();

        
        // 3. Linearize dynamics about the new operating point
        std::vector<LinearDynamics> linear_dyn = computeLinearization(current_operating_point);

        // 4. Compute Covariance
        std::vector<Eigen::MatrixXd> covariances = computeCovariance(current_operating_point, linear_dyn);


        // 5. Compute total costs for initial trajectory
        std::vector<double> total_costs = computeTotalCosts(current_operating_point, covariances);

        // 6. Logging initial trajectory
        //solver_log->addSolverIterate(current_operating_point, initial_strategies, total_costs,
         //                            elapsed, has_converged, covariances);

        // Maintain delta_xs and costates for linesearch.
        std::vector<Eigen::VectorXd> delta_xs;
        std::vector<std::vector<Eigen::VectorXd>> costates;
        std::vector<Strategy> current_strategies;

        // 5. Quadraticize costs before first iteration.
        std::vector<std::vector<QuadraticCostApproximation>> cost_quad
                                            = computeCostQuadraticization(current_operating_point, covariances);

        //Main loop
        size_t max_ilqg_iter = solver_params_.max_solver_iters;
        
        if (al_num_iter >= 45)
        {
            max_ilqg_iter = 100;
        }

        if(al_num_iter==46)
        {
            max_ilqg_iter = 90;
        }

        while(num_iterations < max_ilqg_iter && !has_converged)
        {
            // Start loop timer.
            timer_.Tic();

            // New iteration.
            num_iterations++;
            


            // 7. Solve LQ game.
            
            
            if(al_num_iter >= 45)
            {
                std::cout<<"current ilqg iter is: "<<num_iterations<<std::endl;
                current_strategies = lq_solver_->debug_solve(linear_dyn, cost_quad, x0,al_num_iter,num_iterations, &delta_xs, &costates);
            }
            else{
                current_strategies = lq_solver_->solve(linear_dyn, cost_quad, x0, &delta_xs, &costates);
            }


            // 8. Do Line Search (Update linear_dyn, cost_quad)
            if (!doLineSearch(delta_xs, costates,
                              linear_dyn, cost_quad, current_strategies,
                              current_operating_point, has_converged, covariances,al_num_iter,num_iterations))
            {
                // Emit warning if exiting early.
                std::cerr << "Solver exited due to linesearch failure." << std::endl;
                return false;
            }

            // Compute total costs and check if we've converged.
            total_costs = computeTotalCosts(current_operating_point, covariances);

            // Record loop runtime.
            elapsed += timer_.Toc();

            //std::cout<<"al iter is: "<<al_num_iter<<std::endl;
            //std::cout<<"ilqg iter is: "<<num_iterations<<std::endl;
            // Log current iterate.
            //solver_log->addSolverIterate(current_operating_point, current_strategies,
                                        // total_costs, elapsed, has_converged, covariances,cost_quad,al_num_iter,num_iterations,linear_dyn);
           if(al_num_iter>=45){ has_converged=false;}
           solver_log->addSolverIterate(current_operating_point, current_strategies,
                                         total_costs, elapsed, has_converged, covariances,cost_quad,al_num_iter,num_iterations,linear_dyn);
        }

        return true;
    }



    // Debug version Line Search
    bool ILQGSolver::doLineSearch(const std::vector<Eigen::VectorXd>& delta_xs,
                                  const std::vector<std::vector<Eigen::VectorXd>>& costates,
                                  std::vector<LinearDynamics>& linear_dyn,
                                  std::vector<std::vector<QuadraticCostApproximation>>& cost_quad,
                                  std::vector<Strategy>& strategies,
                                  OperatingPoint& current_operating_point,
                                  bool& has_converged,
                                  std::vector<Eigen::MatrixXd>& covariances,
                                  const size_t al_iter, const size_t ilqg_iter)
    {
        
        
        //For no line search. use small step sizes. update current strategies, operating point,
        //costquad, linear dynamics, last merit function, has_converged
        if (!solver_params_.linesearch)
        {   
             double current_stepsize;
            //setup a small stepsize, try: 0.05 to 0.1 to 0.2
           
            current_stepsize = 0.2;
            if(al_iter==45&&ilqg_iter>=4) {current_stepsize = 0.1;}
            

            //scale the strategies
            scaleByLineSearchParam(current_stepsize, strategies);

            //Forward to update op
            const OperatingPoint last_operating_point(current_operating_point);

            current_operating_point = forward(last_operating_point, strategies);

            //Update linear dyanmics based on current operating point
            computeLinearization(current_operating_point, linear_dyn);

            //Update cost_quad based on current operating point
                       
            covariances = computeCovariance(current_operating_point, linear_dyn);

            
            computeCostQuadraticization(current_operating_point, covariances, cost_quad);

            //update current merit value
            const double current_merit_function_value = computeMeritFunction(current_operating_point, cost_quad);
            
            //update has_converged and last_merit_value
            has_converged = checkConverged(current_merit_function_value);
            last_merit_function_value_ = current_merit_function_value;

            
            return true;

        }
        
        // 1. Precompute expected decrease before we do anything else.
        double expected_decrease = computeExpectedDecrease(strategies, delta_xs, costates, linear_dyn, cost_quad);
        
        // 2. Scale ff term by Line Search Parameters
        scaleByLineSearchParam(solver_params_.initial_alpha_scaling, strategies);

        // 3. Forward
        const OperatingPoint last_operating_point(current_operating_point);
        double current_stepsize = solver_params_.initial_alpha_scaling;
        //double current_stepsize = 0.1;
        //std::cout<<"current_stepsize"<<current_stepsize;
        current_operating_point = forward(last_operating_point, strategies);

        // Keep reducing alphas until we satisfy the Armijo condition.
        for (size_t ii = 0; ii < solver_params_.max_backtracking_steps; ii++)
        {
            // 4. Compute Covariance regarding current operating point
            covariances = computeCovariance(current_operating_point, linear_dyn);

            // 5. Quadraticization
            computeCostQuadraticization(current_operating_point, covariances, cost_quad);

            // 6. Compute merit function value.
            const double current_merit_function_value = computeMeritFunction(current_operating_point, cost_quad);

            // 7. Check Armijo condition.
            if (checkArmijoCondition(current_merit_function_value, current_stepsize, expected_decrease))
            {
                // Success! Update cached terms and check convergence.
                has_converged = checkConverged(current_merit_function_value);
                last_merit_function_value_ = current_merit_function_value;

                // Update linearized dynamics
                computeLinearization(current_operating_point, linear_dyn);

                return true;
            }

            // 8. Update current_step_size
            current_stepsize *= solver_params_.geometric_alpha_scaling;

            // 9. Scale down the alphas and update operating point and linear dynamics. Try again
            scaleByLineSearchParam(solver_params_.geometric_alpha_scaling, strategies);
            current_operating_point = forward(last_operating_point, strategies);
            computeLinearization(current_operating_point, linear_dyn);
        }

        // Output a warning. Solver should revert to last valid operating point.
        std::cerr << "Exceeded maximum number of backtracking steps." << std::endl;
        return false;
    }


}
