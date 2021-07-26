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
// Base class for all explicit (scalar-valued) constraints. These
// constraints are of the form: g(x) = 0 or g(x) <= 0 for some vector x.
//
// In addition to checking for satisfaction (and returning the constraint value
// g(x)), they also support computing first and second derivatives of the
// constraint value itself and the square of the constraint value, each scaled
// by lambda or mu respectively (from the augmented Lagrangian). That is, they
// compute gradients and Hessians of
//         L(x, lambda, mu) = lambda * g(x) + mu * g(x) * g(x) / 2
//
///////////////////////////////////////////////////////////////////////////////
#ifndef GAME_PLANNER_CONSTRAINT_H
#define GAME_PLANNER_CONSTRAINT_H

#include "utils/types.h"
#include "constraint/base_al_constraint.h"

namespace game_planner
{
    class ALConstraint : public BaseALConstraint
    {
    public:
        virtual ~ALConstraint() = default;

        virtual double evaluate(const int& time_id, const Eigen::VectorXd& input) const = 0;
        virtual double evaluate(const Eigen::VectorXd& input) const = 0;

        // Quadraticize the constraint value and its square, each scaled by lambda or
        // mu, respectively (terms in the augmented Lagrangian).
        virtual void quadraticize(const int& time_id,
                                  const Eigen::VectorXd& input,
                                  Eigen::MatrixXd& hess,
                                  Eigen::VectorXd& grad) const = 0;

        // Check if this constraint is satisfied, and optionally return the constraint
        // value, which equals zero if the constraint is satisfied.
        bool isSatisfied(const int& time_id,
                         const Eigen::VectorXd& input,
                         double* level) const
        {
            const double value = evaluate(time_id, input); //Evaluate this constraint
            if (level) *level = value;

            return BaseALConstraint::isSatisfied(value);
        }

        // Evaluate this constraint value, i.e., g(x), and the augmented Lagrangian,
        // i.e., lambda g(x) + mu g(x) g(x) / 2.
        double evaluateAugmentedLagrangian(const int& time_id, const Eigen::VectorXd& input) const
        {
            const double g = evaluate(time_id, input);
            const double lambda = lambdas_[time_id];
            return lambda * g + 0.5 * getMu(lambda, g) * g * g;
        }

        // Accessors and setters.
        bool getIsEquality() const { return is_equality_; }
        double& getLambda(const int& time_id) { return lambdas_[time_id]; }
        double getLambda(const int& time_id) const { return lambdas_[time_id]; }

        void incrementLambda(const int& time_id, double value)
        {
            const double new_lambda = lambdas_[time_id] + mu_ * value;
            lambdas_[time_id] = (is_equality_) ? new_lambda : std::max(0.0, new_lambda);
        }

        void scaleLambdas(double scale)
        {
            for (auto& lambda : lambdas_) lambda *= scale;
        }

        double getMu(const int& time_id, const Eigen::VectorXd& input) const
        {
            const double g = evaluate(time_id, input);
            return getMu(getLambda(time_id), g);
        }

        double getMu(double lambda, double g) const
        {
            if (!is_equality_ && g <= constants::kSmallNumber &&
                std::abs(lambda) <= constants::kSmallNumber)
                return 0.0;
            return mu_;
        }

    protected:
        explicit ALConstraint(const bool is_equality, const std::string& name)
                : BaseALConstraint(is_equality, name)
        {}

        // Modify derivatives to account for the multipliers and the quadratic term in
        // the augmented Lagrangian. The inputs are the derivatives of g in the
        // appropriate variables (assumed to be arbitrary coordinates of the input,
        // here called x and y).
        //Update Constraint by Augmented Lagrangian Method
        void adaptALConstraint(const int &time_id,
                               const double &g,
                               double *dx,
                               double *ddx,
                               double *dy = nullptr,
                               double *ddy = nullptr,
                               double *dxdy = nullptr) const
        {
            assert(time_id < lambdas_.size());
            // Unpack lambda.
            const double lambda = lambdas_[time_id];
            const double mu = getMu(lambda, g);

            // Assumes that these are just the derivatives of g(x, y), and modifies them
            // to be derivatives of lambda g(x) + mu g(x) g(x) / 2.
            const double new_dx = lambda * *dx + mu * g * *dx;
            const double new_ddx = lambda * *ddx + mu * (*dx * *dx + g * *ddx);

            if (dy) {
                const double new_dy = lambda * *dy + mu * g * *dy;
                const double new_ddy = lambda * *ddy + mu * (*dy * *dy + g * *ddy);
                const double new_dxdy = lambda * *dxdy + mu * (*dy * *dx + g * *dxdy);

                *dy = new_dy;
                *ddy = new_ddy;
                *dxdy = new_dxdy;
            }

            *dx = new_dx;
            *ddx = new_ddx;
        }
    };
}

#endif //GAME_PLANNER_CONSTRAINT_H
