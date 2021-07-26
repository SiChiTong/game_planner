///////////////////////////////////////////////////////////////////////////////
//
// Container to store a linear approximation of the dynamics at a particular
// time.
//
///////////////////////////////////////////////////////////////////////////////
//checked: 2
#ifndef GAME_PLANNER_LINEAR_DYNAMICS_H
#define GAME_PLANNER_LINEAR_DYNAMICS_H

#include "utils/types.h"

namespace game_planner
{
    struct LinearDynamics
    {
        LinearDynamics() = default;
        LinearDynamics(const unsigned int& num_players,
                       const int& xdim,
                       const std::vector<int>& udims)
                : A(Eigen::MatrixXd::Identity(xdim, xdim)),
                  Bs(num_players)
        {
            for (size_t ii = 0; ii < num_players; ii++)
                Bs[ii] = Eigen::MatrixXd::Zero(xdim, udims[ii]);
        }

        Eigen::MatrixXd A;               // State Matrix
        std::vector<Eigen::MatrixXd> Bs; // Control Matrix
    };
}

#endif //GAME_PLANNER_LINEAR_DYNAMICS_H
