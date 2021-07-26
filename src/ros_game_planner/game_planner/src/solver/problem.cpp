#include "solver/problem.h"

namespace game_planner
{
    bool Problem::isConstrained() const
    {
        for(const auto& pc : player_costs_)
            if (pc.isConstrained()) return true;

        return false;
    }
}
