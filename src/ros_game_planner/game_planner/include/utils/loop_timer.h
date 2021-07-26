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
// Keeps track of elapsed time (e.g., during loops) and provides an upper bound
// on the runtime of the next loop. To reduce memory consumption and adapt to
// changing processor activity, computes statistics based on a moving window of
// specified length.
//
///////////////////////////////////////////////////////////////////////////////
#ifndef GAME_PLANNER_LOOP_TIMER_H
#define GAME_PLANNER_LOOP_TIMER_H

#include <chrono>
#include <list>
#include "utils/types.h"

namespace game_planner
{
    class LoopTimer
    {
    public:
        ~LoopTimer() = default;

        LoopTimer(size_t max_samples = 10)
                : max_samples_(max_samples), total_time_(0.0)
        {
            assert(max_samples>=1);

            // For defined behavior, starting with a Tic().
            Tic();
        }

        // Tic and toc. Start and stop loop timer.
        void Tic();
        double Toc();

    private:
        // Maximum number of samples used to compute mean and variance.
        const size_t max_samples_;

        // Most recent timer start time.
        std::chrono::time_point<std::chrono::high_resolution_clock> start_;

        // Queue of observed loop times.
        std::list<double> loop_times_;

        // Running sum of times in the queue.
        double total_time_;
    };
}


#endif //GAME_PLANNER_LOOP_TIMER_H
