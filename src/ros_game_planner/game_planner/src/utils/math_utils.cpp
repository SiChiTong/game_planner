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
// Container For Math Utils
//
///////////////////////////////////////////////////////////////////////////////
#include "utils/math_utils.h"

namespace game_planner
{
    namespace math
    {
        std::tuple<Eigen::Vector2d, double, double> computeConvexFeasibleSet(const Eigen::VectorXd& input,
                                                                             const RectangularObstacle& obs,
                                                                             const int& xidx,
                                                                             const int& yidx)
        {
            // Compute Current World Poly vertexes
            std::array<std::pair<double,double>,4> world_poly;
            for(int poly_id=0; poly_id<4; ++poly_id)
            {
                world_poly[poly_id].first  = obs.getX() + obs.getVertexes(poly_id).first;
                world_poly[poly_id].second = obs.getY() + obs.getVertexes(poly_id).second;
            }

            //Current Ego Position
            Eigen::Vector2d p0;
            p0 << input(xidx), input(yidx);

            //calculate distance from p0 to world_poly
            return computeDistanceToPolygon(p0, world_poly);
        }

        std::tuple<Eigen::Vector2d, double, double> computeDistanceToPolygon(const Eigen::Vector2d& p0,
                                                                             const std::array<std::pair<double, double>,4>& poly)
        {
            int nside = poly.size();
            Eigen::Vector2d L = Eigen::Vector2d::Zero();
            double d = std::numeric_limits<double>::max();
            double S = 0.0;
            int min_id = -1;

            // search for every edge of polygon
            for (int i = 0; i < nside; i++)
            {
                int j = (i + 1) % nside;
                Eigen::Vector2d p1;
                Eigen::Vector2d p2;
                p1 << poly[i].first, poly[i].second;	// first vertex
                p2 << poly[j].first, poly[j].second;	// second vertex
                double x1 = p1(0);
                double y1 = p1(1);
                double x2 = p2(0);
                double y2 = p2(1);
                double px = p0(0);
                double py = p0(1);


                double trid[3];
                trid[0] = (p1 - p2).norm(); // norm([x1 - x2, y1 - y2]);
                trid[1] = (p1 - p0).norm(); // norm([x1 - point(1), y1 - point(2)]);
                trid[2] = (p2 - p0).norm(); // norm([x2 - point(1), y2 - point(2)]);

                Eigen::Vector2d Lr;
                double Sr;
                double vd;

                if (trid[1] * trid[1] > trid[0] * trid[0] + trid[2] * trid[2])
                {
                    vd = trid[2];
                    Lr = p0 - p2; // [point(1) - x2, point(2) - y2];
                    Sr = (p0 - p2).dot(p2);  // [point(1) - x2, point(2) - y2] * [x2, y2]';
                }
                else if (trid[2] * trid[2] > trid[0] * trid[0] + trid[1] * trid[1])
                {
                    vd = trid[1];
                    Lr = p0 - p1;	//  [point(1) - x1, point(2) - y1];
                    Sr = (p0 - p1).dot(p1);  // [point(1) - x1, point(2) - y1] * [x1, y1]';
                }
                else
                {
                    Lr << (y1 - y2), (x2 - x1);
                    Sr = -x1 * y2 + x2 * y1;
                    vd = abs(Lr(0) * px + Lr(1) * py - Sr) / trid[0];
                }

                if (vd < d)
                {
                    d = vd;
                    L = Lr;
                    S = Sr;
                    min_id = i;
                }
            }

            double nL = L.norm();
            L = L / nL;
            S = S / nL;

            int jj = (min_id + 2) % nside;
            Eigen::Vector2d polyJ;
            polyJ << poly[jj].first, poly[jj].second;
            if (L.dot(polyJ) < S)
            {
                L = -L;
                S = -S;
            }

            return std::make_tuple(L, S, d);
        }

        double computeInverseErrorFunction(const double& x)
        {
            const double sgn = (x < 0) ? -1.0 : 1.0;

            const double y = (1 - x)*(1 + x);        //y = 1 - x*x;
            const double lnx = std::log(y);

            const double tt1 = 2/(M_PI*0.147) + 0.5f * lnx;
            const double tt2 = 1/(0.147) * lnx;

            return sgn*std::sqrt(-tt1 + std::sqrt(tt1*tt1 - tt2));
        }
    } // namespace math
} // namespace game planner