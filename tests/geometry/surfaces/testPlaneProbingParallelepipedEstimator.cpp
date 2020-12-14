/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

/**
 * @file
 * @ingroup Tests
 * @author Jocelyn Meyron (\c jocelyn.meyron@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2020/12/09
 *
 * Functions for testing class PlaneProbingTetrahedronEstimator.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/arithmetic/IntegerComputer.h"
#include "DGtal/geometry/surfaces/DigitalPlanePredicate.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingParallelepipedEstimator.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class PlaneProbingTetrahedronEstimator.
///////////////////////////////////////////////////////////////////////////////

template < typename Integer >
std::vector<typename SpaceND<3, Integer>::Vector>
generateNormals(Integer const& N)
{
    using Space = SpaceND<3, Integer>;
    using Vector = typename Space::Point;

    std::vector<Vector> normals;

    IntegerComputer<Integer> intComp;

    for (Integer i = 1; i <= N; ++i)
    {
        for (Integer j = 1; j <= N; ++j)
        {
            for (Integer k = 1; k <= N; ++k)
            {
                if (intComp.gcd(intComp.gcd(i, j), k) == 1)
                {
                    normals.emplace_back(i, j, k);
                }
            }
        }
    }

    return normals;
}

template < typename Integer, ProbingMode mode, typename F >
void testPlaneProbingParallelepipedEstimator (typename SpaceND<3, Integer>::Vector const& n, int height,
                                              F const& f)
{
    using Space = SpaceND<3, Integer>;
    using DigitalPlane = DigitalPlanePredicate<Space>;
    using Point = typename DigitalPlane::Vector;
    using Estimator = PlaneProbingParallelepipedEstimator<DigitalPlane, mode>;

    Point o(0, 0, 0);
    std::array<Point, 3> frame = { Point(1, 0, 0), Point(0, 1, 0), Point(0, 0, 1) };

    DigitalPlane plane(n, -height, n.norm1());
    Estimator estimator(o, frame, plane, n.norm1() + 1);

    f(estimator);
}

TEST_CASE( "Testing PlaneProbingParallelepipedEstimator" )
{
    const int maxComponent = 20;
    const auto normals = generateNormals(maxComponent);

    SECTION("H-algorithm should return the correct normal")
    {
        int nbNormals = 0;
        int nbOk = 0;

        for (const auto& n: normals) {
            for (int height = 0; height < int(n.normInfinity()); ++height) {
                ++nbNormals;

                testPlaneProbingParallelepipedEstimator<int, ProbingMode::H>
                    (n, height,
                     [&] (auto& estimator) {
                        auto estimated = estimator.compute();

                        if (estimated == n)
                        {
                            nbOk++;
                        }
                     });
            }
        }

        REQUIRE(nbNormals == nbOk);
    }
#ifdef WITH_GMP
    SECTION("H-algorithm should return the correct normal with BigInteger")
    {
        int nbNormals = 0;
        int nbOk = 0;

        for (const auto& n: normals) {
            for (int height = 0; height < int(n.normInfinity()); ++height) {
                ++nbNormals;

                testPlaneProbingParallelepipedEstimator<BigInteger, ProbingMode::H>
                    (n, height,
                     [&] (auto& estimator) {
                        auto estimated = estimator.compute();

                        if (estimated == n)
                        {
                            nbOk++;
                        }
                     });
            }
        }

        REQUIRE(nbNormals == nbOk);
    }
#endif
}

/** @ingroup Tests **/
