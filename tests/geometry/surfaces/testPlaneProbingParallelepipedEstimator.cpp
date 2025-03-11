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
 * Functions for testing class DGtal::PlaneProbingParallelepipedEstimator.
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
// Functions for testing class PlaneProbingParallelepipedEstimator.
///////////////////////////////////////////////////////////////////////////////

static const Z3i::Vector NORMALS[100] = {
    {1, 117, 148}, {1, 118, 149}, {1, 120, 25}, {1, 120, 152}, {1, 121, 153}, {1, 122, 154}, {1, 123, 155}, {1, 123, 156}, {1, 124, 26}, {1, 124, 157},
    {1, 125, 26}, {1, 125, 158}, {1, 126, 77}, {1, 126, 159}, {1, 127, 160}, {1, 127, 161}, {1, 128, 27}, {1, 128, 162}, {1, 129, 27}, {1, 129, 163},
    {1, 130, 164}, {1, 130, 165}, {1, 131, 165}, {1, 131, 166}, {1, 132, 28}, {1, 132, 166}, {1, 132, 167}, {1, 133, 28}, {1, 133, 168}, {1, 134, 169},
    {1, 134, 170}, {1, 135, 170}, {1, 135, 171}, {1, 136, 171}, {1, 136, 172}, {1, 137, 29}, {1, 137, 173}, {1, 137, 174}, {1, 138, 174}, {1, 138, 175},
    {1, 139, 175}, {1, 139, 176}, {1, 140, 176}, {1, 140, 177}, {1, 140, 178}, {1, 141, 30}, {1, 141, 178}, {1, 141, 179}, {1, 142, 179}, {1, 142, 180},
    {1, 143, 180}, {1, 143, 181}, {1, 144, 182}, {1, 144, 183}, {1, 145, 183}, {1, 145, 184}, {1, 146, 184}, {1, 146, 185}, {1, 147, 186}, {1, 147, 187},
    {1, 148, 187}, {1, 148, 188}, {1, 149, 188}, {1, 149, 189}, {1, 150, 190}, {1, 151, 191}, {1, 151, 192}, {1, 152, 192}, {1, 152, 193}, {1, 153, 194},
    {1, 154, 195}, {1, 154, 196}, {1, 155, 196}, {1, 155, 197}, {1, 156, 198}, {1, 157, 199}, {1, 173, 30}, {1, 174, 30}, {1, 175, 30}, {1, 178, 31},
    {1, 179, 31}, {1, 180, 31}, {1, 181, 31}, {1, 184, 32}, {1, 185, 32}, {1, 186, 32}, {1, 187, 32}, {1, 188, 32}, {1, 189, 33}, {1, 190, 33},
    {1, 191, 33}, {1, 192, 33}, {1, 193, 33}, {1, 194, 33}, {1, 194, 34}, {1, 195, 34}, {1, 196, 34}, {1, 197, 34}, {1, 198, 34}, {1, 199, 34},
};

static const Z3i::Vector NORMALS_BIG[2] = {
    {1, 59438, 82499}, {2071, 8513, 6444},
};

template < typename Integer, ProbingMode mode >
struct TestPlaneProbingParallelepipedEstimator
{
    using Space        = SpaceND<3, Integer>;
    using DigitalPlane = DigitalPlanePredicate<Space>;
    using Point        = typename DigitalPlane::Vector;
    using Estimator    = PlaneProbingParallelepipedEstimator<DigitalPlane, mode>;

    template < typename F >
    static void compute (Point const& n, int height, F const& f)
    {
        Point o(0, 0, 0);
        std::array<Point, 3> frame = { Point(1, 0, 0), Point(0, 1, 0), Point(0, 0, 1) };

        DigitalPlane plane(n, -height, n.norm1());
        Estimator estimator(o, frame, plane, 2 * n.norm1());

        f(estimator);
    }
};

TEST_CASE( "Testing PlaneProbingParallelepipedEstimator" )
{
    static const int MAX_HEIGHT = 10;

    SECTION("H-algorithm should return the correct normal")
    {
        int nbNormals = 0;
        int nbOk = 0;

        for (const auto& n: NORMALS) {
            for (int height = 0; height < min(int(n.normInfinity()), MAX_HEIGHT); ++height) {
                ++nbNormals;

                TestPlaneProbingParallelepipedEstimator<int, ProbingMode::H>::compute
                    (n, height,
                     [&] (TestPlaneProbingParallelepipedEstimator<int, ProbingMode::H>::Estimator& estimator) {
                        auto estimated = estimator.compute();
                        bool isReducedH = estimator.isReduced();

                        if (estimated == n && !isReducedH)
                        {
                            nbOk++;
                        }
                     });
            }
        }

        REQUIRE(nbNormals == nbOk);
    }

    SECTION("R1-algorithm should return the correct normal and a reduced basis")
    {
        int nbNormals = 0;
        int nbOk = 0;

        for (const auto& n: NORMALS) {
            for (int height = 0; height < min(int(n.normInfinity()), MAX_HEIGHT); ++height) {
                ++nbNormals;

                TestPlaneProbingParallelepipedEstimator<int, ProbingMode::R1>::compute
                    (n, height,
                     [&] (TestPlaneProbingParallelepipedEstimator<int, ProbingMode::R1>::Estimator& estimator) {
                        auto estimated = estimator.compute();
                        bool isReducedR = estimator.isReduced();

                        if (estimated == n && isReducedR)
                        {
                            nbOk++;
                        }
                     });
            }
        }

        REQUIRE(nbNormals == nbOk);
    }

    SECTION("L-algorithm should return the correct normal and a reduced basis")
    {
        int nbNormals = 0;
        int nbOk = 0;

        for (const auto& n: NORMALS) {
            for (int height = 0; height < min(int(n.normInfinity()), MAX_HEIGHT); ++height) {
                ++nbNormals;

                TestPlaneProbingParallelepipedEstimator<int, ProbingMode::L>::compute
                    (n, height,
                     [&] (TestPlaneProbingParallelepipedEstimator<int, ProbingMode::L>::Estimator& estimator) {
                        auto estimated = estimator.compute();
                        bool isReduced = estimator.isReduced();

                        if (estimated == n && isReduced)
                        {
                            nbOk++;
                        }
                     });
            }
        }

        REQUIRE(nbNormals == nbOk);
    }    

#ifdef DGTAL_WITH_GMP
    SECTION("H-algorithm should return the correct normal with BigInteger")
    {
        int nbNormals = 0;
        int nbOk = 0;

        for (const auto& n: NORMALS_BIG) {
            for (int height = 0; height < MAX_HEIGHT; ++height) {
                ++nbNormals;

                TestPlaneProbingParallelepipedEstimator<BigInteger, ProbingMode::H>::compute
                    (n, height,
                     [&] (TestPlaneProbingParallelepipedEstimator<BigInteger, ProbingMode::H>::Estimator& estimator) {
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
