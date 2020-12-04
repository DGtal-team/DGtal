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
 * @date 2020/12/03
 *
 * Functions for testing class testDigitalPlanePredicate.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/surfaces/DigitalPlanePredicate.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DigitalPlanePredicate.
///////////////////////////////////////////////////////////////////////////////

TEST_CASE("Testing DigitalPlanePredicate")
{
    using DigitalPlane = DigitalPlanePredicate<Z3i::Space>;
    using Vector = DigitalPlane::Vector;
    using Point = DigitalPlane::Point;

    Vector n(2, 6, 15);
    DigitalPlane standardPlane(n, 0, n.norm1());

    SECTION("Testing operator() of DigitalPlanePredicate")
    {
        REQUIRE(standardPlane(Point(0, 0, 0)));
        REQUIRE(standardPlane(Point(1, 0, 0)));
        REQUIRE(standardPlane(Point(0, 1, 0)));
        REQUIRE(standardPlane(Point(0, 0, 1)));
        REQUIRE(! standardPlane(Point(1, 1, 1)));
    }
}

/** @ingroup Tests **/
