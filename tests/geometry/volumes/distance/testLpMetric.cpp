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
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2019/01/27
 *
 * Functions for testing class LpMetric.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/volumes/distance/LpMetric.h"
#include "DGtal/geometry/volumes/distance/CMetricSpace.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class LpMetric.
///////////////////////////////////////////////////////////////////////////////
TEST_CASE( "Testing LpMetric" )
{
  
  DGTAL_CONCEPT_CHECK(requires concepts::CMetricSpace<LpMetric<Z2i::Space>> );

  LpMetric<Z2i::Space> l2_2D(2.0);
  LpMetric<Z3i::Space> l2_3D(2.0);
  LpMetric<Z2i::Space> l55_2D(5.5);
  Z2i::Space::RealPoint a(0,0), b(1.0,1.0), c(0.5,0.5);
  Z3i::Space::RealPoint aa(0,0,0), bb(1.0,1.0,1.0);
  
  SECTION("Testing LpMetric distance values")
    {
      CAPTURE( l55_2D) ;
      REQUIRE( l2_2D.rawDistance(a,b) == Approx(2.0) );
      REQUIRE( l2_3D.rawDistance(aa,bb) == Approx(3.0) );
      REQUIRE( l2_2D(a,b) == Approx(std::sqrt(2.0)) );
      REQUIRE( l55_2D(a,b) == Approx(1.1343125) );
      REQUIRE( l2_2D.closest(a,c,b) == DGtal::ClosestFIRST );
      REQUIRE( l2_2D.closest(c,a,b) == DGtal::ClosestBOTH );
    }
}

/** @ingroup Tests **/
