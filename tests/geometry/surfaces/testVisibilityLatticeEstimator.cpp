/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

/**
 * @file testVisibilityLatticeEstimator.cpp
 * @ingroup Tests
 * @author Romain Negro
 * @date 2026/11/20
 *
 * Unit tests for VisibilityLatticeEstimator.
 *
 * This file is part of the DGtal library.
 */

#include <cmath>
#include <vector>

#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/base/Common.h"
#include "DGtal/geometry/surfaces/estimation/VisibilityLatticeEstimator.h"
#include "DGtal/helpers/ShortcutsGeometry.h"
#include "DGtal/helpers/StdDefs.h"

using namespace DGtal;
using namespace Z3i;

typedef VisibilityLatticeEstimator< Space > VisibilityEstimator;
typedef ShortcutsGeometry< KSpace > GeometryShortcuts;

namespace
{
  std::vector< Point > planarPointels()
  {
    std::vector< Point > pointels;
    for ( Integer x = 0; x <= 4; ++x )
      for ( Integer y = 0; y <= 4; ++y )
        pointels.emplace_back( x, y, 0 );
    return pointels;
  }
}

TEST_CASE( "Visibility lattice detects visible segments in a planar shape" )
{
  const auto pointels = planarPointels();
  auto params = GeometryShortcuts::parametersVisibilityComputation();
  params( "verbose", 0 )( "radius", 1 )( "nStar", 1 )( "gridstep", 1.0 );
  const auto visibility =
    GeometryShortcuts::getVisibilityNoOpenMP( pointels, params );

  REQUIRE_FALSE( visibility.empty() );
  REQUIRE( visibility.isVisible( pointels[ 0 ], pointels[ 5 ] ) );
  REQUIRE( visibility.isVisible( pointels[ 5 ], pointels[ 0 ] ) );
}

TEST_CASE( "Visibility normal shortcuts forward their parameters" )
{
  const auto pointels = planarPointels();
  const auto allVisible = [] ( const Point&, const Point& ) { return true; };
  auto params = GeometryShortcuts::parametersVisibilityNormals();
  params( "verbose", 0 )( "sigma", 1.0 )( "neighborhood-radius", 2.0 )
        ( "auto-limiter", 0 )( "visibility-kernel", "none" );

  const auto normals = GeometryShortcuts::getVisibilityNormalVectorsNoOpenMP
    ( pointels, allVisible, params );

  REQUIRE( normals.size() == pointels.size() );
  for ( const auto& normal : normals )
    REQUIRE( std::abs( normal[ 2 ] ) == Approx( 1.0 ) );
}

TEST_CASE( "Visibility normals recover the normal direction of a planar patch" )
{
  const auto pointels = planarPointels();
  const auto allVisible = [] ( const Point&, const Point& ) { return true; };
  const auto normals = VisibilityEstimator::getVisibilityNormalVectorsNoOpenMP
    ( pointels, allVisible, 1.0, 2.0, false, 1.0,
      VisibilityEstimator::VisibilityCenter::Centroid,
      VisibilityEstimator::VisibilityKernel::None, 0 );

  REQUIRE( normals.size() == pointels.size() );
  for ( const auto& normal : normals )
  {
    REQUIRE( normal.norm() == Approx( 1.0 ) );
    REQUIRE( std::abs( normal[ 2 ] ) == Approx( 1.0 ) );
  }
}
