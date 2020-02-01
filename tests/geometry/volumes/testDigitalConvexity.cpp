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
 * @file testDigitalConvexity.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/02/01
 *
 * Functions for testing class CellGeometry.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/geometry/volumes/CellGeometry.h"
#include "DGtal/geometry/volumes/DigitalConvexity.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DigitalConvexity.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "DigitalConvexity< Z2 > unit tests", "[digital_convexity][2d]" )
{
  typedef KhalimskySpaceND<2,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Vector                   Vector;
  typedef KSpace::Integer                  Integer;
  typedef DigitalConvexity< KSpace >       DConvexity;

  DConvexity dconv( Point( -5, -5 ), Point( 10, 10 ) );

  GIVEN( "Given a fat simplex { (0,0), (4,-1), (2,5) } " ) {
    std::vector<Point> V = { Point(0,0), Point(4,-1), Point(2,5) };
    auto vertex_cover  = dconv.makeCellCover( V.begin(), V.end() );
    auto fat_simplex   = dconv.makeSimplex  ( V.begin(), V.end() );
    auto inside_pts    = dconv.insidePoints ( fat_simplex );
    auto simplex_cover = dconv.makeCellCover( fat_simplex );
    auto point_cover   = dconv.makeCellCover( inside_pts.begin(), inside_pts.end() );
    THEN( "The fat simplex is not degenerated." ) {
      REQUIRE( dconv.isSimplex( V.begin(), V.end() ) );
    }
    THEN( "Its vertex cover contains 3 0-cells, 12 1-cells, 12 2-cells" ) {
      REQUIRE( vertex_cover.cubicalComplex().nbCells( 0 ) == 3 );
      REQUIRE( vertex_cover.cubicalComplex().nbCells( 1 ) == 12 );
      REQUIRE( vertex_cover.cubicalComplex().nbCells( 2 ) == 12 );
    }
    THEN( "Its vertex cover is a subset of its point cover" ) {
      REQUIRE( vertex_cover.subset( point_cover ) );
    }
    THEN( "Its point cover is a subset of its simplex cover" ) {
      REQUIRE( point_cover.subset( simplex_cover ) );
    }
    THEN( "Being fat, its simplex cover is equal to its point cover" ) {
      REQUIRE( simplex_cover.subset( point_cover ) );
    }
  }
  GIVEN( "Given a thin simplex { (0,0), (4,3), (7,5) } " ) {
    std::vector<Point> V = { Point(0,0), Point(4,3), Point(7,5) };
    auto vertex_cover  = dconv.makeCellCover( V.begin(), V.end() );
    auto thin_simplex  = dconv.makeSimplex  ( V.begin(), V.end() );
    auto inside_pts    = dconv.insidePoints ( thin_simplex );
    auto simplex_cover = dconv.makeCellCover( thin_simplex );
    auto point_cover   = dconv.makeCellCover( inside_pts.begin(), inside_pts.end() );
    THEN( "The thin simplex is not degenerated." ) {
      REQUIRE( dconv.isSimplex( V.begin(), V.end() ) );
    }
    THEN( "Its vertex cover contains 3 0-cells, 12 1-cells, 12 2-cells" ) {
      REQUIRE( vertex_cover.cubicalComplex().nbCells( 0 ) == 3 );
      REQUIRE( vertex_cover.cubicalComplex().nbCells( 1 ) == 12 );
      REQUIRE( vertex_cover.cubicalComplex().nbCells( 2 ) == 12 );
    }
    THEN( "Its vertex cover is a subset of its point cover" ) {
      REQUIRE( vertex_cover.subset( point_cover ) );
    }
    THEN( "Its point cover is a subset of its simplex cover" ) {
      REQUIRE( point_cover.subset( simplex_cover ) );
    }
    THEN( "Being thin, its simplex cover is not equal to its point cover for 1<=dim<=2" ) {
      REQUIRE( ! simplex_cover.subset( point_cover ) );
      REQUIRE( simplex_cover.subset( point_cover, 0 ) );
      REQUIRE( ! simplex_cover.subset( point_cover, 1 ) );
      REQUIRE( ! simplex_cover.subset( point_cover, 2 ) );
    }
  }
}
