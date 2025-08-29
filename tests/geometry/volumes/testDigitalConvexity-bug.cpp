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
 * Functions for testing class DigitalConvexity.
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
#include "DGtal/geometry/tools/AffineGeometry.h"
#include "DGtal/geometry/tools/AffineBasis.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DigitalConvexity.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "DigitalConvexity< Z3 > envelope bug", "[envelope][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef DigitalConvexity< KSpace >       DConvexity;
  typedef AffineBasis< Point >             Basis;

  DConvexity dconv( Point( -36, -36, -36 ), Point( 36, 36, 36 ) );

  WHEN( "Using basis B = (1, 0, -2) (1, 0, -1)" ) {
    std::vector< Point > b = { Point( 0, 0, 0), Point(1, 0, -2), Point(1, 0, -1) };
    const auto [ o, B ] = functions::computeAffineBasis( b );
    Point e  = functions::computeIndependentVector( B );
    Basis AB( Point( 0,0 ), b );
    bool parallel = AB.isParallel( e );
    const auto [ d, L, r ] = AB.decomposeVector( e );
    CAPTURE( B ); 
    CAPTURE( AB.basis() ); 
    CAPTURE( d ); 
    CAPTURE( L );
    CAPTURE( r );
    CAPTURE( e );
    REQUIRE( ! parallel );
    REQUIRE( r != Point::zero );
  }
  
  WHEN( "Computing the envelope Z of a digital set X with direct algorithm" ) {
    std::vector< Point > X = { Point(5, 1, 9), Point(8, 1, 8), Point(9, 1, 1) };
    auto Z = dconv.envelope( X );
    THEN( "Z is fully convex" ){
      CAPTURE( X );
      CAPTURE( dconv.depthLastEnvelope() );
      REQUIRE( dconv.isFullyConvex( Z ) );
    }
  }
}
