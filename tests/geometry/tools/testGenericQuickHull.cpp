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
 * @file testGenericQuickHull.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2025/09/04
 *
 * Functions for testing class GenericQuickHull.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/geometry/tools/GenericQuickHull.h"
#include "DGtal/geometry/tools/AffineGeometry.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

std::random_device rd;
std::mt19937 g(rd());

template <typename Point>
std::vector< Point >
randomPointsInBall( int nb, int R )
{
  std::vector< Point > V;
  Point c = Point::diagonal( R );
  double R2 = (double) R * (double) R;
  for ( int i = 0; i < nb; ) {
    Point p;
    for ( DGtal::Dimension k = 0; k < Point::dimension; ++k )
      p[ k ] = rand() % (2*R);
    if ( ( p - c ).squaredNorm() < R2 ) { V.push_back( p ); i++; }
  }
  return V;
}

template < typename Point >
std::vector< Point >
makeRandomLatticePointsFromDirVectors( int nb, const vector< Point>& V )
{
  std::uniform_int_distribution<int> U(-10, 10);
  vector< Point > P;
  int n = V[0].size();
  int m = V.size();
  Point A;
  for ( auto i = 0; i < n; i++ )
    A[ i ] = U( g );
  P.push_back( A );
  for ( auto k = 0; k < nb; k++ )
    {
      Point B = A;
      for ( auto i = 0; i < m; i++ )
        {
          int l = U( g );
          B += l * V[ i ];
        }
      P.push_back( B );
    }
  std::shuffle( P.begin(), P.end(), g );
  return P;
}

const int debug_level = 0;

// ///////////////////////////////////////////////////////////////////////////////
// // Functions for testing class GenericQuickHull in 2D.
// ///////////////////////////////////////////////////////////////////////////////

SCENARIO( "GenericQuickHull< ConvexHullIntegralKernel< 2 > > unit tests", "[genquickhull][integral_kernel][2d]" )
{
  typedef ConvexHullIntegralKernel< 2 >    QHKernel;
  typedef GenericQuickHull< QHKernel >     QHull;
  typedef SpaceND< 2, int >                Space;      
  typedef Space::Point                     Point;
  typedef QHull::Index                     Index;
  typedef QHull::IndexRange                IndexRange;

  
  GIVEN( "Given a set { } " ) {
    std::vector<Point> V = { };
    QHull hull( QHKernel(), debug_level );
    bool ok = hull.compute( V, false );
    std::cout << hull << std::endl;
    THEN( "Everything went fine and dim=-1." ) {
      REQUIRE( ok );
      REQUIRE( hull.affine_dimension == -1 );
    }
  }
  GIVEN( "Given a set { (2,1), (2,1) } " ) {
    std::vector<Point> V
      = { Point(2,1), Point(2,1) };
    QHull hull( QHKernel(), debug_level );
    bool ok = hull.compute( V, false );
    std::cout << hull << std::endl;
    THEN( "Everything went fine and dim=0." ) {
      REQUIRE( ok );
      REQUIRE( hull.affine_dimension == 0 );
    }
  }
  GIVEN( "Given a set { (0,0), (-4,-1), (8, 2), (16,4) } " ) {
    std::vector<Point> V
      = { Point(0,0), Point(-4,-1), Point(8,2), Point(16,4) };
    QHull hull( QHKernel(), debug_level );
    bool ok = hull.compute( V, false );
    std::cout << hull << std::endl;
    THEN( "Everything went fine and dim=1." ) {
      REQUIRE( ok );
      REQUIRE( hull.affine_dimension == 1 );
    }
  }
  GIVEN( "Given a set { (0,0), (-4,-1), (-3,5), (7,3), (5, -2), (2, 2) } " ) {
    std::vector<Point> V
      = { Point(0,0), Point(-4,-1), Point(-3,5), Point(7,3), Point(5, -2), Point(2,2) };
    QHull hull( QHKernel(), debug_level );
    bool ok = hull.compute( V, false );
    std::cout << hull << std::endl;
    THEN( "Everything went fine and dim=2." ) {
      REQUIRE( ok );
      REQUIRE( hull.affine_dimension == 2 );
    }
  }

}

SCENARIO( "GenericQuickHull< ConvexHullIntegralKernel< 3 > > unit tests", "[genquickhull][integral_kernel][3d]" )
{
  typedef ConvexHullIntegralKernel< 3 >    QHKernel;
  typedef GenericQuickHull< QHKernel >     QHull;
  typedef SpaceND< 3, int >                Space;      
  typedef Space::Point                     Point;
  typedef QHull::Index                     Index;
  typedef QHull::IndexRange                IndexRange;

  std::vector< Point > V1 = { Point{ 5,-2, 1 } };
  std::vector< Point > X1 = makeRandomLatticePointsFromDirVectors( 20, V1 );
  std::vector< Point > V2 = { Point{ 5,-2, 1 }, Point{ -3, 4, 2 } };
  std::vector< Point > X2 = makeRandomLatticePointsFromDirVectors( 20, V2 );
  std::vector< Point > V3 = { Point{ 5,-2, 1 }, Point{ -3, 4, 2 }, Point{ 1, -7, 11 } };
  std::vector< Point > X3 = makeRandomLatticePointsFromDirVectors( 20, V3 );
  
  GIVEN( "Given a 1-d lattice set in Z3 " ) {
    QHull hull( QHKernel(), debug_level );
    bool ok = hull.compute( X1, false );
    std::cout << hull << std::endl;
    THEN( "Everything went fine and dim=1." ) {
      REQUIRE( ok );
      REQUIRE( hull.affine_dimension == 1 );
    }
  }
  GIVEN( "Given a 2-d lattice set in Z3 " ) {
    QHull hull( QHKernel(), debug_level );
    bool ok = hull.compute( X2, false );
    std::cout << hull << std::endl;
    THEN( "Everything went fine and dim=2." ) {
      REQUIRE( ok );
      REQUIRE( hull.affine_dimension == 2 );
    }
  }
  GIVEN( "Given a 3-d lattice set in Z3 " ) {
    QHull hull( QHKernel(), debug_level );
    bool ok = hull.compute( X3, false );
    std::cout << hull << std::endl;
    THEN( "Everything went fine and dim=3." ) {
      REQUIRE( ok );
      REQUIRE( hull.affine_dimension == 3 );
    }
  }
}
