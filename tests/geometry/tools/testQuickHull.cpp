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
 * @file testQuickHull.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/02/01
 *
 * Functions for testing class QuickHull.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/geometry/tools/QuickHull.h"
#include "DGtal/geometry/tools/AffineSubset.h"
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

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class QuickHull in 2D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "QuickHull< ConvexHullIntegralKernel< 2 > > unit tests", "[quickhull][integral_kernel][2d]" )
{
  typedef ConvexHullIntegralKernel< 2 >    QHKernel;
  typedef QuickHull< QHKernel >            QHull;
  typedef SpaceND< 2, int >                Space;      
  typedef Space::Point                     Point;
  typedef QHull::Index                     Index;
  typedef QHull::IndexRange                IndexRange;

  
  GIVEN( "Given a star { (0,0), (-4,-1), (-3,5), (7,3), (5, -2) } " ) {
    std::vector<Point> V
      = { Point(0,0), Point(-4,-1), Point(-3,5), Point(7,3), Point(5, -2) };
    QHull hull;
    hull.setInput( V, false );
    hull.computeConvexHull();
    THEN( "The convex hull is valid and contains every point" ) {
      REQUIRE( hull.check() );
    }
    THEN( "Its convex hull has 4 vertices" ) {
      REQUIRE( hull.nbVertices() == 4 );
    }
    THEN( "Its convex hull has 4 facets" ) {
      REQUIRE( hull.nbFacets() == 4 );
    }
    THEN( "Its facets form a linked list" ) {
      std::vector< IndexRange > facets;
      hull.getFacetVertices( facets );
      std::vector< Index > next( hull.nbVertices(), (Index) -1 );
      Index nb_zero_next = hull.nbVertices();
      Index  nb_two_next = 0;
      for ( auto f : facets ) {
        if ( next[ f[ 0 ] ] != (Index) -1 ) nb_two_next += 1;
        else {
          next[ f[ 0 ] ] = f[ 1 ];
          nb_zero_next  -= 1;
        }
      }
      REQUIRE( nb_zero_next == 0 );
      REQUIRE( nb_two_next == 0 );
    }
  }
  GIVEN( "Given 100 random point in a ball of radius 50 " ) {
    std::vector<Point> V = randomPointsInBall< Point >( 100, 50 );
    QHull hull;
    hull.setInput( V, false );
    hull.computeConvexHull();
    THEN( "The convex hull is valid and contains every point" ) {
      REQUIRE( hull.check() );
    }
    THEN( "Its convex hull has the same number of vertices and facets" ) {
      REQUIRE( hull.nbVertices() == hull.nbFacets() );
    }
    THEN( "Its convex hull has much fewer vertices than input points" ) {
      REQUIRE( 2*hull.nbVertices() <= hull.nbPoints() );
    }
    THEN( "Its facets form a linked list" ) {
      std::vector< IndexRange > facets;
      hull.getFacetVertices( facets );
      std::vector< Index > next( hull.nbVertices(), (Index) -1 );
      Index nb_zero_next = hull.nbVertices();
      Index  nb_two_next = 0;
      for ( auto f : facets ) {
        if ( next[ f[ 0 ] ] != (Index) -1 ) nb_two_next += 1;
        else {
          next[ f[ 0 ] ] = f[ 1 ];
          nb_zero_next  -= 1;
        }
      }
      REQUIRE( nb_zero_next == 0 );
      REQUIRE( nb_two_next == 0 );
    }
  }
}


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class QuickHull in 3D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "QuickHull< ConvexHullIntegralKernel< 3 > > unit tests", "[quickhull][integral_kernel][3d]" )
{
  typedef ConvexHullIntegralKernel< 3 >    QHKernel;
  typedef QuickHull< QHKernel >            QHull;
  typedef SpaceND< 3, int >                Space;      
  typedef Space::Point                     Point;
  typedef QHull::IndexRange                IndexRange;

  
  GIVEN( "Given an octahedron" ) {
    const int R = 5;
    std::vector<Point> V = { Point( 0,0,0 ) };
    for ( Dimension k = 0; k < 3; ++k ) {
      V.push_back( Point::base( k,  R ) );
      V.push_back( Point::base( k, -R ) );
    }
    QHull hull;
    hull.setInput( V, false );
    hull.setInitialSimplex( IndexRange { 0, 1, 3, 5 } );
    hull.computeConvexHull();
    THEN( "The convex hull is valid and contains every point" ) {
      REQUIRE( hull.check() );
    }
    THEN( "Its convex hull has 6 vertices" ) {
      REQUIRE( hull.nbVertices() == 6 );
    }
    THEN( "Its convex hull has 8 facets" ) {
      REQUIRE( hull.nbFacets() == 8 );
    }
  }
  GIVEN( "Given 100 random point in a ball of radius 50 " ) {
    std::vector<Point> V = randomPointsInBall< Point >( 100, 50 );
    QHull hull;
    hull.setInput( V, false );
    hull.computeConvexHull();
    THEN( "The convex hull is valid and contains every point" ) {
      REQUIRE( hull.check() );
    }
    THEN( "Its convex hull has more facets than vertices" ) {
      REQUIRE( hull.nbVertices() < hull.nbFacets() );
    }
    THEN( "Its convex hull has fewer vertices than input points" ) {
      REQUIRE( hull.nbVertices() < hull.nbPoints() );
    }
  }
}


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class QuickHull in 4D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "QuickHull< ConvexHullIntegralKernel< 4 > > unit tests", "[quickhull][integral_kernel][4d]" )
{
  typedef ConvexHullIntegralKernel< 4 >    QHKernel;
  typedef QuickHull< QHKernel >            QHull;
  typedef SpaceND< 4, int >                Space;      
  typedef Space::Point                     Point;

  
  GIVEN( "Given an octahedron" ) {
    const int R = 5;
    std::vector<Point> V = { Point( 0,0,0,0 ) };
    for ( Dimension k = 0; k < 4; ++k ) {
      V.push_back( Point::base( k,  R ) );
      V.push_back( Point::base( k, -R ) );
    }
    QHull hull;
    hull.setInput( V, false );
    hull.computeConvexHull();
    THEN( "The convex hull is valid and contains every point" ) {
      REQUIRE( hull.check() );
    }
    THEN( "Its convex hull has 8 vertices" ) {
      REQUIRE( hull.nbVertices() == 8 );
    }
    THEN( "Its convex hull has 16 facets" ) {
      REQUIRE( hull.nbFacets() == 16 );
    }
  }
  GIVEN( "Given 100 random point in a ball of radius 50 " ) {
    std::vector<Point> V = randomPointsInBall< Point >( 100, 50 );
    QHull hull;
    hull.setInput( V, false );
    hull.computeConvexHull();
    THEN( "The convex hull is valid and contains every point" ) {
      REQUIRE( hull.check() );
    }
    THEN( "Its convex hull has fewer vertices than input points" ) {
      REQUIRE( hull.nbVertices() < hull.nbPoints() );
    }
  }
}


SCENARIO( "QuickHull< ConvexHullIntegralKernel< 2 > > dimensionality tests", "[quickhull][integral_kernel][2d][not_full_dimensional]" )
{
  typedef ConvexHullIntegralKernel< 4 >    QHKernel;
  typedef QuickHull< QHKernel >            QHull;
  typedef SpaceND< 2, int >                Space;      
  typedef Space::Point                     Point;
  typedef AffineSubset< Point >            Affine;
  
  std::vector< Point > V = { Point{ 3, 1 } };
  GIVEN( "Given 100 aligned points + another not aligned" ) {
    auto X = makeRandomLatticePointsFromDirVectors( 100, V );
    X.push_back( X[ 0 ] + Point(-1,1) );
    std::shuffle( X.begin(), X.end(), g );
    auto I = Affine::affineSubset( X );
    auto d = Affine::affineDimension( X );
    QHull hull;
    hull.setInput( V, false );
    bool ok = hull.computeConvexHull();
    auto status = hull.status();
    THEN( "AffineSubset should detect full dimensionality" ) {
      CAPTURE( d );
      REQUIRE( d == 2 );
    }      
    THEN( "QuickHull should detect full dimensionality" ) {
      REQUIRE( status != QHull::Status::NotFullDimensional );
    }
    THEN( "QuickHull should find 3 vertices" ) {
      REQUIRE( hull.nbVertices() == 3 );
    }
  }
}
