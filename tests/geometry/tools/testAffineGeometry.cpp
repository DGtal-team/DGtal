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
 * @file testAffineGeometry.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2025/08/24
 *
 * Functions for testing class AffineGeometry.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include <random>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/geometry/tools/AffineGeometry.h"
#include "DGtal/geometry/tools/AffineBasis.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

std::random_device rd;
std::mt19937 g(rd());
std::uniform_real_distribution<double> uniform(-1.0, 1.0);

template <typename RealPoint>
void perturbate( RealPoint& x, double perturbation )
{
  for ( auto& c : x ) c += uniform( g ) * perturbation;
}

template <typename RealPoint>
void perturbate( std::vector< RealPoint >& X, double perturbation )
{
  for ( auto& x : X ) perturbate( x, perturbation );
}

template < typename Point >
std::vector< Point >
makeRandomVectors( int nb, int amplitude )
{
  std::uniform_int_distribution<int> U(-amplitude, amplitude);
  std::vector< Point > P;
  for ( auto n = 0; n < nb; ++n )
    {
      Point A;
      for ( auto i = 0; i < Point::dimension; i++ )
        A[ i ] = U( g );
      P.push_back( A );
    }
  return P;
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

template < typename Point >
std::vector< Point >
makeRandomRealPointsFromDirVectors( int nb, const vector< Point>& V )
{
  std::uniform_real_distribution<double> U(-1., 1.);
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
          double l = 5.* U( g );
          B += l * V[ i ];
        }
      P.push_back( B );
    }
  std::shuffle( P.begin(), P.end(), g );
  return P;
}

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class AffineGeometry in 2D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "AffineGeometry< Point2i > unit tests", "[affine_subset][2i]" )
{
  typedef SpaceND< 2, int >                Space;      
  typedef Space::Point                     Point;
  typedef AffineGeometry< Point >            Affine;
  GIVEN( "Given X = { (0,0), (-4,-1), (16,4), (-3,5), (7,3), (5, -2) } of affine dimension 2" ) {
    std::vector<Point> X
      = { Point(0,0), Point(-4,-1), Point(16,4), Point(-3,5), Point(7,3), Point(5, -2) };
    auto I = Affine::affineSubset( X );
    THEN( "It has an affine basis of 3 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 3 );
      REQUIRE( I[ 2 ] == 3 );
    }
  }
  GIVEN( "Given X = { (0,0), (-4,-1), (-8,-2), (8,2), (16,4), (200,50) } of affine dimension 1" ) {
    std::vector<Point> X
      = { Point(0,0), Point(-4,-1), Point(-8,-2), Point(8,2), Point(16,4), Point(200,50) };
    auto I = Affine::affineSubset( X );
    THEN( "It has an affine basis of 2 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 2 );
    }
  }
}

SCENARIO( "AffineGeometry< Point2d > unit tests", "[affine_subset][2d]" )
{
  typedef SpaceND< 2, int >                Space;      
  typedef Space::RealPoint                 Point;
  typedef AffineGeometry< Point >            Affine;
  GIVEN( "Given X = { (0,0), (-4,-1), (16,4), (-3,5), (7,3), (5, -2) } of affine dimension 2" ) {
    std::vector<Point> X
      = { Point(0,0), Point(-4,-1), Point(16,4), Point(-3,5), Point(7,3), Point(5, -2) };
    auto I = Affine::affineSubset( X );
    THEN( "It has an affine basis of 3 points [0,1,3]" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 3 );
      REQUIRE( I[ 2 ] == 3 );
    }
  }
  GIVEN( "Given X = { (0,0), (-4,-1), (-8,-2), (8,2), (16,4), (200,50) } of affine dimension 1" ) {
    std::vector<Point> X
      = { Point(0,0), Point(-4,-1), Point(-8,-2), Point(8,2), Point(16,4), Point(200,50) };
    auto I = Affine::affineSubset( X );
    THEN( "It has an affine basis of 2 points [0,1]" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 2 );
    }
  }
  GIVEN( "Given a perturbated X = { (0,0), (-4,-1), (16,4), (-3,5), (7,3), (5, -2) } of affine dimension 2 by U[-1e-6,1e-6]" ) {
    std::vector<Point> X
      = { Point(0,0), Point(-4,-1), Point(16,4), Point(-3,5), Point(7,3), Point(5, -2) };
    perturbate( X, 1e-6 );
    auto I = Affine::affineSubset( X, 1e-12 );
    THEN( "It has an affine basis of 3 points [0,1,2]" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 3 );
      REQUIRE( I[ 2 ] == 2 );
    }
  }
  GIVEN( "Given a perturbated X = { (0,0), (-4,-1), (-8,-2), (8,2), (16,4), (200,50) } of affine dimension 1 by U[-1e-6,1e-6]" ) {
    std::vector<Point> X
      = { Point(0,0), Point(-4,-1), Point(-8,-2), Point(8,2), Point(16,4), Point(200,50) };
    perturbate( X, 1e-6 );
    auto I = Affine::affineSubset( X, 1e-12 );
    THEN( "It has an affine basis of 3 points [0,1,x]" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 3 );
    }
  }
  GIVEN( "Given a perturbated X = { (0,0), (-4,-1), (-8,-2), (8,2), (16,4), (200,50) } of affine dimension 1 by U[-1e-11,1e-11]" ) {
    std::vector<Point> X
      = { Point(0,0), Point(-4,-1), Point(-8,-2), Point(8,2), Point(16,4), Point(200,50) };
    perturbate( X, 1e-11 );
    auto I = Affine::affineSubset( X, 1e-9 );
    THEN( "It has an affine basis of 2 points [0,1] if tolerance is 1e-9" ) {
      CAPTURE( X );
      CAPTURE( I );
      REQUIRE( I.size() == 2 );
    }
  }
}

SCENARIO( "AffineGeometry< Point2i > orthogonal tests", "[orthogonal_vector][2i]" )
{
  typedef SpaceND< 2, int >                Space;      
  typedef Space::Point                     Point;
  typedef AffineGeometry< Point >          Affine;
  GIVEN( "Given basis B = { (7,3) } " ) {
    std::vector<Point> B = { Point(7,3) };
    Affine::completeBasis( B, false );
    THEN( "The complete basis has dimension 2" ) {
      CAPTURE( B );
      REQUIRE( B.size() == 2 );
    }
    THEN( "The last vector is non null and orthogonal to all the others" ) {
      CAPTURE( B.back() );
      REQUIRE( B.back().normInfinity() > 0 );
      REQUIRE( B.back().dot( B[ 0 ] ) == 0 );
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class AffineGeometry in 3D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "AffineGeometry< Point3i > unit tests", "[affine_subset][3i]" )
{
  typedef SpaceND< 3, int >                Space;      
  typedef Space::Point                     Point;
  typedef AffineGeometry< Point >            Affine;
  GIVEN( "Given X = { (1, 0, 0), (2, 1, 0), (3, 1, 1), (3, 2, 0), (5, 2, 2), (4, 2, 1)} of affine dimension 2" ) {
    std::vector<Point> X
      = { Point{1, 0, 0}, Point{2, 1, 0}, Point{3, 1, 1}, Point{3, 2, 0}, Point{5, 2, 2}, Point{4, 2, 1} };

    auto I = Affine::affineSubset( X );
    THEN( "It has an affine basis of 3 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 3 );
      REQUIRE( I[ 2 ] == 2 );
    }
  }
  GIVEN( "Given X = { (1, 0, 0), (2, 1, 0), (3, 1, 1), (3, 2, 0), (5, 2, 2), (4, 2, 1), (7, 3, 2)} of affine dimension 3" ) {
    std::vector<Point> X
      = { Point{1, 0, 0}, Point{2, 1, 0}, Point{3, 1, 1}, Point{3, 2, 0}, Point{5, 2, 2}, Point{4, 2, 1}, Point{7, 3, 2} };

    auto I = Affine::affineSubset( X );
    THEN( "It has an affine basis of 4 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 4 );
      REQUIRE( I[ 2 ] == 2 );
      REQUIRE( I[ 3 ] == 6 );
    }
  }
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 1 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0 } };
    auto X = makeRandomLatticePointsFromDirVectors( 20, V );
    auto I = Affine::affineSubset( X );
    THEN( "It has an affine basis of 2 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 2 );
    }
  }
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 2 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0 }, Point{ -2, -1, 2 } };
    auto X = makeRandomLatticePointsFromDirVectors( 20, V );
    auto I = Affine::affineSubset( X );
    THEN( "It has an affine basis of 3 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 3 );
    }
  }
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 3 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0 }, Point{ -2, -1, 2 }, Point{ -1, 4, 3 } };
    auto X = makeRandomLatticePointsFromDirVectors( 20, V );
    auto I = Affine::affineSubset( X );
    THEN( "It has an affine basis of 4 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 4 );
    }
  }
}

SCENARIO( "AffineGeometry< Point3i > orthogonal tests", "[orthogonal_vector][3i]" )
{
  typedef SpaceND< 3, int >                Space;      
  typedef Space::Point                     Point;
  typedef AffineGeometry< Point >          Affine;
  GIVEN( "Given basis B = { (7,3,1), (2,-5,1) } " ) {
    std::vector<Point> B = { Point(7,3,1), Point( 2,-5,1) };
    Point n = B[ 0 ].crossProduct( B[ 1 ] );
    Affine::completeBasis( B, false );
    THEN( "The complete basis has dimension 3" ) {
      CAPTURE( B );
      REQUIRE( B.size() == 3 );
    }
    THEN( "The last vector is non null and orthogonal to all the others" ) {
      CAPTURE( B.back() );
      REQUIRE( B.back().normInfinity() > 0 );
      REQUIRE( B.back().dot( B[ 0 ] ) == 0 );
      REQUIRE( B.back().dot( B[ 1 ] ) == 0 );
    }
    THEN( "The last vector is the cross product of the two others" ) {
      CAPTURE( B.back() );
      REQUIRE( B.back() == n );
    }
  }
  GIVEN( "Given basis B = { (7,3,1) } " ) {
    std::vector<Point> B = { Point(7,3,1) };
    Affine::completeBasis( B, false );
    THEN( "The complete basis has dimension 3" ) {
      CAPTURE( B );
      REQUIRE( B.size() == 3 );
    }
    THEN( "The mid vector is non null and trivial" ) {
      CAPTURE( B[ 1 ] );
      REQUIRE( B[ 1 ].norm1() == 1 );
    }
    THEN( "The last vector is non null and orthogonal to all the others" ) {
      CAPTURE( B.back() );
      REQUIRE( B.back().normInfinity() > 0 );
      REQUIRE( B.back().dot( B[ 0 ] ) == 0 );
      REQUIRE( B.back().dot( B[ 1 ] ) == 0 );
    }
    THEN( "The last vector is the cross product of the two others" ) {
      Point n = B[ 0 ].crossProduct( B[ 1 ] );
      CAPTURE( B.back() );
      REQUIRE( B.back() == n );
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class AffineGeometry in 4D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "AffineGeometry< Point4i > unit tests", "[affine_subset][4i]" )
{
  typedef SpaceND< 4, int >                Space;      
  typedef Space::Point                     Point;
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 1 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0, 2 } };
    auto X = makeRandomLatticePointsFromDirVectors( 20, V );
    auto I = functions::computeAffineSubset( X );
    auto B = functions::computeAffineBasis( X );
    THEN( "It has an affine basis of 2 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 2 );
    }
    THEN( "It has an affine basis of 1 vector" ) {
      REQUIRE( B.second.size() == 1 );
    }

  }
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 2 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0, 2 }, Point{ -2, -1, 2, 7 } };
    auto X = makeRandomLatticePointsFromDirVectors( 20, V );
    auto I = functions::computeAffineSubset( X );
    auto B = functions::computeAffineBasis( X );
    THEN( "It has an affine basis of 3 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 3 );
    }
    THEN( "It has an affine basis of 2 vectors" ) {
      REQUIRE( B.second.size() == 2 );
    }
  }
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 3 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0, 2 }, Point{ -2, -1, 2, 7 }, Point{ -1, 4, 3, -1  } };
    auto X = makeRandomLatticePointsFromDirVectors( 20, V );
    auto I = functions::computeAffineSubset( X );
    auto B = functions::computeAffineBasis( X );
    THEN( "It has an affine basis of 4 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 4 );
    }
    THEN( "It has an affine basis of 3 vectors" ) {
      REQUIRE( B.second.size() == 3 );
    }
  }
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 4 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0, 2 }, Point{ -2, -1, 2, 7 }, Point{ -1, 4, 3, -1 }, Point{ 2, 1, -3, -4 } };
    auto X = makeRandomLatticePointsFromDirVectors( 20, V );
    auto I = functions::computeAffineSubset( X );
    auto B = functions::computeAffineBasis( X, 1e-10 );
    THEN( "It has an affine basis of 5 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 5 );
    }
    THEN( "It has an affine basis of 4 vectors" ) {
      REQUIRE( B.second.size() == 4 );
    }
  }
}

SCENARIO( "AffineGeometry< Point4d > unit tests", "[affine_subset][4d]" )
{
  // NB: 1e-10 in tolerance is ok for these examples.
  // max norm of rejected vectors are 2e-12.
  typedef SpaceND< 4, int >                Space;      
  typedef Space::RealPoint                 Point;
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 1 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0, 2 } };
    auto X = makeRandomRealPointsFromDirVectors( 20, V );
    auto I = functions::computeAffineSubset( X, 1e-10 );
    auto B = functions::computeAffineBasis( X, 1e-10 );
    THEN( "It has an affine subset of 2 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 2 );
    }
    THEN( "It has an affine basis of 1 vector" ) {
      REQUIRE( B.second.size() == 1 );
    }
  }
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 2 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0, 2 }, Point{ -2, -1, 2, 7 } };
    auto X = makeRandomRealPointsFromDirVectors( 20, V );
    auto I = functions::computeAffineSubset( X, 1e-10 );
    auto B = functions::computeAffineBasis( X, 1e-10 );
    THEN( "It has an affine subset of 3 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 3 );
    }
    THEN( "It has an affine basis of 2 vectors" ) {
      REQUIRE( B.second.size() == 2 );
    }
  }
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 3 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0, 2 }, Point{ -2, -1, 2, 7 }, Point{ -1, 4, 3, -1  } };
    auto X = makeRandomRealPointsFromDirVectors( 20, V );
    auto I = functions::computeAffineSubset( X, 1e-10 );
    auto B = functions::computeAffineBasis( X, 1e-10 );    
    THEN( "It has an affine subset of 4 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 4 );
    }
    THEN( "It has an affine basis of 3 vectors" ) {
      REQUIRE( B.second.size() == 3 );
    }
  }
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 4 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0, 2 }, Point{ -2, -1, 2, 7 }, Point{ -1, 4, 3, -1 }, Point{ 2, 1, -3, -4 } };
    auto X = makeRandomRealPointsFromDirVectors( 20, V );
    auto I = functions::computeAffineSubset( X, 1e-10 );
    auto B = functions::computeAffineBasis( X, 1e-10 );    
    THEN( "It has an affine subset of 5 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 5 );
    }
    THEN( "It has an affine basis of 4 vectors" ) {
      REQUIRE( B.second.size() == 4 );
    }
  }
}

SCENARIO( "AffineGeometry< Point4i > orthogonal tests", "[orthogonal_vector][4i]" )
{
  typedef SpaceND< 4, int >                Space;      
  typedef Space::Point                     Point;
  typedef AffineGeometry< Point >          Affine;
  GIVEN( "Given basis B = { (7,3,1,0), (2,-5,1,2), (-1,2,2,-3) } " ) {
    std::vector<Point> B = { Point(7,3,1,0), Point(2,-5,1,2), Point(-1,2,2,-3) };
    Affine::completeBasis( B, false );
    THEN( "The complete basis has dimension 4" ) {
      CAPTURE( B );
      REQUIRE( B.size() == 4 );
    }
    THEN( "The last vector is non null and orthogonal to all the others" ) {
      CAPTURE( B.back() );
      REQUIRE( B.back().normInfinity() > 0 );
      REQUIRE( B.back().dot( B[ 0 ] ) == 0 );
      REQUIRE( B.back().dot( B[ 1 ] ) == 0 );
      REQUIRE( B.back().dot( B[ 2 ] ) == 0 );
    }
  }
  GIVEN( "Given basis B = { (7,3,1,0), (2,-5,1,2) } " ) {
    std::vector<Point> B = { Point(7,3,1,0), Point(2,-5,1,2) };
    Affine::completeBasis( B, false );
    THEN( "The complete basis has dimension 4" ) {
      CAPTURE( B );
      REQUIRE( B.size() == 4 );
    }
    THEN( "The third vector is non null and trivial" ) {
      CAPTURE( B[ 2 ] );
      REQUIRE( B[ 2 ].norm1() == 1 );
    }
    THEN( "The last vector is non null and orthogonal to all the others" ) {
      CAPTURE( B.back() );
      REQUIRE( B.back().normInfinity() > 0 );
      REQUIRE( B.back().dot( B[ 0 ] ) == 0 );
      REQUIRE( B.back().dot( B[ 1 ] ) == 0 );
      REQUIRE( B.back().dot( B[ 2 ] ) == 0 );
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class AffineBasis in 2D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "AffineBasis< Point2i > unit tests", "[affine_basis][2i]" )
{
  typedef SpaceND< 2, int >                Space;      
  typedef Space::Point                     Point;
  typedef AffineBasis< Point >             Basis;
  GIVEN( "Given B = (0,0) + { (8,2), (-4,-1),  (-8,-2), (16,4), (200,50) } of affine dimension 1" ) {
    Point o( 0, 0 );
    std::vector<Point> X
      = { Point(8,2), Point(-4,-1), Point(-8,-2), Point(16,4), Point(200,50) };
    Basis B( o, X );
    THEN( "When reduced, it has dimension 1" ) {
      CAPTURE( B.basis() );
      REQUIRE( B.dimension() == 1 );
    }
    THEN( "When reduced, it is the vector (-4,-1) or (4,1)" ) {
      Point b0 = B.basis()[ 0 ];
      CAPTURE( b0 );
      REQUIRE( ((b0 == Point(-4,-1)) || (b0 == Point(4,1))) );
    }
  }
}

SCENARIO( "AffineBasis< Point4i > unit tests", "[affine_basis][4i]" )
{
  typedef SpaceND< 4, int >                Space;      
  typedef Space::Point                     Point;
  typedef AffineBasis< Point >             Basis;
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 1 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0, 2 } };
    auto X = makeRandomLatticePointsFromDirVectors( 20, V );
    Basis B( X );
    THEN( "When reduced, it has dimension 1" ) {
      CAPTURE( B.basis() );
      REQUIRE( B.dimension() == 1 );
    }
    THEN( "When reduced, it is the vector V[0] or -V[0]" ) {
      CAPTURE( V );
      Point b0 = B.basis()[ 0 ];
      CAPTURE( b0 );
      REQUIRE( ((b0 == V[0]) || (b0 == -V[0])) );
    }
  }
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 2 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0, 2 }, Point{ -2, -1, 2, 7 } };
    auto X = makeRandomLatticePointsFromDirVectors( 20, V );
    Basis B( X );
    THEN( "When reduced, basis has dimension 2" ) {
      CAPTURE( B.basis() );
      REQUIRE( B.dimension() == 2 );
    }
    THEN( "When reduced, basis spans vectors V[i] or -V[i]" ) {
      CAPTURE( V );
      Point b0 = B.basis()[ 0 ];
      Point b1 = B.basis()[ 1 ];
      CAPTURE( b0 );
      CAPTURE( b1 );
      REQUIRE( B.isParallel( V[ 0 ] ) );
      REQUIRE( B.isParallel( V[ 1 ] ) );
    }
    THEN( "every point of X has rational coordinates with no remainder" ) {
      unsigned int nb_ok = 0;
      for ( auto p : X )
        {
          const auto [d, lambda, rem ] = B.decompose( p );
          std::cout << "p=" << p << " d=" << d
                    << " lambda=" << lambda << " rem=" << rem << "\n";
          nb_ok += ( rem == Point::zero ) ? : 1;
        }
      REQUIRE( nb_ok == X.size() );
    }
    THEN( "every lattice point can be written as a linear combination" ) {
      auto Y = makeRandomVectors<Point>( 20, 10 );
      unsigned int nb_ok = 0;
      for ( auto y : Y )
        {
          const auto p = y + B.first;
          const auto [d, lambda, rem ] = B.decompose( p );
          auto q = B.recompose( d, lambda, rem );
          std::cout << "p=" << p << " d=" << d
                    << " lambda=" << lambda << " rem=" << rem
                    << " q=" << q << "\n";
          nb_ok += ( p == q ) ? : 1;
        }
      REQUIRE( nb_ok == Y.size() );
    }
  }
}

SCENARIO( "AffineBasis< Point4i > projection tests", "[affine_basis][4i][4d]" )
{
  typedef SpaceND< 4, int >                Space;      
  typedef Space::Point                     Point;
  typedef SpaceND< 2, int >                Space2;      
  typedef Space2::Point                    PPoint;
  typedef AffineBasis< Point >             Basis;
  typedef Space::RealPoint                 RealPoint;
  typedef Space2::RealPoint                PRealPoint;
  typedef AffineBasis< RealPoint >         RealBasis;
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 2 lattice vectors, and Y the same set but with real coordinates" ) {
    std::vector< Point > V = { Point{ 3, 4, 0, 2 }, Point{ -2, -1, 5, -7 } };
    auto X = makeRandomLatticePointsFromDirVectors( 20, V );
    Basis B( X );
    std::vector< RealPoint > Y( X.size() );
    for ( auto i = 0; i < Y.size(); i++ )
      Basis::transform( Y[ i ], X[ i ] );
    RealBasis RB( Y );
    std::vector< PPoint >     pX;
    std::vector< PRealPoint > pY;
    auto lcm  = B .projectPoints( pX, X );
    auto rlcm = RB.projectPoints( pY, Y );
    THEN( "When reduced, their affine bases has same dimension 2" ) {
      CAPTURE( B.basis() );
      CAPTURE( RB.basis() );
      REQUIRE( B.dimension() == 2 );
      REQUIRE( RB.dimension() == 2 );
    }
    THEN( "Their projections have the same geometry (i.e. orientations within points)" ) {
      CAPTURE( lcm );
      CAPTURE( rlcm );
      CAPTURE( pX );
      CAPTURE( pY );
      REQUIRE( pX.size() == pY.size() );
      // Computing arbitrary determinants between triplets of points
      const std::size_t nb = 2000;
      std::size_t    nb_ok = 0;
      const double     eps = 1e-10 * double( lcm ) / rlcm;
      for ( auto i = 0; i < nb; i++ )
        {
          const std::size_t j = rand() % pX.size();
          const std::size_t k = rand() % pX.size();
          const std::size_t l = rand() % pX.size();
          const auto u    = pX[ k ] - pX[ j ];
          const auto v    = pX[ l ] - pX[ j ];
          const auto ru   = pY[ k ] - pY[ j ];
          const auto rv   = pY[ l ] - pY[ j ];
          const auto det  = u [ 0 ] * v [ 1 ] - u [ 1 ] * v [ 0 ];
          const auto rdet = ru[ 0 ] * rv[ 1 ] - ru[ 1 ] * rv[ 0 ];
          if ( rdet > eps )       nb_ok += ( det >  0 ) ? 1 : 0;
          else if ( rdet < -eps ) nb_ok += ( det <  0 ) ? 1 : 0;
          else                    nb_ok += ( det == 0 ) ? 1 : 0;
        }
      REQUIRE( nb_ok == nb );
    }
  }
}
    

SCENARIO( "AffineGeometry< Z3 > bug", "[affine_geom][3d]" )
{
  typedef SpaceND<3,int>          Space;
  typedef Space::Point            Point;

  typedef AffineGeometry< Point > Affine;
  typedef AffineBasis< Point >    Basis;  

  std::vector< Point > X = { {-46, 38, -43}, {27, -89, 20}, {53, 26, -57} };
  auto  ref_basis = functions::computeAffineBasis ( X );
  auto  ref       = ref_basis.first;
  auto& basis     = ref_basis.second;
  auto  C         = ( X[1]-X[0] ).crossProduct( X[2]-X[0] );
  auto  sC        = functions::computeSimplifiedVector( C );
  WHEN( "Computing orthogonal vector" ) {
    Point N;
    functions::computeOrthogonalVector( N, basis );
    THEN( "It is non null" ) {
      CAPTURE( N );
      REQUIRE( N != Point::zero );
    }
    THEN( "It corresponds to the reduced cross product" ) {
      CAPTURE( C );
      CAPTURE( sC );
      REQUIRE( N == sC );
    }
  }
}
