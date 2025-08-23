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
 * @file testAffineSubset.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/02/01
 *
 * Functions for testing class AffineSubset.
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
#include "DGtal/geometry/tools/AffineSubset.h"
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
  for ( auto& c : x ) x += uniform( g ) * perturbation;
}

template <typename RealPoint>
void perturbate( std::vector< RealPoint >& X, double perturbation )
{
  for ( auto& x : X ) perturbate( x, perturbation );
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
// Functions for testing class AffineSubset in 2D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "AffineSubset< Point2i > unit tests", "[affine_subset][2i]" )
{
  typedef SpaceND< 2, int >                Space;      
  typedef Space::Point                     Point;
  typedef AffineSubset< Point >            Affine;
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

SCENARIO( "AffineSubset< Point2d > unit tests", "[affine_subset][2d]" )
{
  typedef SpaceND< 2, int >                Space;      
  typedef Space::RealPoint                 Point;
  typedef AffineSubset< Point >            Affine;
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

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class AffineSubset in 3D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "AffineSubset< Point3i > unit tests", "[affine_subset][3i]" )
{
  typedef SpaceND< 3, int >                Space;      
  typedef Space::Point                     Point;
  typedef AffineSubset< Point >            Affine;
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

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class AffineSubset in 4D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "AffineSubset< Point4i > unit tests", "[affine_subset][4i]" )
{
  typedef SpaceND< 4, int >                Space;      
  typedef Space::Point                     Point;
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 1 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0, 2 } };
    auto X = makeRandomLatticePointsFromDirVectors( 20, V );
    auto I = computeAffineSubset( X );
    auto B = computeAffineBasis( X );
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
    auto I = computeAffineSubset( X );
    auto B = computeAffineBasis( X );
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
    auto I = computeAffineSubset( X );
    auto B = computeAffineBasis( X );
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
    auto I = computeAffineSubset( X );
    auto B = computeAffineBasis( X, 1e-10 );
    THEN( "It has an affine basis of 5 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 5 );
    }
    THEN( "It has an affine basis of 4 vectors" ) {
      REQUIRE( B.second.size() == 4 );
    }
  }
}

SCENARIO( "AffineSubset< Point4d > unit tests", "[affine_subset][4d]" )
{
  // NB: 1e-10 in tolerance is ok for these examples.
  // max norm of rejected vectors are 2e-12.
  typedef SpaceND< 4, int >                Space;      
  typedef Space::RealPoint                 Point;
  GIVEN( "Given X a set of randomly generated points by adding linear combinations of 1 lattice vectors" ) {
    std::vector< Point > V = { Point{ 3, 1, 0, 2 } };
    auto X = makeRandomRealPointsFromDirVectors( 20, V );
    auto I = computeAffineSubset( X, 1e-10 );
    auto B = computeAffineBasis( X, 1e-10 );
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
    auto I = computeAffineSubset( X, 1e-10 );
    auto B = computeAffineBasis( X, 1e-10 );
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
    auto I = computeAffineSubset( X, 1e-10 );
    auto B = computeAffineBasis( X, 1e-10 );    
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
    auto I = computeAffineSubset( X, 1e-10 );
    auto B = computeAffineBasis( X, 1e-10 );    
    THEN( "It has an affine subset of 5 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 5 );
    }
    THEN( "It has an affine basis of 4 vectors" ) {
      REQUIRE( B.second.size() == 4 );
    }
  }
}

