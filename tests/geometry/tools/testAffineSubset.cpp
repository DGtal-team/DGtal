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

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class QuickHull in 2D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "AffineSubset< Point2i > unit tests", "[affine_subset][2i]" )
{
  typedef SpaceND< 2, int >                Space;      
  typedef Space::Point                     Point;
  typedef AffineSubset< Point >            Affine;
  GIVEN( "Given X = { (0,0), (-4,-1), (16,4), (-3,5), (7,3), (5, -2) } of affine dimension 2" ) {
    std::vector<Point> X
      = { Point(0,0), Point(-4,-1), Point(16,4), Point(-3,5), Point(7,3), Point(5, -2) };
    auto I = Affine::affineBasis( X );
    THEN( "It has an affine basis of 3 points" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 3 );
      REQUIRE( I[ 2 ] == 3 );
    }
  }
  GIVEN( "Given X = { (0,0), (-4,-1), (-8,-2), (8,2), (16,4), (200,50) } of affine dimension 1" ) {
    std::vector<Point> X
      = { Point(0,0), Point(-4,-1), Point(-8,-2), Point(8,2), Point(16,4), Point(200,50) };
    auto I = Affine::affineBasis( X );
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
    auto I = Affine::affineBasis( X );
    THEN( "It has an affine basis of 3 points [0,1,3]" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 3 );
      REQUIRE( I[ 2 ] == 3 );
    }
  }
  GIVEN( "Given X = { (0,0), (-4,-1), (-8,-2), (8,2), (16,4), (200,50) } of affine dimension 1" ) {
    std::vector<Point> X
      = { Point(0,0), Point(-4,-1), Point(-8,-2), Point(8,2), Point(16,4), Point(200,50) };
    auto I = Affine::affineBasis( X );
    THEN( "It has an affine basis of 2 points [0,1]" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 2 );
    }
  }
  GIVEN( "Given a perturbated X = { (0,0), (-4,-1), (16,4), (-3,5), (7,3), (5, -2) } of affine dimension 2 by U[-1e-6,1e-6]" ) {
    std::vector<Point> X
      = { Point(0,0), Point(-4,-1), Point(16,4), Point(-3,5), Point(7,3), Point(5, -2) };
    perturbate( X, 1e-6 );
    auto I = Affine::affineBasis( X );
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
    auto I = Affine::affineBasis( X );
    THEN( "It has an affine basis of 3 points [0,1,x]" ) {
      CAPTURE( I );
      REQUIRE( I.size() == 3 );
    }
  }
  GIVEN( "Given a perturbated X = { (0,0), (-4,-1), (-8,-2), (8,2), (16,4), (200,50) } of affine dimension 1 by U[-1e-11,1e-11]" ) {
    std::vector<Point> X
      = { Point(0,0), Point(-4,-1), Point(-8,-2), Point(8,2), Point(16,4), Point(200,50) };
    perturbate( X, 1e-11 );
    auto I = Affine::affineBasis( X, 1e-8 );
    THEN( "It has an affine basis of 2 points [0,1] if tolerance is 1e-8" ) {
      CAPTURE( X );
      CAPTURE( I );
      REQUIRE( I.size() == 2 );
    }
  }
}

