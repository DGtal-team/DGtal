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
 * @file geometry/tools/exampleAffineGeometry.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2025/08/28
 *
 * An example file named exampleAffineGeometry.
 *
 * This file is part of the DGtal library.
 */

/**

   Determines the affine geometry of a set of points.

\verbatim
# Computes an affine subset of 8 points of affine dimension 2 in 3D.
./examples/geometry/tools/exampleAffineGeometry 3 8
\endverbatim
outputs
\verbatim
Dimension is 3
X = [ (21, -29, -21) (-6, 14, 8) (-19, 1, 9) (39, 16, -12) (-26, 29, 23) (-4, 45, 19) (6, 31, 9) (9, 6, -2) (8, 49, 15) ]
Expected dimension of affine set of points X is 2
AffineDim(X) = 2
AffineSubset(X) = [ 0 1 2 ]
AffineBasis(X) =: p+B = (21, -29, -21) + [ (-27, 43, 29) (-40, 30, 30) ]
Independent(X) =: e = (1, 0, 0)
Orthogonal(X) =: n = (420, -350, 910)
Orthogonal(X)/gcd =: ns = (6, -5, 13)
B[0] . ns = (-27, 43, 29) . (6, -5, 13) == 0 (should be 0)
B[1] . ns = (-40, 30, 30) . (6, -5, 13) == 0 (should be 0)
\endverbatim

@see \ref moduleAffineGeometry

\example geometry/tools/exampleAffineGeometry.cpp
*/

#include <iostream>
#include <vector>
#include <random>
#include "DGtal/base/Common.h"
//! [AffineGeometry-Includes]
#include "DGtal/geometry/tools/AffineGeometry.h"
//! [AffineGeometry-Includes]
#include "ConfigExamples.h"

using namespace DGtal;

std::random_device rd;
std::mt19937 g(rd());

template < typename Point >
std::vector< Point >
makeRandomLatticePointsFromDirVectors( int nb, const std::vector< Point>& V )
{
  std::uniform_int_distribution<int> U(-10, 10);
  std::vector< Point > P;
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

template <typename T>
std::ostream&
operator<<( std::ostream& out, const std::vector< T >& object )
{
  out << "[";
  for ( auto t : object ) out << " " << t;
  out << " ]";
  return out;
}

template < typename Point >
bool checkAffineGeometry( int nb, const std::vector< Point>& V )
{
  auto X = makeRandomLatticePointsFromDirVectors( nb, V );
  std::cout << "Dimension is " << Point::dimension << "\n";
  std::cout << "X = " << X << "\n";
  std::cout << "Expected dimension of affine set of points X is "
            << (Point::dimension-1) << "\n";
  std::cout << "AffineDim(X) = " << functions::computeAffineDimension( X ) << "\n";
  auto I = functions::computeAffineSubset( X );
  std::cout << "AffineSubset(X) = " << I << "\n";
  auto B = functions::computeAffineBasis( X );
  std::cout << "AffineBasis(X) =: p+B = " << B.first << " + " << B.second << "\n";
  auto e = functions::computeIndependentVector( B.second );
  std::cout << "Independent(X) =: e = " << e << "\n";
  Point n;
  functions::computeOrthogonalVector( n, B.second );
  std::cout << "Orthogonal(X) =: n = " << n << "\n";
  Point ns = functions::computeSimplifiedVector( n );
  std::cout << "Orthogonal(X)/gcd =: ns = " << ns << "\n";  
  for ( auto i = 0; i < B.second.size(); i++ )
    std::cout << "B[" << i << "] . ns = " << B.second[i] << " . " << ns
              << " == " << B.second[i].dot(ns) << " (should be 0)\n";
  return true;
}


int main( int argc, char* argv[] )
{
  int dim = argc > 1 ? atoi( argv[ 1 ] ) : 3;  // dimension of the space
  if ( dim > 5 ) dim = 5;
  if ( dim < 2 ) dim = 2;
  int nb  = argc > 2 ? atoi( argv[ 2 ] ) : 10;  // number of points
  if ( dim == 2 )
    {
      typedef PointVector< 2, int64_t> Point;
      std::vector< Point > X = { Point{3,1}  };
      return checkAffineGeometry( nb, X );
    }
  if ( dim == 3 )
    {
      typedef PointVector< 3, int64_t> Point;
      std::vector< Point > X = { Point{3,1,-1}, Point{-1,4,2} };
      return checkAffineGeometry( nb, X );
    }
  if ( dim == 4 )
    {
      typedef PointVector< 4, int64_t> Point;
      std::vector< Point > X = { Point{3,1,-1,2}, Point{-1,4,2,0}, Point{0,5,3,1} };
      return checkAffineGeometry( nb, X );
    }
  if ( dim == 5 )
    {
      typedef PointVector< 5, int64_t> Point;
      std::vector< Point > X = { Point{3,1,-1,2,3}, Point{-1,4,2,0,-2}, Point{0,5,3,1,-1}, Point{-4,-3,2,1,0} };
      return checkAffineGeometry( nb, X );
    }
}
