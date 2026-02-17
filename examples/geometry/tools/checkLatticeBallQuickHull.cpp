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
 * @file geometry/tools/checkLatticeBallQuickHull.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/01/01
 *
 * An example file named checkLatticeBallQuickHull.
 *
 * This file is part of the DGtal library.
 */

/**
   Computation of the convex hull of a set of lattice points in
   arbitrary dimension by Quick Hull algorithm, for arbitrary integer
   types, and check of the output. This example is used to evaluate
   the overflow risk when using limited integers for computing the
   convex hull.

   You specify integer types with 4th parameter in `int64`, `bigint`, or `allbigint`.

\verbatim
# 1000 5D points in digital ball of radius 1e8, using int64 for lattice points
# and BigInteger for internal computations
./examples/geometry/tools/checkLatticeBallQuickHull 5 1000 1e8 bigint
\endverbatim
outputs
\verbatim
#points=1000 #vertices=486 #facets=10610
purge duplicates= 1 ms.
init simplex    = 9 ms.
quickhull core  = 17557 ms.
compute vertices= 7 ms.
total time      = 17574 ms.
Checking hull ...
 ... in 18.0777s => OK
\endverbatim

@see \ref moduleQuickHull

\example geometry/tools/checkLatticeBallQuickHull.cpp
*/

#include <cstdlib>
#include <iostream>
#include <chrono>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/tools/QuickHull.h"

template <typename Point>
std::vector< Point >
randomPointsInBall( int nb, double R )
{
  std::vector< Point > V;
  DGtal::int64_t iR = DGtal::int64_t( round( R ) );
  Point c = Point::diagonal( iR );
  double R2 = (double) R * (double) R;
  for ( int i = 0; i < nb; ) {
    Point p;
    for ( DGtal::Dimension k = 0; k < Point::dimension; ++k )
      p[ k ] = DGtal::int64_t( round( (double) rand() * 2.0 * R / (double) RAND_MAX ));
    if ( ( p - c ).squaredNorm() < R2 ) { V.push_back( p - c ); i++; }
  }
  return V;
}

template <typename Point>
std::vector< Point >
randomPointsInBallBigInteger( int nb, double R )
{
  std::vector< Point > V;
  DGtal::int64_t iR = DGtal::int64_t( round( R ) );
  typedef DGtal::IntegerConverter< Point::dimension, DGtal::BigInteger > Converter;
  Point c = Point::diagonal( Converter::cast( iR ) );
  double R2 = (double) R * (double) R;
  for ( int i = 0; i < nb; ) {
    Point p;
    for ( DGtal::Dimension k = 0; k < Point::dimension; ++k )
      p[ k ] = Converter::cast( DGtal::int64_t( round( (double) rand() * 2.0 * R / (double) RAND_MAX )) );
    if ( ( p - c ).squaredNorm() < R2 ) { V.push_back( p - c ); i++; }
  }
  return V;
}

template < DGtal::Dimension dim, typename Integer >
bool
checkQuickHull( int nb, double R )
{
  typedef DGtal::ConvexHullIntegralKernel< dim, DGtal::int64_t, Integer > Kernel;
  typedef DGtal::QuickHull< Kernel >         ConvexHull;
  typedef typename ConvexHull::Point Point;

  const auto V = randomPointsInBall< Point >( nb, R );
  ConvexHull hull;
  hull.setInput( V );
  hull.computeConvexHull();
  std::cout << "#points="    << hull.nbPoints()
            << " #vertices=" << hull.nbVertices()
            << " #facets="   << hull.nbFacets() << std::endl;
  double total_time = 0;
  std::for_each( hull.timings.cbegin(), hull.timings.cend(),
                 [&total_time] ( double t ) { total_time += t; } );
  std::cout << "purge duplicates= " << round(hull.timings[ 0 ]) << " ms." << std::endl;
  std::cout << "init simplex    = " << round(hull.timings[ 1 ]) << " ms." << std::endl;
  std::cout << "quickhull core  = " << round(hull.timings[ 2 ]) << " ms." << std::endl;
  std::cout << "compute vertices= " << round(hull.timings[ 3 ]) << " ms." << std::endl;
  std::cout << "total time      = " << round(total_time) << " ms." << std::endl;
  std::cout << "Checking hull ... " << std::endl;
  auto start = std::chrono::steady_clock::now();
  bool ok = hull.check();
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end-start;
  std::cout << " ... in " << elapsed_seconds.count() << "s"
            << " => " << ( ok ? "OK" : "ERROR" ) << std::endl;
  return ok;
}

template < DGtal::Dimension dim >
bool
checkQuickHullBigInteger( int nb, double R )
{
  typedef DGtal::ConvexHullIntegralKernel< dim, DGtal::BigInteger, DGtal::BigInteger > Kernel;
  typedef DGtal::QuickHull< Kernel >         ConvexHull;
  typedef typename ConvexHull::Point Point;

  const auto V = randomPointsInBallBigInteger< Point >( nb, R );
  ConvexHull hull;
  hull.setInput( V );
  hull.computeConvexHull();
  std::cout << "#points="    << hull.nbPoints()
            << " #vertices=" << hull.nbVertices()
            << " #facets="   << hull.nbFacets() << std::endl;
  double total_time = 0;
  std::for_each( hull.timings.cbegin(), hull.timings.cend(),
                 [&total_time] ( double t ) { total_time += t; } );
  std::cout << "purge duplicates= " << round(hull.timings[ 0 ]) << " ms." << std::endl;
  std::cout << "init simplex    = " << round(hull.timings[ 1 ]) << " ms." << std::endl;
  std::cout << "quickhull core  = " << round(hull.timings[ 2 ]) << " ms." << std::endl;
  std::cout << "compute vertices= " << round(hull.timings[ 3 ]) << " ms." << std::endl;
  std::cout << "total time      = " << round(total_time) << " ms." << std::endl;
  std::cout << "Checking hull ... " << std::endl;
  auto start = std::chrono::steady_clock::now();
  bool ok = hull.check();
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end-start;
  std::cout << " ... in " << elapsed_seconds.count() << "s"
            << " => " << ( ok ? "OK" : "ERROR" ) << std::endl;
  return ok;
}

int main( int argc, char* argv[] )
{
  int       dim = argc > 1 ? atoi( argv[ 1 ] ) : 3;    // dimension
  int        nb = argc > 2 ? atoi( argv[ 2 ] ) : 1000;  // nb points
  double      R = argc > 3 ? atof( argv[ 3 ] ) : 100.0; // radius of ball
  std::string i = argc > 4 ? argv[ 4 ] : "int64";     // type for internal integers
  bool       ok = true;
  if ( ( i != "int64" ) && ( i != "bigint" ) && ( i != "allbigint" ) )
    {
      DGtal::trace.error() << "Integer type in {int64,bigint,allbigint}" << std::endl;
      ok = false;
    }
  if ( ( dim < 2 ) || ( dim > 6 ) )
    {
      DGtal::trace.error() << "Dimension must be in {2,3,4,5,6}" << std::endl;
      ok = false;
    }
  if ( ! ok ) return 1;
  if ( i == "bigint" )
    {
      switch( dim ) {
      case 2 : ok = checkQuickHull< 2, DGtal::BigInteger >( nb, R ); break;
      case 3 : ok = checkQuickHull< 3, DGtal::BigInteger >( nb, R ); break;
      case 4 : ok = checkQuickHull< 4, DGtal::BigInteger >( nb, R ); break;
      case 5 : ok = checkQuickHull< 5, DGtal::BigInteger >( nb, R ); break;
      case 6 : ok = checkQuickHull< 6, DGtal::BigInteger >( nb, R ); break;
      }
    }
  else if ( i == "int64" )
    {
      switch( dim ) {
      case 2 : ok = checkQuickHull< 2, DGtal::int64_t >( nb, R ); break;
      case 3 : ok = checkQuickHull< 3, DGtal::int64_t >( nb, R ); break;
      case 4 : ok = checkQuickHull< 4, DGtal::int64_t >( nb, R ); break;
      case 5 : ok = checkQuickHull< 5, DGtal::int64_t >( nb, R ); break;
      case 6 : ok = checkQuickHull< 6, DGtal::int64_t >( nb, R ); break;
      }
    }
  else if ( i == "allbigint" )
    {
      switch( dim ) {
      case 2 : ok = checkQuickHullBigInteger< 2 >( nb, R ); break;
      case 3 : ok = checkQuickHullBigInteger< 3 >( nb, R ); break;
      case 4 : ok = checkQuickHullBigInteger< 4 >( nb, R ); break;
      case 5 : ok = checkQuickHullBigInteger< 5 >( nb, R ); break;
      case 6 : ok = checkQuickHullBigInteger< 6 >( nb, R ); break;
      }
    }
  return ok ? 0 : 1;
}
