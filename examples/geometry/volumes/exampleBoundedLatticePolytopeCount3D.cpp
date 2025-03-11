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
 * @file geometry/volumes/exampleBoundedLatticePolytopeCount3D.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2022/06/19
 *
 * An example file named exampleBoundedLatticePolytopeCount3D.
 *
 * This file is part of the DGtal library.
 */

/**
   Computes the number of lattice points within random polytopes in
   two different ways: (1) by simple range scanning and inside test,
   (2) by computing intersections along an axis.

   Second method is much faster (from 5x to easily 100x).

\verbatim
# 100 polytopes made from 10 points in range [-50:50]^3
./examples/geometry/tools/exampleBoundedLatticePolytopeCount3D 100 10 50
\endverbatim
outputs
\verbatim
New Block [Compute polytope]
EndBlock [Compute polytope] (14.6467 ms)
Computed 100 polytopes with 19.11 facets on average, in 14.6467 ms.
New Block [Compute number of lattice points within polytope (slow)]
EndBlock [Compute number of lattice points within polytope (slow)] (853.593 ms)
New Block [Compute number of lattice points within polytope (fast)]
EndBlock [Compute number of lattice points within polytope (fast)] (61.2751 ms)
Computed inside points is OK
Reference method computed 15234308 points in 853.593 ms.
Fast method computed 15234308 points in 61.2751 ms.
\endverbatim

\example geometry/volumes/exampleBoundedLatticePolytopeCount3D.cpp
*/

#include "DGtal/base/Common.h"
#include "DGtal/geometry/volumes/ConvexityHelper.h"
#include "DGtal/geometry/volumes/BoundedLatticePolytope.h"

int main( int argc, char* argv[] )
{
  int N  = argc > 1 ? atoi( argv[ 1 ] ) : 100;  // nb of polytopes
  int nb = argc > 2 ? atoi( argv[ 2 ] ) : 10;   // nb points per polytope
  int R  = argc > 3 ? atoi( argv[ 3 ] ) : 50;   // max diameter of shape

  typedef int64_t                              Integer;
  typedef DGtal::ConvexityHelper< 3, Integer > Helper;
  typedef Helper::LatticePolytope              Polytope;
  typedef Polytope::Point                      Point;
  // Compute all polytopes
  DGtal::trace.beginBlock( "Compute 3D polytopes" );
  std::vector< Polytope > polytopes;
  int sum_nb_facets = 0;
  for ( int i = 0; i < N; i++ )
    {
      std::vector< Point > V;
      for ( int j = 0; j < nb; j++ ) {
        Point p( rand() % (2*R+1) - R, rand() % (2*R+1) - R, rand() % (2*R+1) - R );
        V.push_back( p );
      }
      Polytope P = Helper::computeLatticePolytope( V );
      sum_nb_facets += P.nbHalfSpaces();
      polytopes.push_back( P );
    }
  double t1 = DGtal::trace.endBlock();
  DGtal::trace.info() << "Computed " << N
               << " 3D polytopes with " << ( sum_nb_facets / (double) N )
               << " facets on average, in " << t1 << " ms." << std::endl;
  // Count interior points (slow method)
  DGtal::trace.beginBlock( "Compute number of lattice points within polytope (slow)" );
  std::size_t slow_nb = 0;
  std::vector< Integer > slow_counts;
  for ( const auto& P : polytopes )
    {
      const auto nb2 = P.countByScanning();
      slow_nb      += nb2;
      slow_counts.push_back( nb2 ); 
    }
  double t2 = DGtal::trace.endBlock();
  // Count interior points (fast method)
  DGtal::trace.beginBlock( "Compute number of lattice points within polytope (fast)" );
  std::size_t fast_nb = 0;
  std::vector< Integer > fast_counts;
  for ( const auto& P : polytopes )
    {
      const auto nb2 = P.count();
      fast_nb      += nb2;
      fast_counts.push_back( nb2 );
    }
  double t3 = DGtal::trace.endBlock();
  bool ok = std::equal( slow_counts.cbegin(), slow_counts.cend(), fast_counts.cbegin() );
  DGtal::trace.info() << "Computed inside points is " << ( ok ? "OK" : "ERROR" ) << std::endl;
  DGtal::trace.info() << "Reference method computed " << slow_nb
                      << " points in " << t2 << " ms." << std::endl;
  DGtal::trace.info() << "Fast method computed " << fast_nb
                      << " points in " << t3 << " ms." << std::endl;
  return 0;
} 
  
