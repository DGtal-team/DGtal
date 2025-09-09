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
 * @file geometry/tools/exampleGenericLatticeConvexHull.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2025/09/08
 *
 * An example file named exampleGenericLatticeConvexHull
 *
 * This file is part of the DGtal library.
 */

/**
   Computes the affine dimension of the convex hull of many small sets
   of lattice points and computes various statistics.
   
\verbatim
0.024% are 0-dimensional #V=1 #F=0 (24/24)
11.2% are 1-dimensional #V=2 #F=0 (11208/11208)
11.3% are 2-dimensional #V=3.02 #F=3.02 (11255/11255)
77.5% are 3-dimensional #V=6.41 #F=8.62 (77513/77513)
\endverbatim

@see \ref moduleQuickHull

@example geometry/tools/exampleGenericLatticeConvexHull.cpp
 */

#include <iostream>
#include <format>
#include <vector>
//! [generic-qhull-example]
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/geometry/tools/GenericLatticeConvexHull.h"
int main()
{
  using namespace DGtal;
  // point coordinates are int (32 bits), computation with int64_t
  typedef GenericLatticeConvexHull< 3, int, int64_t > QHull; 
  typedef SpaceND< 3, int >                           Space;
  typedef Space::Point                                Point;

  std::size_t nb_ok      [ 4 ] = { 0, 0, 0, 0 };
  std::size_t nb_per_dim [ 4 ] = { 0, 0, 0, 0 };
  std::size_t nb_vertices[ 4 ] = { 0, 0, 0, 0 };
  std::size_t nb_facets  [ 4 ] = { 0, 0, 0, 0 };
  const std::size_t nb = 100000;
  for ( std::size_t n = 0; n < nb; ++n )
    {
      // Create a random set of points
      std::vector< Point > X;
      int m = 2 + rand() % 9; //< number of points
      for ( int i = 0; i < m; i++ )
        X.push_back( Point{ rand() % 8, rand() % 8, rand() % 8 } );
      // Compute convex hull
      QHull hull;
      bool ok = hull.compute( X ); 
      int  k  = hull.affine_dimension;    // affine dimension of X
      // Compute statistics
      nb_per_dim [ k ] += 1;
      nb_vertices[ k ] += hull.positions.size(); // positions of vertices
      nb_facets  [ k ] += hull.facets.size();    // facets
      nb_ok      [ k ] += ok ? 1 : 0;
    }
  for ( auto k = 0; k < 4; k++ )
    std::cout << std::setprecision(3) << ( 100.0 * nb_per_dim[ k ] ) / nb
              << "% are " << k << "-dimensional"
              << " #V=" << double( nb_vertices[ k ] ) / nb_per_dim[ k ] 
              << " #F=" << double( nb_facets[ k ] ) / nb_per_dim[ k ]
              << " (" << nb_ok[ k ] << "/" << nb_per_dim[ k ] << ")\n";
  return 0;
}
//! [generic-qhull-example]
