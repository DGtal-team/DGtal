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
 */
/**
 * @file exampleGenericLatticeConvexHull3D.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2025/09/07
 *
 * @ingroup Examples
 * This file is part of the DGtal library.
 */
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/tools/GenericLatticeConvexHull.h"


using namespace DGtal;


//Polyscope global
polyscope::PointCloud   *psPoints;
polyscope::PointCloud   *psVertices;
polyscope::PointCloud   *psBoundary0;
polyscope::CurveNetwork *psBoundary1;
polyscope::SurfaceMesh  *psBoundary2;

std::random_device rd;
std::mt19937 g(rd());

template < typename Point >
static
std::vector< Point >
makeRandomLatticePointsFromDirVectors( Point A, const std::vector< Point>& V,
                                       int nb, double radius, int amplitude, int aff_dim )
{
  std::uniform_int_distribution<int> U(-amplitude, amplitude);
  std::vector< Point > P;
  int m = std::min( aff_dim, (int) V.size() );
  for ( auto k = 0; P.size() < nb && k < 100000; k++ )
    {
      Point B = A;
      for ( auto i = 0; i < m; i++ )
        {
          int l = U( g );
          B += l * V[ i ];
        }
      if ( (B-A).norm() <= radius )
        P.push_back( B );
    }
  std::shuffle( P.begin(), P.end(), g );
  return P;
}

int main( int argc, char* argv[] )
{
  typedef GenericLatticeConvexHull< 3, int > QHull;
  typedef SpaceND< 3, int >                  Space;
  typedef Space::Point                       Point;

  std::cout << "Usage: " << argv[ 0 ] << " [R=30] [N=30] [D=2]\n";
  std::cout << "Computes the convex hull of N points within a ball of radius R, these points belonging to a lattice of chosen dimension D.\n";
  double radius = argc > 1 ? atof( argv[ 1 ] ) : 30.0;
  int    nb     = argc > 2 ? atoi( argv[ 2 ] ) : 30;
  int    adim   = argc > 3 ? atoi( argv[ 3 ] ) : 2;
  if ( nb < 0 ) return 1;
  if ( adim < 0 || adim > 3 ) return 1;

  // Create points
  std::vector< Point > L = { Point{ 4, 1, -3 }, Point{ 0, 2, 5 }, Point{ -1, -3, 5 } };
  std::vector< Point > X
    = makeRandomLatticePointsFromDirVectors( Point(1,2,-1),
                                             L, nb,
                                             radius,
                                             int( round( radius+0.5 ) ),
                                             adim );

  // Compute convex hull
  QHull hull;
  bool ok = hull.compute( X, false );
  std:: cout << ( ok ? "[PASSED]" : "[FAILED]" ) << " hull=" << hull << "\n";
  // Initialize polyscope
  polyscope::init();
  psPoints   = polyscope::registerPointCloud( "Points",   X );
  psVertices = polyscope::registerPointCloud( "Vertices", hull.positions );
  if ( hull.affine_dimension <= 1 ) // 1 or 2 points
    {
      // no facets
      psBoundary0 = polyscope::registerPointCloud( "Convex hull bdy dim=0",
                                                   hull.positions );
    }
  else if ( hull.affine_dimension == 2 ) // 2D
    {
      // facets are edges (and implicitly converted by polyscope)
      psBoundary1 = polyscope::registerCurveNetwork( "Convex hull bdy dim=1",
                                                     hull.positions, hull.facets );
    }
  else if ( hull.affine_dimension == 3 ) // 3D
    {
      // facets are polygons
      psBoundary2 = polyscope::registerSurfaceMesh("Convex hull bdy dim=2",
                                                   hull.positions, hull.facets );
    }
  
  polyscope::show();
  return EXIT_SUCCESS;
  
}
