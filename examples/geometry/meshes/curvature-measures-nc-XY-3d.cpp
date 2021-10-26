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
 * @file geometry/meshes/curvature-measures-nc-XY-3d.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/10/25
 *
 * An example file named curvature-measures-nc-XY-3d.
 *
 * This file is part of the DGtal library.
 */

/**
   Computation of principal curvatures and directions on a torus mesh,
   using Normal cycle curvature measures (based on the theory of
   Normal cycle).

\verbatim
# a 20x20 discretized torus with a radius 0.5 for measures
./examples/geometry/meshes/curvature-measures-nc-XY-3d 20 20 0.5
\endverbatim
outputs
\verbatim
\endverbatim


It also produces several OBJ files to display curvature estimation
results, `example-nc-K1.obj`, `example-nc-D1.obj`,
`example-nc-K2.obj`, and `example-nc-D2.obj` as well as the associated
MTL file.

<table>
<tr><td>
\image html torus-nc-K1-D1-True-r0_5.jpg "Normal cycle smallest principal curvature and direction, r=0.5" width=90%
</td><td>
\image html torus-nc-K2-D2-True-r0_5.jpg "Normal cycle greatest principal curvature and direction, r=0.5" width=90%
</td></tr>
</table>

@see \ref moduleCurvatureMeasures


\example geometry/meshes/curvature-measures-nc-XY-3d.cpp
*/

#include <iostream>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/math/linalg/EigenDecomposition.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/shapes/SurfaceMeshHelper.h"
//! [curvature-measures-Includes]
#include "DGtal/geometry/meshes/NormalCycleComputer.h"
//! [curvature-measures-Includes]
#include "DGtal/io/writers/SurfaceMeshWriter.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/helpers/Shortcuts.h"

DGtal::GradientColorMap< double >
makeColorMap( double min_value, double max_value )
{
  DGtal::GradientColorMap< double > gradcmap( min_value, max_value );
  gradcmap.addColor( DGtal::Color( 0, 0, 255 ) );
  gradcmap.addColor( DGtal::Color( 0, 255, 255 ) );
  gradcmap.addColor( DGtal::Color( 255, 255, 255 ) );
  gradcmap.addColor( DGtal::Color( 255, 255, 0 ) );
  gradcmap.addColor( DGtal::Color( 255, 0, 0 ) );
  return gradcmap;
}

int main( int argc, char* argv[] )
{
  //! [curvature-measures-Typedefs]
  using namespace DGtal;
  using namespace DGtal::Z3i;
  typedef SurfaceMesh< RealPoint, RealVector >          SM;
  typedef NormalCycleComputer< RealPoint, RealVector >  NC;
  typedef SurfaceMeshHelper< RealPoint, RealVector >    SMH;
  //! [curvature-measures-Typedefs]
  int    m = argc > 1 ? atoi( argv[ 1 ] ) : 20;  // nb latitude points
  int    n = argc > 2 ? atoi( argv[ 2 ] ) : 20;  // nb longitude points
  double R = argc > 3 ? atof( argv[ 3 ] ) : 0.5; // radius of measuring ball

  //! [curvature-measures-SurfaceMesh]
  const double big_radius   = 3.0;
  const double small_radius = 1.0;
  SM torus = SMH::makeTorus( big_radius, small_radius, RealPoint { 0.0, 0.0, 0.0 },
                             m, n, 0,
                             SMH::NormalsType::NO_NORMALS );
  //! [curvature-measures-SurfaceMesh]

  //! [curvature-measures-NC]
  // builds a NormalCycleComputer object onto the torus mesh
  NC nc( torus );
  // computes area, anisotropic XY curvature measures
  auto mu0  = nc.computeMu0();
  auto muXY = nc.computeMuXY();
  //! [curvature-measures-NC]

  //! [curvature-measures-estimations]
  // estimates mean (H) and Gaussian (G) curvatures by measure normalization.
  std::vector< double > K1( torus.nbFaces() );
  std::vector< double > K2( torus.nbFaces() );
  std::vector< RealVector > D1( torus.nbFaces() );
  std::vector< RealVector > D2( torus.nbFaces() );
  torus.computeFaceNormalsFromPositions();
  for ( auto f = 0; f < torus.nbFaces(); ++f )
    {
      const auto b    = torus.faceCentroid( f );
      const auto N    = torus.faceNormals()[ f ];
      const auto area = mu0 .measure( b, R, f );
      auto M          = muXY.measure( b, R, f );
      std::cout << f << " " << b << " " << N << " " << area << " " << M << std::endl;
      M += M.transpose();
      M *= 0.5;
      const double   coef_N = 1000.0 * area;
      // Trick to force orthogonality to normal vector.
      for ( int j = 0; j < 3; j++ )
        for ( int k = 0; k < 3; k++ )
          M( j, k ) += coef_N * N[ j ] * N[ k ];
      auto V = M;
      std::cout << M << std::endl;
      RealVector L;
      EigenDecomposition< 3, double>::getEigenDecomposition( M, V, L );
      std::cout << M.rows() << " " << M.cols() << " "
                << V.rows() << " " << V.cols() << " "
                << L.size() << std::endl;
      std::cout << -L[ 1 ] / area << " " << V.column( 1 ) << std::endl
                << -L[ 0 ] / area << " " << V.column( 0 ) << std::endl;
                
      D1[ f ] = V.column( 1 );
      D2[ f ] = V.column( 0 );
      K1[ f ] = ( area != 0.0 ) ? -L[ 1 ] / area : 0.0;
      K2[ f ] = ( area != 0.0 ) ? -L[ 0 ] / area : 0.0;
    }
  //! [curvature-measures-estimations]

  //! [curvature-measures-check]
  auto K1_min_max = std::minmax_element( K1.cbegin(), K1.cend() );
  auto K2_min_max = std::minmax_element( K2.cbegin(), K2.cend() );
  std::cout << "Expected k1 curvatures:"
            << " min=" << ( 1.0 / ( small_radius - big_radius ) )
            << " max=" << ( 1.0 / ( big_radius + small_radius ) )
            << std::endl;
  std::cout << "Computed k1 curvatures:"
            << " min=" << *K1_min_max.first
            << " max=" << *K1_min_max.second
            << std::endl;
  std::cout << "Expected k2 curvatures:"
            << " min=" << ( 1.0 / small_radius )
            << " max=" << ( 1.0 / small_radius )
            << std::endl;
  std::cout << "Computed k2 curvatures:"
            << " min=" << *K2_min_max.first
            << " max=" << *K2_min_max.second
            << std::endl;
  //! [curvature-measures-check]

  //! [curvature-measures-output]
  typedef SurfaceMeshWriter< RealPoint, RealVector > SMW;
  typedef Shortcuts< KSpace > SH;
  const auto colormapK1 = makeColorMap( -1.0 / small_radius, 1.0 / small_radius );
  const auto colormapK2 = makeColorMap( -1.0 / small_radius, 1.0 / small_radius );
  auto colorsK1 = SMW::Colors( torus.nbFaces() );
  auto colorsK2 = SMW::Colors( torus.nbFaces() );
  for ( auto i = 0; i < torus.nbFaces(); i++ )
    {
      colorsK1[ i ] = colormapK1( K1[ i ] );
      colorsK2[ i ] = colormapK2( K2[ i ] );
    }
  SMW::writeOBJ( "example-nc-K1", torus, colorsK1 );
  SMW::writeOBJ( "example-nc-K2", torus, colorsK2 );
  const auto avg_e = torus.averageEdgeLength();
  SH::RealPoints positions( torus.nbFaces() );
  for ( auto f = 0; f < positions.size(); ++f )
    {
      D1[ f ] *= torus.localWindow( f );
      positions[ f ] = torus.faceCentroid( f ) - 0.5 * D1[ f ];
    }
  SH::saveVectorFieldOBJ( positions, D1, 0.05 * avg_e, SH::Colors(),
                          "example-nc-D1",
                          SH::Color::Black, SH::Color( 0, 128, 0 ) );
  for ( auto f = 0; f < positions.size(); ++f )
    {
      D2[ f ] *= torus.localWindow( f );
      positions[ f ] = torus.faceCentroid( f ) - 0.5 * D2[ f ];
    }
  SH::saveVectorFieldOBJ( positions, D2, 0.05 * avg_e, SH::Colors(),
                          "example-nc-D2",
                          SH::Color::Black, SH::Color(128, 0,128 ) );
  
  //! [curvature-measures-output]
  return 0;
}
