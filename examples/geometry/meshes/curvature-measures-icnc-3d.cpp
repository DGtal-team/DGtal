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
 * @file geometry/meshes/curvature-measures-icnc-3d.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/10/25
 *
 * An example file named curvature-measures-icnc-3d.
 *
 * This file is part of the DGtal library.
 */

/**
   Computation of curvatures on a torus mesh, using interpolated
   corrected curvature measures (based on the theory of corrected
   normal currents).

\verbatim
./examples/geometry/meshes/curvature-measures-icnc-3d 
\endverbatim
outputs
\verbatim
Expected mean curvatures: min=0.25 max=0.625
Computed mean curvatures: min=0.264763 max=0.622318
Expected Gaussian curvatures: min=-0.5 max=0.25
Computed Gaussian curvatures: min=-0.470473 max=0.244636
\endverbatim

It also produces several OBJ files to display curvature estimation
results, `example-cnc-H.obj` and `example-cnc-H.obj` as well as the
associated MTL file.

<table>
<tr><td>
\image html torus-cnc-H-True-r0.jpg "Interpolated corrected mean curvature measure, r=0" width=90%
</td><td>
\image html torus-cnc-G-True-r0.jpg "Interpolated corrected Gaussian curvature measure, r=0" width=90%
</td></tr>
<tr><td>
\image html torus-cnc-H-True-r0_5.jpg "Interpolated corrected mean curvature measure, r=0.5" width=90%
</td><td>
\image html torus-cnc-G-True-r0_5.jpg "Interpolated corrected Gaussian curvature measure, r=0.5" width=90%
</td></tr>
</table>

@see \ref moduleCurvatureMeasures


\example geometry/meshes/curvature-measures-icnc-3d.cpp
*/

#include <iostream>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/shapes/SurfaceMeshHelper.h"
//! [curvature-measures-Includes]
#include "DGtal/geometry/meshes/CorrectedNormalCurrentComputer.h"
//! [curvature-measures-Includes]
#include "DGtal/io/writers/SurfaceMeshWriter.h"
#include "DGtal/io/colormaps/GradientColorMap.h"

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
  typedef SurfaceMesh< RealPoint, RealVector >                    SM;
  typedef CorrectedNormalCurrentComputer< RealPoint, RealVector > CNC;
  typedef SurfaceMeshHelper< RealPoint, RealVector >              SMH;
  //! [curvature-measures-Typedefs]
  int    m = argc > 1 ? atoi( argv[ 1 ] ) : 20;  // nb latitude points
  int    n = argc > 2 ? atoi( argv[ 2 ] ) : 20;  // nb longitude points
  double R = argc > 3 ? atof( argv[ 3 ] ) : 0.5; // radius of measuring ball

  //! [curvature-measures-SurfaceMesh]
  const double big_radius   = 3.0;
  const double small_radius = 1.0;
  SM torus = SMH::makeTorus( big_radius, small_radius, RealPoint { 0.0, 0.0, 0.0 },
                             m, n, 0,
                             SMH::NormalsType::VERTEX_NORMALS );
  //! [curvature-measures-SurfaceMesh]

  //! [curvature-measures-CNC]
  // builds a CorrectedNormalCurrentComputer object onto the torus mesh
  CNC cnc( torus );
  // computes area, mean and Gaussian curvature measures
  auto mu0 = cnc.computeMu0();
  auto mu1 = cnc.computeMu1();
  auto mu2 = cnc.computeMu2();
  //! [curvature-measures-CNC]

  //! [curvature-measures-estimations]
  // estimates mean (H) and Gaussian (G) curvatures by measure normalization.
  std::vector< double > H( torus.nbFaces() );
  std::vector< double > G( torus.nbFaces() );
  for ( auto f = 0; f < torus.nbFaces(); ++f )
    {
      const auto b    = torus.faceCentroid( f );
      const auto area = mu0.measure( b, R, f );
      H[ f ] = mu1.measure( b, R, f ) / ( 2.0 * area );
      G[ f ] = mu2.measure( b, R, f ) / area;
    }
  //! [curvature-measures-estimations]

  //! [curvature-measures-check]
  auto H_min_max = std::minmax_element( H.cbegin(), H.cend() );
  auto G_min_max = std::minmax_element( G.cbegin(), G.cend() );
  std::cout << "Expected mean curvatures:"
            << " min=" << ( 0.5 / ( small_radius - big_radius ) + 0.5 / small_radius )
            << " max=" << ( 0.5 / ( big_radius + small_radius ) + 0.5 / small_radius )
            << std::endl;
  std::cout << "Computed mean curvatures:"
            << " min=" << *H_min_max.first
            << " max=" << *H_min_max.second
            << std::endl;
  std::cout << "Expected Gaussian curvatures:"
            << " min=" << ( 1.0 / ( small_radius - big_radius ) * 1.0 / small_radius )
            << " max=" << ( 1.0 / ( big_radius + small_radius ) * 1.0 / small_radius )
            << std::endl;
  std::cout << "Computed Gaussian curvatures:"
            << " min=" << *G_min_max.first
            << " max=" << *G_min_max.second
            << std::endl;
  //! [curvature-measures-check]

  //! [curvature-measures-output]
  typedef SurfaceMeshWriter< RealPoint, RealVector > SMW;
  const auto colormapH = makeColorMap( -0.625, 0.625 );
  const auto colormapG = makeColorMap( -0.625, 0.625 );
  auto colorsH = SMW::Colors( torus.nbFaces() );
  auto colorsG = SMW::Colors( torus.nbFaces() );
  for ( auto i = 0; i < torus.nbFaces(); i++ )
    {
      colorsH[ i ] = colormapH( H[ i ] );
      colorsG[ i ] = colormapG( G[ i ] );
    }
  SMW::writeOBJ( "example-cnc-H", torus, colorsH );
  SMW::writeOBJ( "example-cnc-G", torus, colorsG );
  //! [curvature-measures-output]
  return 0;
}
