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
 * @file geometry/meshes/curvature-measures-icnc-XY-3d.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/10/25
 *
 * An example file named curvature-measures-icnc-XY-3d.
 *
 * This file is part of the DGtal library.
 */

/**
   Computation of principal curvatures and directions on a torus mesh,
   using interpolated corrected curvature measures (based on the
   theory of corrected normal currents).

\verbatim
# a 20x20 discretized torus with a radius for measures of 0
./examples/geometry/meshes/curvature-measures-icnc-XY-3d torus 20 20 0
\endverbatim
outputs
\verbatim
Expected k1 curvatures: min=-0.5 max=0.25
Computed k1 curvatures: min=-0.500225 max=0.249888
Expected k2 curvatures: min=1 max=1
Computed k2 curvatures: min=1.00011 max=1.00678
\endverbatim
We may increase the radius:
\verbatim
# a 20x20 discretized torus with a radius for measures of 0.5
./examples/geometry/meshes/curvature-measures-icnc-XY-3d torus 20 20 0.5
\endverbatim
outputs
\verbatim
Expected k1 curvatures: min=-0.5 max=0.25
Computed k1 curvatures: min=-0.454026 max=0.242436
Expected k2 curvatures: min=1 max=1
Computed k2 curvatures: min=0.924283 max=0.95338
\endverbatim

It also produces several OBJ files to display curvature estimation
results, `example-cnc-K1.obj`, `example-cnc-D1.obj`,
`example-cnc-K2.obj`, and `example-cnc-D2.obj` as well as the associated
MTL file.

<table>
<tr><td>
\image html torus-cnc-K1-D1-True-r0.jpg "Interpolated corrected smallest principal curvature and direction, r=0" width=90%
</td><td>
\image html torus-cnc-K2-D2-True-r0.jpg "Interpolated corrected greatest principal curvature and direction, r=0" width=90%
</td></tr>
<tr><td>
\image html torus-cnc-K1-D1-True-r0_5.jpg "Interpolated corrected smallest principal curvature and direction, r=0.5" width=90%
</td><td>
\image html torus-cnc-K2-D2-True-r0_5.jpg "Interpolated corrected greatest principal curvature and direction, r=0.5" width=90%
</td></tr>
</table>

@see \ref moduleCurvatureMeasures

@note Interpolated corected curvature measures can provide consistent results even on on bad sampling of smooth surfaces. A well known example is the Schwarz lantern. You may try the following:

\verbatim
./examples/geometry/meshes/curvature-measures-icnc-XY-3d lantern 20 20 0.5
\endverbatim

outputs

\verbatim
Expected k1 curvatures: min=0 max=0
Computed k1 curvatures: min=-0.00258327 max=0.006962
Expected k2 curvatures: min=0.5 max=0.5
Computed k2 curvatures: min=0.48689 max=0.487788
\endverbatim

\example geometry/meshes/curvature-measures-icnc-XY-3d.cpp
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
#include "DGtal/io/colormaps/QuantifiedColorMap.h"
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

void usage( int argc, char* argv[] )
{
  ((void) argc); 
  std::cout << "Usage: " << std::endl
            << "\t" << argv[ 0 ] << " <shape> <m> <n> <R>" << std::endl
            << std::endl
            << "Computation of principal curvatures and directions on a shape, " << std::endl
            << "using interpolated corrected curvature measures (based " << std::endl
            << "on the theory of corrected normal currents)."            << std::endl
            << "- builds a <shape> in {torus,lantern,sphere}, with     " << std::endl
            << "  <m> latitude points and <n> longitude points."         << std::endl
            << "- <R> is the radius of the measuring balls."             << std::endl
            << "It produces several OBJ files to display principal "     << std::endl
            << "curvatures and directions estimations: `example-cnc-K1.obj`" << std::endl
            << "`example-cnc-K2.obj`, `example-cnc-D1.obj`, and"         << std::endl
            << "`example-cnc-D2.obj` as well as associated MTL files."   << std::endl;
}

int main( int argc, char* argv[] )
{
  if ( argc <= 1 )
    {
      usage( argc, argv );
      return 0;
    }
  //! [curvature-measures-Typedefs]
  using namespace DGtal;
  using namespace DGtal::Z3i;
  typedef SurfaceMesh< RealPoint, RealVector >                    SM;
  typedef CorrectedNormalCurrentComputer< RealPoint, RealVector > CNC;
  typedef SurfaceMeshHelper< RealPoint, RealVector >              SMH;
  //! [curvature-measures-Typedefs]
  // a shape in "torus|lantern|sphere"
  std::string input = argv[ 1 ];
  int    m = argc > 2 ? atoi( argv[ 2 ] ) : 20;  // nb latitude points
  int    n = argc > 3 ? atoi( argv[ 3 ] ) : 20;  // nb longitude points
  double R = argc > 4 ? atof( argv[ 4 ] ) : 0.5; // radius of measuring ball

  //! [curvature-measures-SurfaceMesh]
  SM smesh;
  double exp_K1_min = 0.0;
  double exp_K1_max = 0.0;
  double exp_K2_min = 0.0;
  double exp_K2_max = 0.0;
  if ( input == "torus" )
    {
      const double big_radius   = 3.0;
      const double small_radius = 1.00001; // avoid codacy warnings
      smesh = SMH::makeTorus( big_radius, small_radius,
                              RealPoint { 0.0, 0.0, 0.0 }, m, n, 0,
                              SMH::NormalsType::VERTEX_NORMALS );
      exp_K1_min = ( 1.0 / ( small_radius - big_radius ) );
      exp_K1_max = ( 1.0 / ( big_radius + small_radius ) );
      exp_K2_min = 1.0 / small_radius;
      exp_K2_max = 1.0 / small_radius;
    }
  else if ( input == "sphere" )
    {
      const double radius = 2.0;
      smesh = SMH::makeSphere( radius, RealPoint { 0.0, 0.0, 0.0 }, m, n,
                               SMH::NormalsType::VERTEX_NORMALS );
      exp_K1_min = 1.0 / radius;
      exp_K1_max = 1.0 / radius;
      exp_K2_min = 1.0 / radius;
      exp_K2_max = 1.0 / radius;
    }
  else if ( input == "lantern" )
    {
      const double radius = 2.0;
      smesh = SMH::makeLantern( radius, 1.0, RealPoint { 0.0, 0.0, 0.0 }, m, n,
                                SMH::NormalsType::VERTEX_NORMALS );
      exp_K1_min = 0.0;
      exp_K1_max = 0.0;
      exp_K2_min = 1.0 / radius;
      exp_K2_max = 1.0 / radius;
    }
  //! [curvature-measures-SurfaceMesh]

  //! [curvature-measures-CNC]
  // builds a CorrectedNormalCurrentComputer object onto the mesh
  CNC cnc( smesh );
  // computes area, anisotropic XY curvature measures
  auto mu0  = cnc.computeMu0();
  auto muXY = cnc.computeMuXY();
  //! [curvature-measures-CNC]

  //! [curvature-measures-estimations]
  // Estimates principal curvatures (K1,K2) and directions (D1,D2) by
  // measure normalization and eigen decomposition.
  std::vector< double > K1( smesh.nbFaces() );
  std::vector< double > K2( smesh.nbFaces() );
  std::vector< RealVector > D1( smesh.nbFaces() );
  std::vector< RealVector > D2( smesh.nbFaces() );
  // Principal directions computation requires a local face normal
  smesh.computeFaceNormalsFromPositions();
  for ( auto f = 0; f < smesh.nbFaces(); ++f )
    {
      const auto b    = smesh.faceCentroid( f );
      const auto N    = smesh.faceNormals()[ f ];
      const auto area = mu0 .measure( b, R, f );
      const auto M    = muXY.measure( b, R, f );
      std::tie( K1[ f ], K2[ f ], D1[ f ], D2[ f ] )
        = cnc.principalCurvatures( area, M, N );
    }
  //! [curvature-measures-estimations]

  //! [curvature-measures-check]
  auto K1_min_max = std::minmax_element( K1.cbegin(), K1.cend() );
  auto K2_min_max = std::minmax_element( K2.cbegin(), K2.cend() );
  std::cout << "Expected k1 curvatures:"
            << " min=" << exp_K1_min << " max=" << exp_K1_max
            << std::endl;
  std::cout << "Computed k1 curvatures:"
            << " min=" << *K1_min_max.first << " max=" << *K1_min_max.second
            << std::endl;
  std::cout << "Expected k2 curvatures:"
            << " min=" << exp_K2_min << " max=" << exp_K2_max
            << std::endl;
  std::cout << "Computed k2 curvatures:"
            << " min=" << *K2_min_max.first << " max=" << *K2_min_max.second
            << std::endl;
  //! [curvature-measures-check]

  //! [curvature-measures-output]
  typedef SurfaceMeshWriter< RealPoint, RealVector > SMW;
  typedef Shortcuts< KSpace > SH;
  const auto colormapK1 = makeQuantifiedColorMap( makeColorMap( -0.625, 0.625 ) );
  const auto colormapK2 = makeQuantifiedColorMap( makeColorMap( -0.625, 0.625 ) );
  auto colorsK1 = SMW::Colors( smesh.nbFaces() );
  auto colorsK2 = SMW::Colors( smesh.nbFaces() );
  for ( auto i = 0; i < smesh.nbFaces(); i++ )
    {
      colorsK1[ i ] = colormapK1( K1[ i ] );
      colorsK2[ i ] = colormapK2( K2[ i ] );
    }
  SMW::writeOBJ( "example-cnc-K1", smesh, colorsK1 );
  SMW::writeOBJ( "example-cnc-K2", smesh, colorsK2 );
  const auto avg_e = smesh.averageEdgeLength();
  SH::RealPoints positions( smesh.nbFaces() );
  for ( auto f = 0; f < positions.size(); ++f )
    {
      D1[ f ] *= smesh.localWindow( f );
      positions[ f ] = smesh.faceCentroid( f ) - 0.5 * D1[ f ];
    }
  SH::saveVectorFieldOBJ( positions, D1, 0.05 * avg_e, SH::Colors(),
                          "example-cnc-D1",
                          SH::Color::Black, SH::Color( 0, 128, 0 ) );
  for ( auto f = 0; f < positions.size(); ++f )
    {
      D2[ f ] *= smesh.localWindow( f );
      positions[ f ] = smesh.faceCentroid( f ) - 0.5 * D2[ f ];
    }
  SH::saveVectorFieldOBJ( positions, D2, 0.05 * avg_e, SH::Colors(),
                          "example-cnc-D2",
                          SH::Color::Black, SH::Color(128, 0,128 ) );
  
  //! [curvature-measures-output]
  return 0;
}
