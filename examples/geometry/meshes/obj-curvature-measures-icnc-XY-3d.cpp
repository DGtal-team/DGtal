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
 * @file geometry/meshes/obj-curvature-measures-icnc-XY-3d.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/10/25
 *
 * An example file named obj-curvature-measures-icnc-XY-3d.
 *
 * This file is part of the DGtal library.
 */

/**
   Computation of principal curvatures and directions on a mesh defined by
   an OBJ file, using interpolated corrected curvature measures (based on the
   theory of corrected normal currents).

\verbatim
# "spot" OBJ file
./examples/geometry/meshes/obj-curvature-measures-icnc-XY-3d ../examples/samples/spot.obj 0.05 15
\endverbatim
outputs
\verbatim
[SurfaceMeshReader::readOBJ] Read #lines=12011 #V=2930 #VN=0 #F=5856
Mesh=[SurfaceMesh (OK) #V=2930 #VN=0 #E=8784 #F=5856 #FN=0 E[IF]=3 E[IV]=5.9959 E[IFE]=2] diameter=2.58809 radius=0.05
Computed k1 curvatures: min=-15.0745 max=14.493
Computed k2 curvatures: min=-0.0726101 max=18.3485
\endverbatim

It also produces several OBJ files to display curvature estimation
results, `example-cnc-K1.obj`, `example-cnc-D1.obj`,
`example-cnc-K2.obj`, and `example-cnc-D2.obj` as well as the associated
MTL file.

<table>
<tr><td>
\image html spot-cnc-K1-D1-r0_05.jpg "Interpolated corrected smallest principal curvature and direction, r=0.05" width=90%
</td><td>
\image html spot-cnc-K2-D2-r0_05.jpg "Interpolated corrected greatest principal curvature and direction, r=0.05" width=90%
</td></tr>
</table>

@see \ref moduleCurvatureMeasures


\example geometry/meshes/obj-curvature-measures-icnc-XY-3d.cpp
*/

#include <iostream>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/shapes/SurfaceMesh.h"
//! [curvature-measures-Includes]
#include "DGtal/geometry/meshes/CorrectedNormalCurrentComputer.h"
//! [curvature-measures-Includes]
#include "DGtal/io/writers/SurfaceMeshWriter.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/io/readers/SurfaceMeshReader.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/colormaps/QuantifiedColorMap.h"

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
            << "\t" << argv[ 0 ] << " <filename.obj> <R> <Kmax>" << std::endl
            << std::endl
            << "Computation of principal curvatures and directions on a mesh, " << std::endl
            << "using interpolated corrected curvature measures (based " << std::endl
            << "on the theory of corrected normal currents)."            << std::endl
            << "- builds the surface mesh from file <filename.obj>"      << std::endl
            << "- <R> is the radius of the measuring balls."             << std::endl
            << "- <Kmax> gives the colormap range [-Kmax,Kmax] for"      << std::endl
            << "  the output of principal curvatures estimates"          << std::endl
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
  typedef SurfaceMeshReader< RealPoint, RealVector >              SMR;
  //! [curvature-measures-Typedefs]
  // OBJ file
  std::string input = argv[ 1 ];
  const double    R = argc > 2 ? atof( argv[ 2 ] ) : 0.0; // radius of measuring ball
  const double Kmax = argc > 3 ? atof( argv[ 3 ] ) : 5.0; // range curvature colormap

  //! [curvature-measures-SurfaceMesh]
  SM smesh;
  std::ifstream obj_stream( input.c_str() );
  bool ok = SMR::readOBJ( obj_stream, smesh );
  if ( !ok )
    {
      trace.error() <<  "Unable to read file <" << input.c_str() << ">" << std::endl;
      return 1;
    }
  RealPoint lo = smesh.position( 0 );
  RealPoint up = smesh.position( 0 );
  for ( const auto& p : smesh.positions() )
    lo = lo.inf( p ), up = up.sup( p );
  const auto diameter = (up - lo).norm();
  trace.info() << "Mesh=" << smesh
               << " diameter=" << diameter
               << " radius=" << R << std::endl;
  //! [curvature-measures-SurfaceMesh]

  //! [curvature-measures-CNC]
  // builds a CorrectedNormalCurrentComputer object onto the mesh
  CNC cnc( smesh );
  // computes normals if necessary
  if ( smesh.vertexNormals().empty() )
    {
      if ( smesh.faceNormals().empty() )
        smesh.computeFaceNormalsFromPositions();
      smesh.computeVertexNormalsFromFaceNormals();
    }
  // computes area, anisotropic XY curvature measures
  auto mu0  = cnc.computeMu0();
  auto muXY = cnc.computeMuXY();
  //! [curvature-measures-CNC]

  //! [curvature-measures-estimations]
  // estimates mean (H) and Gaussian (G) curvatures by measure normalization.
  std::vector< double > K1( smesh.nbFaces() );
  std::vector< double > K2( smesh.nbFaces() );
  std::vector< RealVector > D1( smesh.nbFaces() );
  std::vector< RealVector > D2( smesh.nbFaces() );
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
  std::cout << "Computed k1 curvatures:"
            << " min=" << *K1_min_max.first << " max=" << *K1_min_max.second
            << std::endl;
  std::cout << "Computed k2 curvatures:"
            << " min=" << *K2_min_max.first << " max=" << *K2_min_max.second
            << std::endl;
  //! [curvature-measures-check]

  //! [curvature-measures-output]
  typedef SurfaceMeshWriter< RealPoint, RealVector > SMW;
  typedef Shortcuts< KSpace > SH;
  const auto colormapK1 = makeQuantifiedColorMap( makeColorMap( -Kmax, Kmax ) );
  const auto colormapK2 = makeQuantifiedColorMap( makeColorMap( -Kmax, Kmax ) );
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
