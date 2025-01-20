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
 * @file geometry/meshes/obj-curvature-measures-icnc-3d.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/10/25
 *
 * An example file named obj-curvature-measures-icnc-3d.
 *
 * This file is part of the DGtal library.
 */

/**
   Computation of curvatures on a mesh defined by an OBJ file, using
   interpolated corrected curvature measures (based on the theory of
   corrected normal currents).

\verbatim
./examples/geometry/meshes/obj-curvature-measures-icnc-3d ../examples/samples/spot.obj 0.05 10 100
\endverbatim
outputs
\verbatim
[SurfaceMeshReader::readOBJ] Read #lines=12011 #V=2930 #VN=0 #F=5856
Mesh=[SurfaceMesh (OK) #V=2930 #VN=0 #E=8784 #F=5856 #FN=0 E[IF]=3 E[IV]=5.9959 E[IFE]=2] diameter=2.58809 radius=0.05
Computed mean curvatures: min=-7.7107 max=19.5898
Computed Gaussian curvatures: min=-190.593 max=377.898
\endverbatim

It also produces several OBJ files to display curvature estimation
results, `example-cnc-H.obj` and `example-cnc-G.obj` as well as the
associated MTL file.

<table>
<tr><td>
\image html spot-cnc-H-r0_05.jpg "Interpolated corrected mean curvature measure, r=0.05" width=90%
</td><td>
\image html spot-cnc-G-r0_05.jpg "Interpolated corrected Gaussian curvature measure, r=0.05" width=90%
</td></tr>
</table>

@see \ref moduleCurvatureMeasures

\example geometry/meshes/obj-curvature-measures-icnc-3d.cpp
*/

#include <iostream>
#include <fstream>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/shapes/SurfaceMeshHelper.h"
//! [curvature-measures-Includes]
#include "DGtal/geometry/meshes/CorrectedNormalCurrentComputer.h"
//! [curvature-measures-Includes]
#include "DGtal/io/readers/SurfaceMeshReader.h"
#include "DGtal/io/writers/SurfaceMeshWriter.h"
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
            << "\t" << argv[ 0 ] << " <filename.obj> <R> <Hmax> <Gmax>" << std::endl
            << std::endl
            << "Computation of mean and Gaussian curvatures on a mesh, " << std::endl
            << "using interpolated corrected curvature measures (based " << std::endl
            << "on the theory of corrected normal currents)."            << std::endl
            << "- builds the surface mesh from file <filename.obj>"      << std::endl
            << "- <R> is the radius of the measuring balls"              << std::endl
            << "- <Hmax> gives the colormap range [-Hmax,Hmax] for"      << std::endl
            << "  the output of mean curvature estimates"                << std::endl
            << "- <Gmax> gives the colormap range [-Gmax,Gmax] for"      << std::endl
            << "  the output of mean curvature estimates"                << std::endl
            << "It produces several OBJ files to display mean and"       << std::endl
            << "Gaussian curvature estimation results: `example-cnc-H.obj`" << std::endl
            << "and `example-cnc-G.obj` as well as the associated MTL file." << std::endl;
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
  const double Hmax = argc > 3 ? atof( argv[ 3 ] ) : 5.0; // range mean curvature colormap
  const double Gmax = argc > 4 ? atof( argv[ 4 ] ) : 0.5*Hmax*Hmax; // range Gaussian curvature colormap
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
  // builds a CorrectedNormalCurrentComputer object onto the SurfaceMesh object
  CNC cnc( smesh );
  // computes normals if necessary
  if ( smesh.vertexNormals().empty() )
    {
      if ( smesh.faceNormals().empty() )
        smesh.computeFaceNormalsFromPositions();
      smesh.computeVertexNormalsFromFaceNormals();
    }
  // computes area, mean and Gaussian curvature measures
  auto mu0 = cnc.computeMu0();
  auto mu1 = cnc.computeMu1();
  auto mu2 = cnc.computeMu2();
  //! [curvature-measures-CNC]

  //! [curvature-measures-estimations]
  // estimates mean (H) and Gaussian (G) curvatures by measure normalization.
  std::vector< double > H( smesh.nbFaces() );
  std::vector< double > G( smesh.nbFaces() );
  for ( auto f = 0; f < smesh.nbFaces(); ++f )
    {
      const auto b    = smesh.faceCentroid( f );
      const auto area = mu0.measure( b, R, f );
      H[ f ] = cnc.meanCurvature    ( area, mu1.measure( b, R, f ) );
      G[ f ] = cnc.GaussianCurvature( area, mu2.measure( b, R, f ) );
    }
  //! [curvature-measures-estimations]

  //! [curvature-measures-check]
  auto H_min_max = std::minmax_element( H.cbegin(), H.cend() );
  auto G_min_max = std::minmax_element( G.cbegin(), G.cend() );
  std::cout << "Computed mean curvatures:"
            << " min=" << *H_min_max.first << " max=" << *H_min_max.second
            << std::endl;
  std::cout << "Computed Gaussian curvatures:"
            << " min=" << *G_min_max.first << " max=" << *G_min_max.second
            << std::endl;
  //! [curvature-measures-check]

  //! [curvature-measures-output]
  typedef SurfaceMeshWriter< RealPoint, RealVector > SMW;
  const auto colormapH = makeQuantifiedColorMap( makeColorMap( -Hmax, Hmax ) );
  const auto colormapG = makeQuantifiedColorMap( makeColorMap( -Gmax, Gmax ) );
  auto colorsH = SMW::Colors( smesh.nbFaces() );
  auto colorsG = SMW::Colors( smesh.nbFaces() );
  for ( auto i = 0; i < smesh.nbFaces(); i++ )
    {
      colorsH[ i ] = colormapH( H[ i ] );
      colorsG[ i ] = colormapG( G[ i ] );
    }
  SMW::writeOBJ( "example-cnc-H", smesh, colorsH );
  SMW::writeOBJ( "example-cnc-G", smesh, colorsG );
  //! [curvature-measures-output]
  return 0;
}
