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
 * @file geometry/meshes/vol-curvature-measures-icnc-XY-3d.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/10/25
 *
 * An example file named vol-curvature-measures-icnc-XY-3d.
 *
 * This file is part of the DGtal library.
 */

/**
   Computation of principal curvatures and directions on a mesh
   defined by a VOL digital file, using interpolated corrected
   curvature measures (based on the theory of corrected normal
   currents). It uses a digital normal vector estimator to improve
   curvature estimations.

\verbatim
# "Al" vol file
./examples/geometry/meshes/vol-curvature-measures-icnc-XY-3d ../examples/samples/Al.100.vol 2.0 0 1 0.33
\endverbatim

outputs
\verbatim
- Domain size is 100 x 100 x 100
- digital shape has 70413 voxels.
- surface has 21239 surfels.
[SurfaceMesh (OK) #V=21278 #VN=0 #E=42522 #F=21239 #FN=0 E[IF]=4.00151 E[IV]=3.9968 E[IFE]=1.99793]
- CTrivial normal t-ring=3 (discrete)
Computed k1 curvatures: min=-0.591355 max=0.332964
Computed k2 curvatures: min=-0.0853815 max=0.559684
\endverbatim

It also produces several OBJ files to display curvature estimation
results, `example-cnc-K1.obj`, `example-cnc-D1.obj`,
`example-cnc-K2.obj`, and `example-cnc-D2.obj` as well as the associated
MTL file.

<table>
<tr><td>
\image html al-cnc-K1-D1-r2.jpg "Interpolated corrected smallest principal curvature and direction, r=2" width=90%
</td><td>
\image html al-cnc-K2-D2-r2.jpg "Interpolated corrected greatest principal curvature and direction, r=2" width=90%
</td></tr>
</table>

@see \ref moduleCurvatureMeasures

\note In opposition with Normal Cycle curvature measures,
(interpolated) corrected curvature measures can take into account an
external normal vector field to estimate curvatures with better
accuracy.

\example geometry/meshes/vol-curvature-measures-icnc-XY-3d.cpp
*/

#include <iostream>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/math/linalg/EigenDecomposition.h"
#include "DGtal/shapes/SurfaceMesh.h"
//! [curvature-measures-Includes]
#include "DGtal/geometry/meshes/CorrectedNormalCurrentComputer.h"
//! [curvature-measures-Includes]
#include "DGtal/io/writers/SurfaceMeshWriter.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"
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
            << "\t" << argv[ 0 ] << " <filename.vol> <R> <m> <M> <Kmax>" << std::endl
            << std::endl
            << "Computation of principal curvatures and directions on a vol file, " << std::endl
            << "using interpolated corrected curvature measures (based " << std::endl
            << "on the theory of corrected normal currents)."            << std::endl
            << "- builds the surface mesh from file <filename.obj>"      << std::endl
            << "- <R> is the radius of the measuring balls."             << std::endl
            << "- <m> is the min threshold value for the vol file"       << std::endl
            << "- <M> is the max threshold value for the vol file"       << std::endl
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
  typedef Shortcuts< KSpace >          SH;
  typedef ShortcutsGeometry< KSpace > SHG;
  //! [curvature-measures-Typedefs]
  // VOL file
  std::string input = argv[ 1 ];
  const double    R = argc > 2 ? atof( argv[ 2 ] ) : 2.0; // radius of measuring ball
  const int       m = argc > 3 ? atoi( argv[ 3 ] ) : 0; // min threshold
  const int       M = argc > 4 ? atoi( argv[ 4 ] ) : 1; // max threshold
  const double Kmax = argc > 5 ? atof( argv[ 5 ] ) : 0.33; // range mean curvature colormap

  //! [curvature-measures-DigitalSurface]
  // Read VOL file and build digital surface
  auto params = SH::defaultParameters() | SHG::defaultParameters();
  params( "thresholdMin", m )( "thresholdMax", M )( "closed", 1);
  params( "t-ring", 3 )( "surfaceTraversal", "Default" );
  auto bimage = SH::makeBinaryImage( input.c_str(), params );
  if ( bimage == nullptr ) 
    {
      trace.error() <<  "Unable to read file <" << input.c_str() << ">" << std::endl;
      return 1;
    }
  auto K      = SH::getKSpace( bimage, params );
  auto sembedder   = SH::getSCellEmbedder( K );
  auto embedder    = SH::getCellEmbedder( K );
  auto surface     = SH::makeDigitalSurface( bimage, K, params );
  auto surfels     = SH::getSurfelRange( surface, params );
  trace.info() << "- surface has " << surfels.size()<< " surfels." << std::endl;
  //! [curvature-measures-DigitalSurface]

  //! [curvature-measures-SurfaceMesh]
  SM smesh;
  std::vector< SM::Vertices > faces;
  SH::Cell2Index c2i;
  auto pointels = SH::getPointelRange( c2i, surface );
  auto vertices = SH::RealPoints( pointels.size() );
  std::transform( pointels.cbegin(), pointels.cend(), vertices.begin(),
                  [&] (const SH::Cell& c) { return embedder( c ); } ); 
  for ( auto&& surfel : *surface )
    {
      const auto primal_surfel_vtcs = SH::getPointelRange( K, surfel );
      SM::Vertices face;	      
      for ( auto&& primal_vtx : primal_surfel_vtcs )
        face.push_back( c2i[ primal_vtx ] );
      faces.push_back( face );
    }
  smesh.init( vertices.cbegin(), vertices.cend(),
              faces.cbegin(),    faces.cend() );
  trace.info() << smesh << std::endl;
  //! [curvature-measures-SurfaceMesh]

  //! [curvature-measures-CNC]
  // Builds a CorrectedNormalCurrentComputer object onto the SurfaceMesh object
  CNC cnc( smesh );
  // Estimates normal vectors using Convolved Trivial Normal estimator 
  auto face_normals = SHG::getCTrivialNormalVectors( surface, surfels, params );
  smesh.setFaceNormals( face_normals.cbegin(), face_normals.cend() );
  if ( smesh.vertexNormals().empty() )
    smesh.computeVertexNormalsFromFaceNormals();
  // computes area, anisotropic XY curvature measures
  auto mu0  = cnc.computeMu0();
  auto muXY = cnc.computeMuXY();
  //! [curvature-measures-CNC]

  //! [curvature-measures-estimations]
  // estimates principal curvatures (K1,K2) and directions (D1,D2) by
  // measure normalization.
  std::vector< double > K1( smesh.nbFaces() );
  std::vector< double > K2( smesh.nbFaces() );
  std::vector< RealVector > D1( smesh.nbFaces() );
  std::vector< RealVector > D2( smesh.nbFaces() );
  for ( auto f = 0; f < smesh.nbFaces(); ++f )
    {
      const auto b    = smesh.faceCentroid( f );
      const auto N    = smesh.faceNormals()[ f ];
      const auto area = mu0 .measure( b, R, f );
      const auto M2   = muXY.measure( b, R, f );
      std::tie( K1[ f ], K2[ f ], D1[ f ], D2[ f ] )
        = cnc.principalCurvatures( area, M2, N );
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
  // Remove normals for better blocky display.
  smesh.vertexNormals() = SH::RealVectors();
  smesh.faceNormals()   = SH::RealVectors();
  typedef SurfaceMeshWriter< RealPoint, RealVector > SMW;
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
