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
 * @file geometry/meshes/digpoly-curvature-measures-cnc-XY-3d.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/10/25
 *
 * An example file named digpoly-curvature-measures-cnc-XY-3d.
 *
 * This file is part of the DGtal library.
 */

/**
   Computation of principal curvatures and directions on a mesh
   defined by a an implicit shape discretized as a digital surface,
   using constant or interpolated corrected curvature measures (based
   on the theory of corrected normal currents). It uses a digital
   normal vector estimator to improve curvature estimations.  Errors
   with respect to true expected curvatures are also computed.

This first example uses constant per face corrected normal vector
field to compute curvatures.

\verbatim
# "Al" vol file
./examples/geometry/meshes/digpoly-curvature-measures-cnc-XY-3d torus 10 0.5 1.0 Const
\endverbatim

outputs

\verbatim
Using face-*Constant* Corrected Normal Current
- surface has 2584 surfels.
[SurfaceMesh (OK) #V=2584 #VN=0 #E=5168 #F=2584 #FN=0 E[IF]=4 E[IV]=4 E[IFE]=2]
- CTrivial normal t-ring=3 (discrete)
Expected K1 curvatures: min=-0.25 max=0.125
Computed k1 curvatures: min=-0.29482 max=0.159283
Expected k2 curvatures: min=0.5 max=0.5
Computed k2 curvatures: min=0.408469 max=0.580381
|K1-K1_CNC|_oo = 0.0740888
|K1-K1_CNC|_2  = 0.0236997
|K2-K2_CNC|_oo = 0.0915306
|K2-K2_CNC|_2  = 0.0386669
\endverbatim

This second example uses vertex-interpolated corrected normal vector
field to compute curvatures.

\verbatim
./examples/geometry/meshes/digpoly-curvature-measures-cnc-XY-3d torus 10 0.5 1.0 Interp
\endverbatim

outputs

\verbatim
Using vertex-*Interpolated* Corrected Normal Current
- surface has 2584 surfels.
[SurfaceMesh (OK) #V=2584 #VN=0 #E=5168 #F=2584 #FN=0 E[IF]=4 E[IV]=4 E[IFE]=2]
- CTrivial normal t-ring=3 (discrete)
Expected K1 curvatures: min=-0.25 max=0.125
Computed k1 curvatures: min=-0.267532 max=0.142428
Expected k2 curvatures: min=0.5 max=0.5
Computed k2 curvatures: min=0.422805 max=0.534006
|K1-K1_CNC|_oo = 0.0524167
|K1-K1_CNC|_2  = 0.0188969
|K2-K2_CNC|_oo = 0.0771952
|K2-K2_CNC|_2  = 0.0379048
\endverbatim

It also produces several OBJ files to display curvature estimation
results, `example-cnc-K1.obj`, `example-cnc-D1.obj`,
`example-cnc-K2.obj`, and `example-cnc-D2.obj` as well as the associated
MTL file.

<table>
<tr><td>
\image html digtorus-h0_5-ccnc-K1-D1-CTrivial-r1.jpg "Face-constant corrected smallest principal curvature and direction, r=1" width=90%
</td><td>
\image html digtorus-h0_5-ccnc-K2-D2-CTrivial-r1.jpg "Face-constant corrected greatest principal curvature and direction, r=1" width=90%
</td><td>
\image html digtorus-h0_5-icnc-K1-D1-CTrivial-r1.jpg "Vertex-interpolated corrected smallest principal curvature and direction, r=1" width=90%
</td><td>
\image html digtorus-h0_5-icnc-K2-D2-CTrivial-r1.jpg "Vertex-interpolated corrected greatest principal curvature and direction, r=1" width=90%
</td></tr>
</table>

@see \ref moduleCurvatureMeasures

\note In opposition with Normal Cycle curvature measures, constant or
interpolated corrected curvature measures can take into account an
external normal vector field to estimate curvatures with better
accuracy.

\example geometry/meshes/digpoly-curvature-measures-cnc-XY-3d.cpp
*/

#include <iostream>
#include <fstream>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/shapes/SurfaceMesh.h"
//! [curvature-measures-Includes]
#include "DGtal/geometry/meshes/CorrectedNormalCurrentComputer.h"
//! [curvature-measures-Includes]
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"
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
  using namespace DGtal;
  using namespace DGtal::Z3i;
  typedef Shortcuts< KSpace >          SH;
  std::cout << "Usage: " << std::endl
            << "\t" << argv[ 0 ] << " <P> <B> <h> <R> <mode>" << std::endl
            << std::endl
            << "Computation of principal curvatures and directions on"   << std::endl
            << "a digitized implicit shape using constant or "           << std::endl
            << "interpolated corrected curvature measures (based "       << std::endl
            << "on the theory of corrected normal currents)."            << std::endl
            << "- builds the surface mesh from polynomial <P>"           << std::endl
            << "- <B> defines the digitization space size [-B,B]^3"      << std::endl
            << "- <h> is the gridstep digitization"                      << std::endl
            << "- <R> is the radius of the measuring balls"              << std::endl
            << "- <mode> is either Const for constant corrected normal"  << std::endl
            << "  vector field or Interp for interpolated corrected"     << std::endl
            << "  normal vector field."                                  << std::endl
            << "It produces several OBJ files to display principal "     << std::endl
            << "curvatures and directions estimations: `example-cnc-K1.obj`" << std::endl
            << "`example-cnc-K2.obj`, `example-cnc-D1.obj`, and"         << std::endl
            << "`example-cnc-D2.obj` as well as associated MTL files."   << std::endl;
  std::cout << "You may either write your own polynomial as 3*x^2*y-z^2*x*y+1" << std::endl
            <<"or use a predefined polynomial in the following list:" << std::endl;
  auto L = SH::getPolynomialList();
  for ( const auto& p : L )
    std::cout << p.first << " : " << p.second << std::endl;
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
  std::string  poly = argv[ 1 ]; // polynomial
  const double    B = argc > 2 ? atof( argv[ 2 ] ) : 1.0; // max ||_oo bbox
  const double    h = argc > 3 ? atof( argv[ 3 ] ) : 1.0; // gridstep  
  const double    R = argc > 4 ? atof( argv[ 4 ] ) : 2.0; // radius of measuring ball
  std::string  mode = argc > 5 ? argv[ 5 ] : "Const"; // either Const or Interp
  bool interpolated = mode == "Interp";
  if ( interpolated )
    std::cout << "Using vertex-*Interpolated* Corrected Normal Current" << std::endl;
  else
    std::cout << "Using face-*Constant* Corrected Normal Current" << std::endl;
  //! [curvature-measures-DigitalSurface]
  // Read polynomial and build digital surface
  auto params = SH::defaultParameters() | SHG::defaultParameters();
  params( "t-ring", 3 )( "surfaceTraversal", "Default" );
  params( "polynomial", poly )( "gridstep", h ); 
  params( "minAABB", -B )( "maxAABB", B );
  params( "offset", 3.0 );
  auto shape       = SH::makeImplicitShape3D( params );
  auto K           = SH::getKSpace( params );
  auto dshape      = SH::makeDigitizedImplicitShape3D( shape, params );
  auto bimage      = SH::makeBinaryImage( dshape, params );
  if ( bimage == nullptr ) 
    {
      trace.error() <<  "Unable to read polynomial <"
                    << poly.c_str() << ">" << std::endl;
      return 1;
    }
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
                  [&] (const SH::Cell& c) { return h * embedder( c ); } ); 
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

  //! [curvature-measures-TrueCurvatures]
  auto exp_K1 = SHG::getFirstPrincipalCurvatures ( shape, K, surfels, params );
  auto exp_K2 = SHG::getSecondPrincipalCurvatures( shape, K, surfels, params );
  auto exp_D1 = SHG::getFirstPrincipalDirections ( shape, K, surfels, params );
  auto exp_D2 = SHG::getSecondPrincipalDirections( shape, K, surfels, params );
  //! [curvature-measures-TrueCurvatures]

  //! [curvature-measures-CNC]
  // Builds a CorrectedNormalCurrentComputer object onto the SurfaceMesh object
  CNC cnc( smesh );
  // Estimates normal vectors using Convolved Trivial Normal estimator 
  auto face_normals = SHG::getCTrivialNormalVectors( surface, surfels, params );
  // Set corrected face normals => Corrected Normal Current with
  // constant per face corrected vector field.
  smesh.setFaceNormals( face_normals.cbegin(), face_normals.cend() ); // CCNC
  // Set corrected vertex normals => Corrected Normal Current with
  // smooth linearly interpolated per face corrected vector field.
  if ( interpolated ) smesh.computeVertexNormalsFromFaceNormals();    // ICNC
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
      const auto M    = muXY.measure( b, R, f );
      std::tie( K1[ f ], K2[ f ], D1[ f ], D2[ f ] )
        = cnc.principalCurvatures( area, M, N );
    }
  //! [curvature-measures-estimations]

  //! [curvature-measures-check]
  auto exp_K1_min_max = std::minmax_element( exp_K1.cbegin(), exp_K1.cend() );
  auto exp_K2_min_max = std::minmax_element( exp_K2.cbegin(), exp_K2.cend() );
  auto K1_min_max = std::minmax_element( K1.cbegin(), K1.cend() );
  auto K2_min_max = std::minmax_element( K2.cbegin(), K2.cend() );
  std::cout << "Expected K1 curvatures:"
            << " min=" << *exp_K1_min_max.first << " max=" << *exp_K1_min_max.second
            << std::endl;
  std::cout << "Computed k1 curvatures:"
            << " min=" << *K1_min_max.first << " max=" << *K1_min_max.second
            << std::endl;
  std::cout << "Expected k2 curvatures:"
            << " min=" << *exp_K2_min_max.first << " max=" << *exp_K2_min_max.second
            << std::endl;
  std::cout << "Computed k2 curvatures:"
            << " min=" << *K2_min_max.first << " max=" << *K2_min_max.second
            << std::endl;
  const auto      error_K1 = SHG::getScalarsAbsoluteDifference( K1, exp_K1 );
  const auto stat_error_K1 = SHG::getStatistic( error_K1 );
  const auto   error_K1_l2 = SHG::getScalarsNormL2( K1, exp_K1 );
  trace.info() << "|K1-K1_CNC|_oo = " << stat_error_K1.max() << std::endl;
  trace.info() << "|K1-K1_CNC|_2  = " << error_K1_l2 << std::endl;
  const auto      error_K2 = SHG::getScalarsAbsoluteDifference( K2, exp_K2 );
  const auto stat_error_K2 = SHG::getStatistic( error_K2 );
  const auto   error_K2_l2 = SHG::getScalarsNormL2( K2, exp_K2 );
  trace.info() << "|K2-K2_CNC|_oo = " << stat_error_K2.max() << std::endl;
  trace.info() << "|K2-K2_CNC|_2  = " << error_K2_l2 << std::endl;
  //! [curvature-measures-check]

  //! [curvature-measures-output]
  // Remove normals for better blocky display.
  smesh.vertexNormals() = SH::RealVectors();
  smesh.faceNormals()   = SH::RealVectors();
  typedef SurfaceMeshWriter< RealPoint, RealVector > SMW;
  const double    Kmax = std::max( fabs( *exp_K1_min_max.first ),
                                   fabs( *exp_K2_min_max.second ) );
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
