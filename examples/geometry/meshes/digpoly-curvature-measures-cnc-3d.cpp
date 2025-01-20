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
 * @file geometry/meshes/digpoly-curvature-measures-cnc-3d.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/10/25
 *
 * An example file named digpoly-curvature-measures-cnc-3d.
 *
 * This file is part of the DGtal library.
 */

/**
   Computation of mean and Gaussian curvatures on a mesh defined by a
   an implicit shape discretized as a digital surface, using constant
   or interpolated corrected curvature measures (based on the theory
   of corrected normal currents). It uses a digital normal vector
   estimator to improve curvature estimations.
   Errors with respect to true expected curvatures are also computed.

This first example uses constant per face corrected normal vector
field to compute curvatures.

\verbatim
# "Al" vol file
./examples/geometry/meshes/digpoly-curvature-measures-cnc-3d torus 10 0.5 1.0 Const
\endverbatim

outputs

\verbatim
Using face-*Constant* Corrected Normal Current
- surface has 2584 surfels.
[SurfaceMesh (OK) #V=2584 #VN=0 #E=5168 #F=2584 #FN=0 E[IF]=4 E[IV]=4 E[IFE]=2]
- CTrivial normal t-ring=3 (discrete)
Expected mean curvatures: min=0.125 max=0.3125
Computed mean curvatures: min=0.118944 max=0.365712
Expected Gaussian curvatures: min=-0.125 max=0.0625
Computed Gaussian curvatures: min=-0.214727 max=0.09246
|H-H_CNC|_oo = 0.0798986
|H-H_CNC|_2  = 0.0274956
|G-G_CNC|_oo = 0.0897268
|G-G_CNC|_2  = 0.0176866
\endverbatim

This second example uses vertex-interpolated corrected normal vector
field to compute curvatures.

\verbatim
./examples/geometry/meshes/digpoly-curvature-measures-cnc-3d torus 10 0.5 1.0 Interp
\endverbatim

outputs

\verbatim
Using vertex-*Interpolated* Corrected Normal Current
- surface has 2584 surfels.
[SurfaceMesh (OK) #V=2584 #VN=0 #E=5168 #F=2584 #FN=0 E[IF]=4 E[IV]=4 E[IFE]=2]
- CTrivial normal t-ring=3 (discrete)
Expected mean curvatures: min=0.125 max=0.3125
Computed mean curvatures: min=0.146652 max=0.33483
Expected Gaussian curvatures: min=-0.125 max=0.0625
Computed Gaussian curvatures: min=-0.1556 max=0.0751507
|H-H_CNC|_oo = 0.0576324
|H-H_CNC|_2  = 0.0201468
|G-G_CNC|_oo = 0.0320427
|G-G_CNC|_2  = 0.00918083
\endverbatim

It also produces several OBJ files to display curvature estimation
results, `example-cnc-H.obj` and `example-cnc-G.obj` as well as the
associated MTL file.

<table>
<tr><td>
\image html digtorus-h0_5-ccnc-H-CTrivial-r1.jpg "Face-constant corrected mean curvature measure, r=1" width=90%
</td><td>
\image html digtorus-h0_5-ccnc-G-CTrivial-r1.jpg "Face-constant corrected Gaussian curvature measure, r=1" width=90%
</td><td>
\image html digtorus-h0_5-icnc-H-CTrivial-r1.jpg "Vertex-interpolated corrected mean curvature measure, r=1" width=90%
</td><td>
\image html digtorus-h0_5-icnc-G-CTrivial-r1.jpg "Vertex-interpolated corrected Gaussian curvature measure, r=1" width=90%
</td></tr>
</table>

@see \ref moduleCurvatureMeasures

\note In opposition with Normal Cycle curvature measures, constant or
interpolated corrected curvature measures can take into account an
external normal vector field to estimate curvatures with better
accuracy.

\example geometry/meshes/digpoly-curvature-measures-cnc-3d.cpp
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
            << "Computation of mean and Gaussian curvatures on an "      << std::endl
            << "digitized implicit shape using constant or "             << std::endl
            << "interpolated corrected curvature measures (based "       << std::endl
            << "on the theory of corrected normal currents)."            << std::endl
            << "- builds the surface mesh from polynomial <P>"           << std::endl
            << "- <B> defines the digitization space size [-B,B]^3"      << std::endl
            << "- <h> is the gridstep digitization"                      << std::endl
            << "- <R> is the radius of the measuring balls"              << std::endl
            << "- <mode> is either Const for constant corrected normal"  << std::endl
            << "  vector field or Interp for interpolated corrected"     << std::endl
            << "  normal vector field."                                  << std::endl
            << "It produces several OBJ files to display mean and"       << std::endl
            << "Gaussian curvature estimation results: `example-cnc-H.obj`" << std::endl
            << "and `example-cnc-G.obj` as well as the associated MTL file." << std::endl;
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
  auto exp_H = SHG::getMeanCurvatures( shape, K, surfels, params );
  auto exp_G = SHG::getGaussianCurvatures( shape, K, surfels, params );
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
  auto exp_H_min_max = std::minmax_element( exp_H.cbegin(), exp_H.cend() );
  auto exp_G_min_max = std::minmax_element( exp_G.cbegin(), exp_G.cend() );
  std::cout << "Expected mean curvatures:"
            << " min=" << *exp_H_min_max.first << " max=" << *exp_H_min_max.second
            << std::endl;
  std::cout << "Computed mean curvatures:"
            << " min=" << *H_min_max.first << " max=" << *H_min_max.second
            << std::endl;
  std::cout << "Expected Gaussian curvatures:"
            << " min=" << *exp_G_min_max.first << " max=" << *exp_G_min_max.second
            << std::endl;
  std::cout << "Computed Gaussian curvatures:"
            << " min=" << *G_min_max.first << " max=" << *G_min_max.second
            << std::endl;
  const auto      error_H = SHG::getScalarsAbsoluteDifference( H, exp_H );
  const auto stat_error_H = SHG::getStatistic( error_H );
  const auto   error_H_l2 = SHG::getScalarsNormL2( H, exp_H );
  trace.info() << "|H-H_CNC|_oo = " << stat_error_H.max() << std::endl;
  trace.info() << "|H-H_CNC|_2  = " << error_H_l2 << std::endl;
  const auto      error_G = SHG::getScalarsAbsoluteDifference( G, exp_G );
  const auto stat_error_G = SHG::getStatistic( error_G );
  const auto   error_G_l2 = SHG::getScalarsNormL2( G, exp_G );
  trace.info() << "|G-G_CNC|_oo = " << stat_error_G.max() << std::endl;
  trace.info() << "|G-G_CNC|_2  = " << error_G_l2 << std::endl;
  //! [curvature-measures-check]

  //! [curvature-measures-output]
  // Remove normals for better blocky display.
  smesh.vertexNormals() = SH::RealVectors();
  smesh.faceNormals()   = SH::RealVectors();
  typedef SurfaceMeshWriter< RealPoint, RealVector > SMW;
  const double    Hmax = std::max( fabs( *exp_H_min_max.first ),
                                   fabs( *exp_H_min_max.second ) );
  const double    Gmax = std::max( fabs( *exp_G_min_max.first ),
                                   fabs( *exp_G_min_max.second ) );
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
