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
 * @file geometry/meshes/curvature-comparator-ii-cnc-3d.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2022/02/27
 *
 * An example file named curvature-comparator-ii-cnc-3d.
 *
 * This file is part of the DGtal library.
 */

/**
   Computation of mean and Gaussian curvatures on a mesh defined by a
   an implicit shape discretized as a digital surface, using Integral
   Invariant (II) method or constant (CCNC) or interpolated corrected
   curvature measures (ICNC) (based on the theory of corrected normal
   currents). For CCNC and ICNC, we use also II normal vector
   estimations. Timings and errors with respect to true expected
   curvatures are computed.

This first example compares II with CCNC for estimating curvatures on
"goursat" polynomial, for gridstep 0.25

\verbatim
./examples/geometry/meshes/curvature-comparator-ii-cnc-3d goursat 10 0.25 Const
\endverbatim

outputs

\verbatim
\endverbatim

This second example compares II with ICNC for estimating curvatures on
"goursat" polynomial, for gridstep 0.25

\verbatim
./examples/geometry/meshes/curvature-comparator-ii-cnc-3d goursat 10 0.25 Interp
\endverbatim

outputs

\verbatim
\endverbatim

@see \ref moduleCurvatureMeasures

\note In opposition with Normal Cycle curvature measures, constant or
interpolated corrected curvature measures can take into account an
external normal vector field to estimate curvatures with better
accuracy.

\example geometry/meshes/curvature-comparator-ii-cnc-3d.cpp
*/

#include <iostream>
#include <fstream>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/shapes/SurfaceMesh.h"
//! [curvature-comparator-Includes]
#include "DGtal/geometry/meshes/CorrectedNormalCurrentComputer.h"
//! [curvature-comparator-Includes]

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
  ((void) argc); ((void) argv);
  using namespace DGtal;
  using namespace DGtal::Z3i;
  typedef Shortcuts< KSpace >          SH;
  std::cout << "Usage: " << std::endl
            << "\t" << argv[ 0 ] << " <P> <B> <h> <mode>" << std::endl
            << std::endl
            << "Compare Integral Invariant (II) curvature estimations "  << std::endl
            << "with Corrected Normal Current (CNC) estimations, either "<< std::endl
            << "interpolated (Interp) or constant per face (Const)."     << std::endl
            << "- builds the surface mesh from polynomial <P>"           << std::endl
            << "- <B> defines the digitization space size [-B,B]^3"      << std::endl
            << "- <h> is the gridstep digitization"                      << std::endl
            << "- <mode> is either Const for constant corrected normal"  << std::endl
            << "  vector field or Interp for interpolated corrected"     << std::endl
            << "  normal vector field."                                  << std::endl
            << "It outputs timings and accuracy statistics for both "    << std::endl
            << "methods."                                                << std::endl;
  std::cout << "You may either write your own polynomial as 3*x^2-z^2*x*y+1" << std::endl
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
  //! [curvature-comparator-Typedefs]
  using namespace DGtal;
  using namespace DGtal::Z3i;
  typedef SurfaceMesh< RealPoint, RealVector >                    SM;
  typedef CorrectedNormalCurrentComputer< RealPoint, RealVector > CNC;
  typedef Shortcuts< KSpace >          SH;
  typedef ShortcutsGeometry< KSpace >  SHG;
  //! [curvature-comparator-Typedefs]
  std::string  poly = argv[ 1 ]; // polynomial
  const double    B = argc > 2 ? atof( argv[ 2 ] ) : 1.0; // max ||_oo bbox
  const double    h = argc > 3 ? atof( argv[ 3 ] ) : 1.0; // gridstep  
  std::string  mode = argc > 4 ? argv[ 4 ] : "Const"; // either Const or Interp
  bool interpolated = mode == "Interp";
  if ( interpolated )
    trace.info() << "Using vertex-*Interpolated* Corrected Normal Current" << std::endl;
  else
    trace.info() << "Using face-*Constant* Corrected Normal Current" << std::endl;
  //! [curvature-comparator-DigitalSurface]
  // Read polynomial and build digital surface
  auto params = SH::defaultParameters() | SHG::defaultParameters();
  // Choose depth-first traversal to speed-up II computations.
  params( "surfaceTraversal", "DepthFirst" );
  params( "t-ring", 3 );
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
  //! [curvature-comparator-DigitalSurface]

  //! [curvature-comparator-ii]
  params( "r-radius", 3.0 );
  params( "alpha",    0.33 );
  double ii_r = 3.0 * pow( h, 0.33 );
  Clock c;
  trace.beginBlock( "Computing II curvatures" );
  std::vector< double > HII  = SHG::getIIMeanCurvatures    ( bimage, surfels, params );
  std::vector< double > GII  = SHG::getIIGaussianCurvatures( bimage, surfels, params );
  auto ii_t = trace.endBlock();
  //! [curvature-comparator-ii]
  
  //! [curvature-comparator-SurfaceMesh]
  SM smesh;
  std::vector< SM::Vertices > faces;
  SH::Cell2Index c2i;
  auto pointels = SH::getPointelRange( c2i, surface );
  auto vertices = SH::RealPoints( pointels.size() );
  std::transform( pointels.cbegin(), pointels.cend(), vertices.begin(),
                  [&] (const SH::Cell& cell) { return h * embedder( cell ); } ); 
  for ( auto&& surfel : surfels )
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
  //! [curvature-comparator-SurfaceMesh]

  //! [curvature-comparator-TrueCurvatures]
  trace.beginBlock( "Computing True curvatures" );
  auto exp_H = SHG::getMeanCurvatures( shape, K, surfels, params );
  auto exp_G = SHG::getGaussianCurvatures( shape, K, surfels, params );
  trace.endBlock();
  //! [curvature-comparator-TrueCurvatures]

  trace.beginBlock( "Computing CNC curvatures" );
  //! [curvature-comparator-CNC]
  // Builds a CorrectedNormalCurrentComputer object onto the SurfaceMesh object
  CNC cnc( smesh );
  // Estimates normal vectors using Integral Invariant Normal estimator 
  trace.beginBlock( "Computing II normal vectors" );
  auto face_normals = SHG::getIINormalVectors( bimage, surfels, params );
  double cnc_tn = trace.endBlock();
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
  //! [curvature-comparator-CNC]

  //! [curvature-comparator-estimations]
  // estimates mean (H) and Gaussian (G) curvatures by measure normalization.
  double cnc_mr = 1.0 * sqrt( h );
  trace.info() << "CNC measuring radius = " << cnc_mr << std::endl;
  std::vector< double > H( smesh.nbFaces() );
  std::vector< double > G( smesh.nbFaces() );
  for ( auto f = 0; f < smesh.nbFaces(); ++f )
    {
      const auto b    = smesh.faceCentroid( f );
      const auto area = mu0.measure( b, cnc_mr, f );
      H[ f ] = cnc.meanCurvature    ( area, mu1.measure( b, cnc_mr, f ) );
      G[ f ] = cnc.GaussianCurvature( area, mu2.measure( b, cnc_mr, f ) );
    }
  //! [curvature-comparator-estimations]
  double cnc_t = trace.endBlock();
  
  //! [curvature-comparator-check]
  auto HII_min_max = std::minmax_element( HII.cbegin(), HII.cend() );
  auto GII_min_max = std::minmax_element( GII.cbegin(), GII.cend() );
  auto H_min_max = std::minmax_element( H.cbegin(), H.cend() );
  auto G_min_max = std::minmax_element( G.cbegin(), G.cend() );
  auto exp_H_min_max = std::minmax_element( exp_H.cbegin(), exp_H.cend() );
  auto exp_G_min_max = std::minmax_element( exp_G.cbegin(), exp_G.cend() );
  trace.info() << "Expected mean curvatures:"
            << " min=" << *exp_H_min_max.first << " max=" << *exp_H_min_max.second
            << std::endl;
  trace.info() << "Computed II mean curvatures:"
            << " min=" << *HII_min_max.first << " max=" << *HII_min_max.second
            << std::endl;
  trace.info() << "Computed CNC mean curvatures:"
            << " min=" << *H_min_max.first << " max=" << *H_min_max.second
            << std::endl;
  trace.info() << "Expected Gaussian curvatures:"
            << " min=" << *exp_G_min_max.first << " max=" << *exp_G_min_max.second
            << std::endl;
  trace.info() << "Computed II Gaussian curvatures:"
            << " min=" << *GII_min_max.first << " max=" << *GII_min_max.second
            << std::endl;
  trace.info() << "Computed CNC Gaussian curvatures:"
            << " min=" << *G_min_max.first << " max=" << *G_min_max.second
            << std::endl;
  const auto      error_HII = SHG::getScalarsAbsoluteDifference( HII, exp_H );
  const auto stat_error_HII = SHG::getStatistic( error_HII );
  const auto   error_HII_l2 = SHG::getScalarsNormL2( HII, exp_H );
  trace.info() << "|H-H_II|_oo = " << stat_error_HII.max() << std::endl;
  trace.info() << "|H-H_II|_2  = " << error_HII_l2 << std::endl;
  const auto      error_H = SHG::getScalarsAbsoluteDifference( H, exp_H );
  const auto stat_error_H = SHG::getStatistic( error_H );
  const auto   error_H_l2 = SHG::getScalarsNormL2( H, exp_H );
  trace.info() << "|H-H_CNC|_oo = " << stat_error_H.max() << std::endl;
  trace.info() << "|H-H_CNC|_2  = " << error_H_l2 << std::endl;
  const auto      error_GII = SHG::getScalarsAbsoluteDifference( GII, exp_G );
  const auto stat_error_GII = SHG::getStatistic( error_GII );
  const auto   error_GII_l2 = SHG::getScalarsNormL2( GII, exp_G );
  trace.info() << "|G-G_II|_oo = " << stat_error_GII.max() << std::endl;
  trace.info() << "|G-G_II|_2  = " << error_GII_l2 << std::endl;
  const auto      error_G = SHG::getScalarsAbsoluteDifference( G, exp_G );
  const auto stat_error_G = SHG::getStatistic( error_G );
  const auto   error_G_l2 = SHG::getScalarsNormL2( G, exp_G );
  trace.info() << "|G-G_CNC|_oo = " << stat_error_G.max() << std::endl;
  trace.info() << "|G-G_CNC|_2  = " << error_G_l2 << std::endl;
  //! [curvature-comparator-check]

  //! [curvature-comparator-results]
  std::cout << "# " << argv[ 0 ] << std::endl
            << "# polynomial: " << poly << std::endl
            << "# CNC mode:   " << mode << std::endl;
  //              1 2          3    4    5      6     7      8
  std::cout << "# h nb_surfels ii_t ii_r ii_Hoo ii_H2 ii_Goo ii_G2 ";
  //            9      10    11     12      13     14      15
  std::cout << "cnc_tn cnc_t cnc_mr cnc_Hoo cnc_H2 cnc_Goo cnc_G2" << std::endl;
  std::cout << h << " " << surfels.size() << " " << ii_t << " " << ii_r
            << " " << stat_error_HII.max() << " " << error_HII_l2
            << " " << stat_error_GII.max() << " " << error_GII_l2
            << " " << cnc_tn << " " << cnc_t << " " << cnc_mr
            << " " << stat_error_H.max() << " " << error_H_l2
            << " " << stat_error_G.max() << " " << error_G_l2 << std::endl;
  //! [curvature-comparator-results]
  return 0;
}
