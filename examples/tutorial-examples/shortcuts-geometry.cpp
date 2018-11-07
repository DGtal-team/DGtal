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
 * @file
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2018/06/26
 *
 * An example file named shortcuts.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "ConfigExamples.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"
#include "DGtal/base/Common.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  unsigned int nb = 0, nbok = 0;
  // 3d tests
  typedef Shortcuts< Z3i::KSpace >         SH3;
  typedef ShortcutsGeometry< Z3i::KSpace > SHG3;

  trace.beginBlock ( "Build polynomial shape -> digitize -> extract ground-truth geometry." );
  {
    auto params          = SH3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_2_5s]
    params( "polynomial", "3*x^2+2*y^2+z^2-90" )( "gridstep", 0.25 );
    auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
    auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
    auto binary_image    = SH3::makeBinaryImage      ( digitized_shape, params );
    auto K               = SH3::getKSpace( params );
    auto surface         = SH3::makeLightDigitalSurface( binary_image, K, params );
    auto surfels         = SH3::getSurfelRange( surface, params );
    auto positions       = SHG3::getPositions( implicit_shape, K, surfels, params );
    auto normals         = SHG3::getNormalVectors( implicit_shape, K, surfels, params );
    auto mean_curvs      = SHG3::getMeanCurvatures( implicit_shape, K, surfels, params );
    auto gauss_curvs     = SHG3::getGaussianCurvatures( implicit_shape, K, surfels, params );
    //! [dgtal_shortcuts_ssec2_2_5s]
    auto stat_mean       = SHG3::getStatistic( mean_curvs );
    auto stat_gauss      = SHG3::getStatistic( gauss_curvs );
    trace.info() << " min(H)=" << stat_mean.min()
		 << " avg(H)=" << stat_mean.mean()
		 << " max(H)=" << stat_mean.max() << std::endl;
    trace.info() << " min(G)=" << stat_gauss.min()
		 << " avg(G)=" << stat_gauss.mean()
		 << " max(G)=" << stat_gauss.max() << std::endl;
    ++nb, nbok += positions.size() == surfels.size() ? 1 : 0;
    ++nb, nbok += normals.size() == surfels.size() ? 1 : 0;
    ++nb, nbok += mean_curvs.size() == surfels.size() ? 1 : 0;
    ++nb, nbok += gauss_curvs.size() == surfels.size() ? 1 : 0;
    ++nb, nbok += stat_mean.min() > 0.08 ? 1 : 0;
    ++nb, nbok += stat_gauss.min() > 0.0064 ? 1 : 0;
  }
  trace.endBlock();
  
  trace.beginBlock ( "Setting up shape, space, etc" );
  auto params         = SH3::defaultParameters()
    | SHG3::defaultParameters();
  params( "polynomial", "goursat" )( "gridstep", 0.5 )
    ( "surfaceComponents", "All" )( "surfelAdjacency",   0 );
  auto implicit_shape = SH3::makeImplicitShape3D( params );
  auto K              = SH3::getKSpace( params );
  auto digital_shape  = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto binary_image   = SH3::makeBinaryImage( digital_shape, params );
  trace.endBlock();
  
  trace.beginBlock ( "Compute true geometry" );
  //params( "surfaceTraversal", "DepthFirst" )( "verbose", 0 );
  params( "surfaceTraversal", "Default" )( "verbose", 0 );
  auto surface     = SH3::makeLightDigitalSurface( binary_image, K, params );
  auto surfels     = SH3::getSurfelRange( surface, params );
  auto positions   = SHG3::getPositions( implicit_shape, K, surfels, params ); 
  auto normals     = SHG3::getNormalVectors( implicit_shape, K, surfels, params ); 
  auto mean_curv   = SHG3::getMeanCurvatures( implicit_shape, K, surfels, params ); 
  auto gauss_curv  = SHG3::getGaussianCurvatures( implicit_shape, K, surfels, params ); 
  SH3::RealPoint p = positions.front();
  SH3::RealPoint q = p;
  for ( auto&& r : positions ) {
    p = p.inf( r ); q = q.sup( r );
  }
  SH3::Scalar   h0 =  1000.0;
  SH3::Scalar   h1 = -1000.0;
  for ( auto&& h : mean_curv ) {
    h0 = std::min( h0, h );	h1 = std::max( h1, h );
  }
  SH3::Scalar   g0 =  1000.0;
  SH3::Scalar   g1 = -1000.0;
  for ( auto&& g : gauss_curv ) {
    g0 = std::min( g0, g );	g1 = std::max( g1, g );
  }
  std::cout << "#position = " << positions.size()
	      << " p=" << p << " q=" << q << std::endl;
  std::cout << "#normals = " << normals.size() << std::endl;
  std::cout << "H_min = " << h0 << " H_max = " << h1;
  std::cout << " expected: H_min = 0.0912870 H_max = 0.263523" << std::endl;
  std::cout << "G_min = " << g0 << " G_max = " << g1;
  std::cout << " expected: G_min = 0.0074074 G_max = 0.0666666" << std::endl;
  trace.endBlock();
  
  trace.beginBlock ( "Estimate geometry" );
  auto     t_normals = SHG3::getTrivialNormalVectors( K, surfels );
  auto    ct_normals = SHG3::getCTrivialNormalVectors( surface, surfels, params );
  auto   vcm_normals = SHG3::getVCMNormalVectors( surface, surfels, params );
  auto    ii_normals = SHG3::getIINormalVectors( binary_image, surfels, params );
  // Need to reorient II normals with CTrivial (otherwise unstable orientation).
  SHG3::orientVectors( ii_normals, ct_normals );
  auto   t_angle_dev = SHG3::getVectorsAngleDeviation( normals, t_normals );
  auto  ct_angle_dev = SHG3::getVectorsAngleDeviation( normals, ct_normals );
  auto vcm_angle_dev = SHG3::getVectorsAngleDeviation( normals, vcm_normals );
  auto  ii_angle_dev = SHG3::getVectorsAngleDeviation( normals, ii_normals );
  auto        stat_t = SHG3::getStatistic(   t_angle_dev );
  auto       stat_ct = SHG3::getStatistic(  ct_angle_dev );
  auto      stat_vcm = SHG3::getStatistic( vcm_angle_dev );
  auto       stat_ii = SHG3::getStatistic(  ii_angle_dev );
  std::cout << "Trivial  angle_dev  mean="
	    << stat_t.mean() << " max=" << stat_t.max() << std::endl;
  std::cout << "CTrivial angle_dev  mean="
	    << stat_ct.mean() << " max=" << stat_ct.max() << std::endl;
  std::cout << "VCM      angle_dev  mean="
	    << stat_vcm.mean() << " max=" << stat_vcm.max() << std::endl;
  std::cout << "II       angle_dev  mean="
	    << stat_ii.mean() << " max=" << stat_ii.max() << std::endl;
  auto M = std::max( std::max( stat_t.max(),   stat_ct.max() ),
		     std::max( stat_vcm.max(), stat_ii.max() ) );
  trace.endBlock();
  trace.beginBlock ( "Save as OBj with normals" );
  {
    auto default_surfels = SH3::getSurfelRange( surface, Parameters( "Traversal", "Default" ) );
    auto match    = SH3::getRangeMatch( default_surfels, surfels );
    auto polysurf = SH3::makePrimalPolygonalSurface( surface );
    auto normals  = SH3::getMatchedRange( vcm_normals, match );
    // for ( SH3::Idx i = 0; i < normals.size(); i++ )
    // 	normals[ i ] = vcm_normals[ match[ i ] ]; 
    bool ok       = SH3::saveOBJ( polysurf, normals, SH3::Colors(),
				  "goursat-vcm-n.obj" );
    ++nb, nbok += ok ? 1 : 0; 
    auto cmap     = SH3::getColorMap( -0.3, 0.3 );
    auto colors   = SH3::Colors( normals.size() );
    for ( SH3::Idx i = 0; i < normals.size(); i++ )
      colors[ i ] = cmap( mean_curv[ match[ i ] ] ); 
    bool ok2      = SH3::saveOBJ( polysurf, normals, colors,
				  "goursat-vcm-mcurv.obj" );
    ++nb, nbok += ok2 ? 1 : 0; 
    auto errcmap  = SH3::getColorMap( 0.0, M, Parameters( "colormap", "Tics" ) );
    // Output error for trivial normals
    normals       = SH3::getMatchedRange( t_normals, match );
    for ( SH3::Idx i = 0; i < normals.size(); i++ )
      colors[ i ] = errcmap( t_angle_dev[ match[ i ] ] ); 
    bool ok_t     = SH3::saveOBJ( polysurf, normals, colors, "goursat-t-err.obj" );
    // Output error for convolved trivial normals
    normals       = SH3::getMatchedRange( ct_normals, match );
    for ( SH3::Idx i = 0; i < normals.size(); i++ )
      colors[ i ] = errcmap( ct_angle_dev[ match[ i ] ] ); 
    bool ok_ct    = SH3::saveOBJ( polysurf, normals, colors, "goursat-ct-err.obj" );
    // Output error for vcm trivial normals
    normals       = SH3::getMatchedRange( vcm_normals, match );
    for ( SH3::Idx i = 0; i < normals.size(); i++ )
      colors[ i ] = errcmap( vcm_angle_dev[ match[ i ] ] ); 
    bool ok_vcm   = SH3::saveOBJ( polysurf, normals, colors, "goursat-vcm-err.obj" );
    // Output error for ii trivial normals
    normals       = SH3::getMatchedRange( ii_normals, match );
    for ( SH3::Idx i = 0; i < normals.size(); i++ )
      colors[ i ] = errcmap( ii_angle_dev[ match[ i ] ] ); 
    bool ok_ii    = SH3::saveOBJ( polysurf, normals, colors, "goursat-ii-err.obj" );
    ++nb, nbok += ok_t   ? 1 : 0; 
    ++nb, nbok += ok_ct  ? 1 : 0; 
    ++nb, nbok += ok_vcm ? 1 : 0; 
    ++nb, nbok += ok_ii  ? 1 : 0; 
  }
  trace.endBlock();
  
  {
    trace.beginBlock ( "Gauss curv example" );
    auto params         = SH3::defaultParameters()
      | SHG3::defaultParameters();
    params( "polynomial", "leopold" )( "gridstep", 0.125 )
      ( "surfaceComponents", "All" )( "surfelAdjacency",   0 )
      ( "surfaceTraversal", "Default" )( "verbose", 0 )
      ( "projectionMaxIter", 50 )( "projectionAccuracy", 0.00001 )
      ( "projectionGamma", 0.05 ); // leopold requires less gamma in projections.
    auto implicit_shape = SH3::makeImplicitShape3D( params );
    auto K              = SH3::getKSpace( params );
    auto digital_shape  = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
    auto binary_image   = SH3::makeBinaryImage( digital_shape, params );
    auto surface     = SH3::makeLightDigitalSurface( binary_image, K, params );
    auto surfels     = SH3::getSurfelRange( surface, params );
    auto normals     = SHG3::getNormalVectors( implicit_shape, K, surfels, params ); 
    auto gauss_curv  = SHG3::getMeanCurvatures( implicit_shape, K, surfels, params );
    auto cmap        = SH3::getColorMap( -0.01, 0.01, Parameters( "colormap", "Tics" ) );
    auto colors      = SH3::Colors( normals.size() );
    std::transform( gauss_curv.cbegin(), gauss_curv.cend(), colors.begin(), cmap );
    bool ok          = SH3::saveOBJ( surface, normals, colors, "leopold-G.obj" );
    ++nb, nbok += ok ? 1 : 0; 
    trace.endBlock();
  }
  trace.info() << nbok << "/" << nb << " passed tests." << std::endl;
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
