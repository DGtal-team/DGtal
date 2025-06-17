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

int main( int /* argc */, char** /* argv */ )
{
  unsigned int nb = 0, nbok = 0;
  // 3d tests
  typedef Shortcuts< Z3i::KSpace >         SH3;
  typedef ShortcutsGeometry< Z3i::KSpace > SHG3;

  trace.beginBlock ( "Load vol file -> build digital surface -> estimate mean curvature -> save OBJ." );
  {
    //! [dgtal_shortcuts_ssec2_1_6s]
    auto params    = SH3::defaultParameters() | SHG3::defaultParameters();
    params( "colormap", "Tics" );
    auto bimage    = SH3::makeBinaryImage( examplesPath + "samples/Al.100.vol", params );
    auto K         = SH3::getKSpace( bimage, params );
    auto surface   = SH3::makeDigitalSurface( bimage, K, params );
    auto surfels   = SH3::getSurfelRange( surface, params );
    auto curv      = SHG3::getIIMeanCurvatures( bimage, surfels, params );
    // To get Gauss curvatures, write instead:
    // auto curv = SHG3::getIIGaussianCurvatures( bimage, surfels, params );
    auto cmap      = SH3::getColorMap( -0.5, 0.5, params );
    auto colors    = SH3::Colors( surfels.size() );
    std::transform( curv.cbegin(), curv.cend(), colors.begin(), cmap );
    bool ok        = SH3::saveOBJ( surface, SH3::RealVectors(), colors,
        			   "al-H-II.obj" );
    //! [dgtal_shortcuts_ssec2_1_6s]
    ++nb; nbok += ok ? 1 : 0;
  }
  trace.endBlock();

  trace.beginBlock ( "Load vol file -> build digital surface -> estimate Gauss curvature -> save OBJ." );
  {
    auto params    = SH3::defaultParameters() | SHG3::defaultParameters();
    params( "colormap", "Tics" );
    auto bimage    = SH3::makeBinaryImage( examplesPath + "samples/Al.100.vol", params );
    auto K         = SH3::getKSpace( bimage, params );
    auto surface   = SH3::makeDigitalSurface( bimage, K, params );
    auto surfels   = SH3::getSurfelRange( surface, params );
    auto curv      = SHG3::getIIGaussianCurvatures( bimage, surfels, params );
    auto cmap      = SH3::getColorMap( -0.25, 0.25, params );
    auto colors    = SH3::Colors( surfels.size() );
    std::transform( curv.cbegin(), curv.cend(), colors.begin(), cmap );
    bool ok        = SH3::saveOBJ( surface, SH3::RealVectors(), colors,
        			   "al-G-II.obj" );
    ++nb; nbok += ok ? 1 : 0;
  }
  trace.endBlock();
  
  trace.beginBlock ( "Build polynomial shape -> digitize -> extract ground-truth geometry." );
  {
    auto params          = SH3::defaultParameters() | SHG3::defaultParameters();
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
    ++nb; nbok += positions.size() == surfels.size() ? 1 : 0;
    ++nb; nbok += normals.size() == surfels.size() ? 1 : 0;
    ++nb; nbok += mean_curvs.size() == surfels.size() ? 1 : 0;
    ++nb; nbok += gauss_curvs.size() == surfels.size() ? 1 : 0;
    ++nb; nbok += stat_mean.min() > 0.08 ? 1 : 0;
    ++nb; nbok += stat_gauss.min() > 0.0064 ? 1 : 0;
  }
  trace.endBlock();

  trace.beginBlock ( "Build polynomial shape -> digitize -> get pointels -> save projected quadrangulated surface." );
  {
    auto params          = SH3::defaultParameters() | SHG3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_2_6s]
    const double h       = 0.25;
    params( "polynomial", "goursat" )( "gridstep", h );
    auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
    auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
    auto binary_image    = SH3::makeBinaryImage      ( digitized_shape, params );
    auto K               = SH3::getKSpace( params );
    auto embedder        = SH3::getCellEmbedder( K );
    auto surface         = SH3::makeLightDigitalSurface( binary_image, K, params );
    SH3::Cell2Index c2i;
    auto pointels        = SH3::getPointelRange( c2i, surface );
    SH3::RealPoints pos( pointels.size() );
    std::transform( pointels.cbegin(), pointels.cend(), pos.begin(),
        	    [&] (const SH3::Cell& c) { return h * embedder( c ); } ); 
    auto ppos     = SHG3::getPositions( implicit_shape, pos, params );
    bool ok       = SH3::saveOBJ( surface,
        			  [&] (const SH3::Cell& c){ return ppos[ c2i[ c ] ];},
        			  SH3::RealVectors(), SH3::Colors(),
        			  "goursat-quad-proj.obj" );
    //! [dgtal_shortcuts_ssec2_2_6s]
    ++nb; nbok += ok ? 1 : 0;
  }
  trace.endBlock();

  trace.beginBlock ( "Build polynomial shape -> digitize -> extract mean curvature -> save as OBJ with colors." );
  {
    auto params          = SH3::defaultParameters() | SHG3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_2_7s]
    params( "polynomial", "goursat" )( "gridstep", 0.25 )( "colormap", "Tics" );
    auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
    auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
    auto binary_image    = SH3::makeBinaryImage      ( digitized_shape, params );
    auto K               = SH3::getKSpace( params );
    auto surface         = SH3::makeLightDigitalSurface( binary_image, K, params );
    auto surfels         = SH3::getSurfelRange( surface, params );
    auto mean_curv       = SHG3::getMeanCurvatures( implicit_shape, K, surfels, params );
    auto cmap            = SH3::getColorMap( -0.3, 0.3, params );
    auto colors          = SH3::Colors( surfels.size() );
    std::transform( mean_curv.cbegin(), mean_curv.cend(), colors.begin(), cmap );
    bool ok              = SH3::saveOBJ( surface, SH3::RealVectors(), colors,
        				 "goursat-H.obj" );
    //! [dgtal_shortcuts_ssec2_2_7s]
    ++nb; nbok += ok ? 1 : 0;
  }
  trace.endBlock();

  trace.beginBlock ( "Build polynomial shape -> digitize -> extract ground-truth and estimated mean curvature -> display errors in OBJ with colors." );
  {
    auto params          = SH3::defaultParameters() | SHG3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_2_8s]
    params( "polynomial", "goursat" )( "gridstep", 0.25 )( "colormap", "Tics" )
      ( "R-radius", 5.0 );
    auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
    auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
    auto bimage          = SH3::makeBinaryImage      ( digitized_shape, params );
    auto K               = SH3::getKSpace( params );
    auto surface         = SH3::makeLightDigitalSurface( bimage, K, params );
    auto surfels         = SH3::getSurfelRange( surface, params );
    auto t_curv          = SHG3::getMeanCurvatures( implicit_shape, K, surfels, params );
    auto ii_curv         = SHG3::getIIMeanCurvatures( bimage, surfels, params );
    auto cmap            = SH3::getColorMap( -0.5, 0.5, params );
    auto colors          = SH3::Colors( surfels.size() );
    std::transform( t_curv.cbegin(),  t_curv.cend(),  colors.begin(), cmap );
    bool ok_t  = SH3::saveOBJ( surface, SH3::RealVectors(), colors, "goursat-H.obj" );
    std::transform( ii_curv.cbegin(), ii_curv.cend(), colors.begin(), cmap );
    bool ok_ii = SH3::saveOBJ( surface, SH3::RealVectors(), colors, "goursat-H-ii.obj" );
    auto errors          = SHG3::getScalarsAbsoluteDifference( t_curv, ii_curv );
    auto stat_errors     = SHG3::getStatistic( errors );
    auto cmap_errors     = SH3::getColorMap( 0.0, stat_errors.max(), params );
    std::transform( errors.cbegin(), errors.cend(), colors.begin(), cmap_errors );
    bool ok_err = SH3::saveOBJ( surface, SH3::RealVectors(), colors, "goursat-H-ii-err.obj" );
    trace.info() << "Error Loo=" << SHG3::getScalarsNormLoo( t_curv, ii_curv )
        	 << " L1="       << SHG3::getScalarsNormL1 ( t_curv, ii_curv )
        	 << " L2="       << SHG3::getScalarsNormL2 ( t_curv, ii_curv )
        	 << std::endl;
    //! [dgtal_shortcuts_ssec2_2_8s]
    ++nb; nbok += ( ok_t && ok_ii && ok_err ) ? 1 : 0;
  }
  trace.endBlock();
    
  trace.beginBlock ( "Build polynomial shape -> digitize -> build digital surface -> save primal surface with VCM normals as obj." );
  {
    auto params          = SH3::defaultParameters() | SHG3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_2_9s]
    params( "polynomial", "goursat" )( "gridstep", 0.25 )
      ( "surfaceTraversal", "Default" );
    auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
    auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
    auto K               = SH3::getKSpace( params );
    auto binary_image    = SH3::makeBinaryImage( digitized_shape, params );
    auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
    auto surfels         = SH3::getSurfelRange( surface, params );
    auto vcm_normals     = SHG3::getVCMNormalVectors( surface, surfels, params );
    bool ok              = SH3::saveOBJ( surface, vcm_normals, SH3::Colors(),
        				 "goursat-primal-vcm.obj" );
    //! [dgtal_shortcuts_ssec2_2_9s]
    ++nb; nbok += ok ? 1 : 0;
  }
  trace.endBlock();

  trace.beginBlock ( "Build polynomial shape -> digitize implicitly -> estimate II normals and curvature." );
  {
    auto params          = SH3::defaultParameters() | SHG3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_2_10s]
    params( "polynomial", "goursat" )( "gridstep", .25 );
    auto implicit_shape  = SH3::makeImplicitShape3D     ( params );
    auto dig_shape       = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
    auto K               = SH3::getKSpace               ( params );
    auto surface         = SH3::makeDigitalSurface      ( dig_shape, K, params );
    auto surfels         = SH3::getSurfelRange          ( surface, params( "surfaceTraversal", "DepthFirst" ) );
    auto def_surfels     = SH3::getSurfelRange          ( surface, params( "surfaceTraversal", "Default" ) );
    auto ii_normals      = SHG3::getIINormalVectors     ( dig_shape, surfels, params );
    trace.beginBlock( "II with default traversal (slower)" );
    auto ii_mean_curv    = SHG3::getIIMeanCurvatures    ( dig_shape, def_surfels, params );
    trace.endBlock();
    trace.beginBlock( "II with depth-first traversal (faster)" );
    auto ii_mean_curv2   = SHG3::getIIMeanCurvatures    ( dig_shape, surfels, params );
    trace.endBlock();
    auto cmap            = SH3::getColorMap             ( -0.5, 0.5, params );
    auto colors          = SH3::Colors                  ( def_surfels.size() );
    auto match           = SH3::getRangeMatch           ( def_surfels, surfels );
    auto normals         = SH3::getMatchedRange         ( ii_normals, match );
    for ( SH3::Idx i = 0; i < colors.size(); i++ )
      colors[ i ] = cmap( ii_mean_curv[ match[ i ] ] ); 
    bool ok_H  = SH3::saveOBJ( surface, SH3::RealVectors(), colors, "goursat-imp-H-ii.obj" );
    //! [dgtal_shortcuts_ssec2_2_10s]
    ++nb; nbok += ( ok_H && ii_mean_curv.size() == ii_mean_curv2.size() ) ? 1 : 0;
  }
  trace.endBlock();

  trace.beginBlock ( "Build polynomial shape -> save several projected quadrangulated surface and digitized boundaries." );
  {
    auto params          = SH3::defaultParameters() | SHG3::defaultParameters();
    std::vector<double> gridsteps {0.5, 0.25, 0.125};
    for ( auto h : gridsteps ) {
      params( "polynomial", "goursat" )( "gridstep", h );
      auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
      auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
      auto binary_image    = SH3::makeBinaryImage      ( digitized_shape, params );
      auto K               = SH3::getKSpace( params );
      auto embedder        = SH3::getCellEmbedder( K );
      auto surface         = SH3::makeLightDigitalSurface( binary_image, K, params );
      SH3::Cell2Index c2i;
      auto pointels        = SH3::getPointelRange( c2i, surface );
      SH3::RealPoints pos( pointels.size() );
      std::transform( pointels.cbegin(), pointels.cend(), pos.begin(),
        	      [&] (const SH3::Cell& c) { return h * embedder( c ); } ); 
      auto ppos       = SHG3::getPositions( implicit_shape, pos, params );
      auto fname      = std::string( "goursat-quad-" ) + std::to_string( h ) + std::string( ".obj" );
      bool ok         = SH3::saveOBJ( surface,
        			      [&] (const SH3::Cell& c){ return pos[ c2i[ c ] ];},
        			      SH3::RealVectors(), SH3::Colors(),
        			      fname );
      auto proj_fname = std::string( "goursat-quad-proj-" ) + std::to_string( h ) + std::string( ".obj" );
      bool proj_ok    = SH3::saveOBJ( surface,
        			      [&] (const SH3::Cell& c){ return ppos[ c2i[ c ] ];},
        			      SH3::RealVectors(), SH3::Colors(),
        			      proj_fname );
      ++nb; nbok += ok      ? 1 : 0;
      ++nb; nbok += proj_ok ? 1 : 0;
    }
  }
  trace.endBlock();

  trace.beginBlock ( "Build polynomial shape -> digitize -> digital surface -> save primal surface and VCM normal field as obj." );
  {
    auto params          = SH3::defaultParameters() | SHG3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_2_11s]
    params( "polynomial", "goursat" )( "gridstep", 0.5 )
      ( "surfaceTraversal", "Default" );
    auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
    auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
    auto K               = SH3::getKSpace( params );
    auto binary_image    = SH3::makeBinaryImage( digitized_shape, params );
    auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
    auto surfels         = SH3::getSurfelRange( surface, params );
    auto vcm_normals     = SHG3::getVCMNormalVectors( surface, surfels, params );
    auto embedder        = SH3::getSCellEmbedder( K );
    SH3::RealPoints positions( surfels.size() );
    std::transform( surfels.cbegin(), surfels.cend(), positions.begin(),
        	    [&] (const SH3::SCell& c) { return embedder( c ); } ); 
    bool ok              = SH3::saveOBJ( surface, vcm_normals, SH3::Colors(),
        				 "goursat-primal-vcm.obj" );
    bool ok2             = SH3::saveVectorFieldOBJ( positions, vcm_normals, 0.05, SH3::Colors(),
        				 "goursat-primal-vcm-normals.obj",
        				 SH3::Color( 0, 0, 0 ), SH3::Color::Red );
    //! [dgtal_shortcuts_ssec2_2_11s]
    ++nb, nbok += ok ? 1 : 0;
    ++nb, nbok += ok2 ? 1 : 0;
  }
  trace.endBlock();

  trace.beginBlock ( "Build polynomial shape -> digitize -> extract ground-truth curvatures -> display in OBJ." );
  {
    auto params          = SH3::defaultParameters() | SHG3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_2_13s]
    params( "polynomial", "goursat" )( "gridstep", 0.25 )( "colormap", "Tics" );
    auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
    auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
    auto bimage          = SH3::makeBinaryImage      ( digitized_shape, params );
    auto K               = SH3::getKSpace( params );
    auto surface         = SH3::makeLightDigitalSurface( bimage, K, params );
    auto surfels         = SH3::getSurfelRange( surface, params );
    auto k1              = SHG3::getFirstPrincipalCurvatures( implicit_shape, K, surfels, params );
    auto k2              = SHG3::getSecondPrincipalCurvatures( implicit_shape, K, surfels, params );
    auto d1              = SHG3::getFirstPrincipalDirections( implicit_shape, K, surfels, params );
    auto d2              = SHG3::getSecondPrincipalDirections( implicit_shape, K, surfels, params );
    auto embedder        = SH3::getSCellEmbedder( K );
    SH3::RealPoints positions( surfels.size() );
    std::transform( surfels.cbegin(), surfels.cend(), positions.begin(),
        	    [&] (const SH3::SCell& c) { return embedder( c ); } ); 
    SH3::saveOBJ( surface, SH3::RealVectors(), SH3::Colors(),
			       "goursat-primal.obj" );
    // output principal curvatures and directions
    auto cmap  = SH3::getColorMap( -0.5, 0.5, params );
    auto colors= SH3::Colors( surfels.size() );
    std::transform( k1.cbegin(), k1.cend(), colors.begin(), cmap );
    bool ok_k1 = SH3::saveOBJ( surface, SH3::RealVectors(), colors, "goursat-primal-k1.obj" );
    bool ok_d1 = SH3::saveVectorFieldOBJ( positions, d1, 0.05, colors,
					  "goursat-primal-d1.obj", SH3::Color::Black );
    std::transform( k2.cbegin(), k2.cend(), colors.begin(), cmap );
    bool ok_k2 = SH3::saveOBJ( surface, SH3::RealVectors(), colors, "goursat-primal-k2.obj" );
    bool ok_d2 = SH3::saveVectorFieldOBJ( positions, d2, 0.05, colors,
					  "goursat-primal-d2.obj", SH3::Color::Black );
    ASSERT(ok_k1 && ok_d1 && ok_k2 && ok_d2);
    //! [dgtal_shortcuts_ssec2_2_13s]
  }
  trace.endBlock();

  trace.beginBlock( "Load mesh file -> estimate mean/gaussian/principal curvatures -> display in obj" );
  {
    //! [dgtal_shortcuts_ssec2_1_14s]
    auto params = SH3::defaultParameters() | SHG3::defaultParameters();
    auto mesh = SH3::loadSurfaceMesh(examplesPath + "samples/lion.obj");

    auto mcurv = SHG3::getMeanCurvatures(mesh, params);
    auto gcurv = SHG3::getGaussianCurvatures(mesh, params);
    auto [k1, k2, d1, d2] = SHG3::getPrincipalCurvaturesAndDirections(mesh);
    auto cmap  = SH3::getColorMap( -0.5, 0.5, params );

    auto mcolors = SH3::Colors( mcurv.size() );
    std::transform( mcurv.cbegin(), mcurv.cend(), mcolors.begin(), cmap );

    auto gcolors = SH3::Colors( gcurv.size() );
    std::transform( gcurv.cbegin(), gcurv.cend(), gcolors.begin(), cmap );

    auto k1colors = SH3::Colors( k1.size() );
    std::transform( k1.begin(), k1.end(), k1colors.begin(), cmap);

    auto k2colors = SH3::Colors( k2.size() );
    std::transform( k2.begin(), k2.end(), k2colors.begin(), cmap);

    bool ok_m = SH3::saveOBJ( mesh, SH3::RealVectors(), mcolors, "lion-meanCurvature.obj" );
    bool ok_g = SH3::saveOBJ( mesh, SH3::RealVectors(), gcolors, "lion-gaussianCurvature.obj" );
    bool ok_k1 = SH3::saveOBJ( mesh, SH3::RealVectors(), k1colors, "lion-firstPrincipalCurvature.obj" );
    bool ok_k2 = SH3::saveOBJ( mesh, SH3::RealVectors(), k2colors, "lion-secondPrincpalCurvature.obj" );
    //! [dgtal_shortcuts_ssec2_1_14s]

    ++nb; nbok += ok_m;
    ++nb; nbok += ok_g;
    ++nb; nbok += ok_k1;
    ++nb; nbok += ok_k2;
  }
  trace.endBlock();
  
#if defined(WITH_EIGEN)
  
  trace.beginBlock ( "Load vol file -> build main digital surface -> II normals -> AT regularization -> save OBJ with colored normals." );
  {
    auto params     = SH3::defaultParameters() | SHG3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_2_12s]
    auto al_capone  = SH3::makeBinaryImage( examplesPath + "samples/Al.100.vol", params );
    auto K          = SH3::getKSpace( al_capone );
    auto surface    = SH3::makeLightDigitalSurface( al_capone, K, params );
    auto surfels    = SH3::getSurfelRange( surface, params );
    auto ii_normals = SHG3::getIINormalVectors( al_capone, surfels, params );
    auto linels     = SH3::getCellRange( surface, 1 );
    auto uembedder  = SH3::getCellEmbedder( K );
    SH3::Scalars features( linels.size() );
    auto at_normals = SHG3::getATVectorFieldApproximation( features, linels.cbegin(), linels.cend(),
                                                           surface, surfels,
                                                           ii_normals, params );
    // Output normals as colors depending on directions
    SH3::Colors colors( surfels.size() );
    for ( size_t i = 0; i < surfels.size(); i++ ) 
      colors[ i ] = SH3::Color( (unsigned char) 255.0*fabs( at_normals[ i ][ 0 ] ),
                                (unsigned char) 255.0*fabs( at_normals[ i ][ 1 ] ),
                                (unsigned char) 255.0*fabs( at_normals[ i ][ 2 ] ) );
    bool ok1 = SH3::saveOBJ( surface, SH3::RealVectors(), SH3::Colors(), "al-surface.obj" );
    bool ok2 = SH3::saveOBJ( surface, at_normals, colors, "al-colored-at-normals.obj" );
    // Output discontinuities as sticks on linels.
    SH3::RealPoints  f0;
    SH3::RealVectors f1;
    for ( size_t i = 0; i < linels.size(); i++ )
      {
        if ( features[ i ] < 0.5 )
          {
            const SH3::Cell linel = linels[ i ];
            const Dimension     d = * K.uDirs( linel );
            const SH3::Cell    p0 = K.uIncident( linel, d, false );
            const SH3::Cell    p1 = K.uIncident( linel, d, true  );
            f0.push_back( uembedder( p0 ) );
            f1.push_back( uembedder( p1 ) - uembedder( p0 ) );
          }
      }
    bool ok3 = SH3::saveVectorFieldOBJ( f0, f1, 0.1, SH3::Colors(),
                                        "al-features.obj",
                                        SH3::Color( 0, 0, 0 ), SH3::Color::Red );
    //! [dgtal_shortcuts_ssec2_2_12s]
    ++nb; nbok += ok1 ? 1 : 0;
    ++nb; nbok += ok2 ? 1 : 0;
    ++nb; nbok += ok3 ? 1 : 0;
  }
  trace.endBlock();
  
#endif // defined(WITH_EIGEN)
  
  trace.info() << nbok << "/" << nb << " passed tests." << std::endl;
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
