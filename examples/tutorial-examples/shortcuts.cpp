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
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/io/viewers/Viewer3D.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main( int /* argc */, char** /* argv */ )
{
  unsigned int nb = 0, nbok = 0;
  // Using standard 2D digital space.
  typedef Shortcuts<Z2i::KSpace> SH2;
  // Using standard 3D digital space.
  typedef Shortcuts<Z3i::KSpace> SH3;

  // 3d tests
  trace.beginBlock ( "Load vol file -> noisify -> save as vol file." );
  {
    auto params    = SH3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_1_1s]
    // load and noisify image directly.
    auto al_capone = SH3::makeBinaryImage( examplesPath + "samples/Al.100.vol",
					   params( "noise", 0.3 ) );
    auto ok        = SH3::saveBinaryImage( al_capone, "noisy-Al.vol" );
    //! [dgtal_shortcuts_ssec2_1_1s]
    ++nb, nbok += ok ? 1 : 0; 
  }
  trace.endBlock();
  trace.beginBlock ( "Load vol file -> build main connected digital surface." );
  {
    auto params    = SH3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_1_2s]
    auto al_capone = SH3::makeBinaryImage( examplesPath + "samples/Al.100.vol", params );
    auto K         = SH3::getKSpace( al_capone );
    auto surface   = SH3::makeLightDigitalSurface( al_capone, K, params );
    trace.info() << "#surfels=" << surface->size() << std::endl;
    //! [dgtal_shortcuts_ssec2_1_2s]
    ++nb, nbok += surface->size() == 21239 ? 1 : 0; 
  }
  trace.endBlock();

  trace.beginBlock ( "Load vol file -> extract 2 isosurfaces -> build mesh -> displays them." );
  {
    auto params    = SH3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_1_3s]
    params( "faceSubdivision", "Centroid" )( "surfelAdjacency", 1);
    auto gimage    = SH3::makeGrayScaleImage( examplesPath + "samples/lobster.vol" );
    auto trisurf150= SH3::makeTriangulatedSurface( gimage, params( "thresholdMin", 150 ) );
    auto trisurf40 = SH3::makeTriangulatedSurface( gimage, params( "thresholdMin", 40 ) );
    auto mesh150   = SH3::makeMesh( trisurf150 );
    auto mesh40    = SH3::makeMesh( trisurf40 );
    trace.info() << "#mesh150=" << mesh150->nbVertex()
		 << " #mesh40=" << mesh40->nbVertex() << std::endl;
    //! [dgtal_shortcuts_ssec2_1_3s]
    ++nb, nbok += ( mesh150->nbVertex() < mesh40->nbVertex() )
      && ( mesh40->nbVertex() == 273182 ) ? 1 : 0;
    // // If you wish to display them.
    // QApplication application(argc,argv);
    // Viewer3D<> viewer;
    // viewer.show();
    // viewer << CustomColors3D( Color::Black, Color::Red  ) << *mesh40;
    // viewer << CustomColors3D( Color::Black, Color::Blue ) << *mesh150;
    // viewer << Viewer3D<>::updateDisplay;
    // application.exec();
  }
  trace.endBlock();

  trace.beginBlock ( "Load vol file -> extract 2 triangulated isosurfaces -> save as OBJ." );
  {
    auto params    = SH3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_1_4s]
    params( "faceSubdivision", "Centroid" )( "surfelAdjacency", 1);
    auto gimage    = SH3::makeGrayScaleImage( examplesPath + "samples/lobster.vol" );
    auto trisurf150= SH3::makeTriangulatedSurface( gimage, params( "thresholdMin", 150 ) );
    auto trisurf40 = SH3::makeTriangulatedSurface( gimage, params( "thresholdMin", 40 ) );
    auto ok40      = SH3::saveOBJ( trisurf40, SH3::RealVectors(), SH3::Colors(),
				   "lobster-40.obj", // semi-transparent red diffuse color
				   SH3::Color( 30,30,30 ), SH3::Color( 255,0,0,100 ) );
    auto ok150     = SH3::saveOBJ( trisurf150, SH3::RealVectors(), SH3::Colors(),
				   "lobster-150.obj", // opaque blue diffuse color
				   SH3::Color( 30,30,30 ), SH3::Color( 0,0,255,255 ) );
    //! [dgtal_shortcuts_ssec2_1_4s]
    ++nb, nbok += ok40  ? 1 : 0;
    ++nb, nbok += ok150 ? 1 : 0;
  }
  trace.endBlock();

  trace.beginBlock ( "Load vol file -> build main digital surface -> breadth first traversal -> save OBJ with colored distance." );
  {
    auto params    = SH3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_1_5s]
    params( "surfaceTraversal", "BreadthFirst" ) // specifies breadth-first traversal
      ( "colormap", "Jet" ); // specifies the colormap
    auto al_capone = SH3::makeBinaryImage( examplesPath + "samples/Al.100.vol", params );
    auto K         = SH3::getKSpace( al_capone );
    auto surface   = SH3::makeLightDigitalSurface( al_capone, K, params );
    auto surfels   = SH3::getSurfelRange( surface, params );
    auto cmap      = SH3::getColorMap( 0, surfels.size(), params );
    SH3::Colors colors( surfels.size() );
    for ( unsigned int i = 0; i < surfels.size(); ++i ) colors[ i ] = cmap( i );
    bool ok        = SH3::saveOBJ( surface, SH3::RealVectors(), colors, "al-primal-bft.obj" );
    //! [dgtal_shortcuts_ssec2_1_5s]
    ++nb, nbok += ok ? 1 : 0; 
  }
  trace.endBlock();

  trace.beginBlock ( "Build polynomial shape -> digitize -> noisify -> save as vol file." );
  {
    auto params          = SH3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_2_1s]
    params( "polynomial", "3*x^2+2*y^2+z^2-90" )( "gridstep", 0.25 ) 
      ( "noise", 0.3 );
    auto implicit_shape  = SH3::makeImplicitShape3D( params );
    auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
    auto noisy_shape     = SH3::makeBinaryImage    ( digitized_shape, params );
    auto ok              = SH3::saveBinaryImage    ( noisy_shape, "noisy-ellipsoid.vol" );
    //! [dgtal_shortcuts_ssec2_2_1s]
    ++nb, nbok += ok ? 1 : 0;
  }
  trace.endBlock();

  trace.beginBlock ( "Build polynomial shape -> digitize -> build digital surface -> save primal surface as obj." );
  {
    auto params          = SH3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_2_2s]
    params( "polynomial", "goursat" )( "gridstep", 0.25 );
    auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
    auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
    auto K               = SH3::getKSpace( params );
    auto binary_image    = SH3::makeBinaryImage( digitized_shape, params );
    auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
    bool ok              = SH3::saveOBJ( surface, "goursat-primal.obj" );
    //! [dgtal_shortcuts_ssec2_2_2s]
    ++nb, nbok += ok ? 1 : 0;
  }
  trace.endBlock();

  trace.beginBlock ( "Build polynomial shape -> digitize -> build indexed surface on a subpart." );
  {
    auto params          = SH3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_2_3s]
    params( "polynomial", "leopold" )( "gridstep", 0.25 )
      ( "minAABB", -12.0 )( "maxAABB", 12.0 )
      ( "surfaceComponents", "All" );
    auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
    auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
    auto Kwhole          = SH3::getKSpace( params );
    auto K               = SH3::getKSpace( SH3::Point::zero, Kwhole.upperBound(), params );
    auto binary_image    = SH3::makeBinaryImage( digitized_shape, 
						 SH3::Domain(K.lowerBound(),K.upperBound()),
						 params );
    auto surface         = SH3::makeIdxDigitalSurface( binary_image, K, params );
    trace.info() << "#surfels=" << surface->size() << std::endl;
    //! [dgtal_shortcuts_ssec2_2_3s]
    ++nb, nbok += surface->size() > 1000 ? 1 : 0;
  }
  trace.endBlock();

  trace.beginBlock ( "Build polynomial shape -> digitize -> noisify -> count components -> save OBJ with different colors." );
  {
    auto params          = SH3::defaultParameters();
    //! [dgtal_shortcuts_ssec2_2_4s]
    params( "polynomial", "leopold" )( "gridstep", 0.25 )
      ( "minAABB", -12.0 )( "maxAABB", 12.0 )
      ( "surfaceComponents", "All" )( "noise", 0.5 );
    auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
    auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
    auto K               = SH3::getKSpace( params );
    auto binary_image    = SH3::makeBinaryImage(digitized_shape,
						SH3::Domain(K.lowerBound(),K.upperBound()),
						params );
    // Extracts the whole surface (with all components)
    auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
    // Extracts a vector of connected surfaces.
    auto vec_surfs       = SH3::makeLightDigitalSurfaces( binary_image, K, params );
    trace.info() << "#connected components = " << vec_surfs.size() << std::endl;
    std::map< SH3::Surfel, unsigned int> label;
    unsigned int n = 0;
    for ( auto&& surf : vec_surfs ) {
      auto surfels = SH3::getSurfelRange( surf, params );
      for ( auto&& s : surfels ) label[ s ] = n;
      n += 1;
    }
    auto cmap        = SH3::getColorMap( 0, vec_surfs.size(), params );
    auto all_surfels = SH3::getSurfelRange( surface, params );
    SH3::Colors colors( all_surfels.size() );
    for ( unsigned int i = 0; i < all_surfels.size(); ++i )
      colors[ i ] = cmap( label[ all_surfels[ i ] ] );
    bool ok = SH3::saveOBJ( surface, SH3::RealVectors(), colors, "leopold-primal-cc.obj" );
    //! [dgtal_shortcuts_ssec2_2_4s]
    ++nb, nbok += ok ? 1 : 0;
  }
  trace.endBlock();
  
  trace.info() << nbok << "/" << nb << " passed tests." << std::endl;
  
  // trace.beginBlock ( "Set parameters" );
  // auto params    = SH3::defaultParameters();
  // // Set your own parameters with operator().
  // params( "polynomial", "3*x^2+2*y^2+z^2-90" )
  //   ( "gridstep", 0.5 )
  //   ( "noise",    0.2 )
  //   ( "surfaceComponents", "All" )
  //   ( "surfelAdjacency",   0 );
  // params( "faceSubdivision", "Centroid" );
  // std::cout << params << std::endl;
  // trace.endBlock();
  // trace.beginBlock ( "Making implicit shape" );
  // auto implicit_shape = SH3::makeImplicitShape3D( params );
  // std::cout << *implicit_shape << std::endl;
  // trace.endBlock();
  // trace.beginBlock ( "Making Khalimsky space" );
  // auto K = SH3::getKSpace( params );
  // std::cout << K << std::endl;
  // trace.endBlock();
  // trace.beginBlock ( "Making implicit digital shape" );
  // auto digital_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  // std::cout << *digital_shape << std::endl;
  // trace.endBlock();
  // trace.beginBlock ( "Making binary image from implicit digital shape" );
  // auto binary_image = SH3::makeBinaryImage( digital_shape, params );
  // std::cout << *binary_image << std::endl;
  // trace.endBlock();
  // trace.beginBlock ( "Save binary image into file" );
  // auto ok = SH3::saveBinaryImage( binary_image, "dummy.vol" );
  // std::cout << ( ok ? "dummy.vol OK" : "dummy.vol ERROR" ) << std::endl;
  // trace.endBlock();
  // trace.beginBlock ( "Making binary image from vol file" );
  // auto al_capone = SH3::makeBinaryImage( examplesPath + "samples/Al.100.vol", params );
  // std::cout << *al_capone << std::endl;
  // auto ok2 = SH3::saveBinaryImage( al_capone, "dummy2.vol" );
  // std::cout << ( ok ? "dummy2.vol OK" : "dummy2.vol ERROR" ) << std::endl;
  // trace.endBlock();
  // trace.beginBlock ( "Making digital surface" );
  // auto Kal         = SH3::getKSpace( al_capone, params );
  // auto light_surf = SH3::makeLightDigitalSurface( al_capone, Kal, params );
  // std::cout << "#surfels = " << light_surf->size() << std::endl;
  // std::vector< std::string > traversals { "Default", "DepthFirst", "BreadthFirst" };
  // for ( auto&& mode : traversals ) {
  //   auto surfels = SH3::getSurfelRange( light_surf, params( "surfaceTraversal", mode ) );
  //   double distance  = 0.0;
  //   for ( unsigned int i = 1; i < surfels.size(); ++i )
  //     distance += ( K.sCoords( surfels[ i-1 ] ) - K.sCoords( surfels[ i ] ) ).norm();
  //   std::cout << "avg " << mode << " distance = " << distance / (surfels.size()-1.0) << std::endl;
  // }
  // trace.endBlock();
  // trace.beginBlock ( "Making all light digital surfaces" );
  // auto vec_surfs   = SH3::makeLightDigitalSurfaces( al_capone, Kal, params );
  // std::cout << "#connected components        = " << vec_surfs.size() << std::endl;
  // unsigned int nb_small = 0;
  // unsigned int nb_big = 0;
  // for ( auto&& surf : vec_surfs )
  //   {
  //     unsigned int n = surf->size();
  //     nb_small += n <  100 ? 1 : 0;
  //     nb_big   += n >= 100 ? 1 : 0;
  //   }
  // std::cout << "#connected components <  100 = " << nb_small << std::endl;
  // std::cout << "#connected components >= 100 = " << nb_big << std::endl;
  // trace.endBlock();
  // trace.beginBlock ( "Make triangulated surface from digital surface" );
  // {
  //   SH3::Surfel2Index s2i;
  //   auto trisurf = SH3::makeTriangulatedSurface( s2i, light_surf );
  //   std::cout << "trisurf=" << *trisurf << std::endl;
  // }
  // trace.endBlock();
  // trace.beginBlock ( "Load a vol file as a float image" );
  // {
  //   auto gimage   = SH3::makeFloatImage( examplesPath + "samples/lobster.vol" );
  //   auto m        = * std::min_element( gimage->begin(), gimage->end() );
  //   auto M        = * std::max_element( gimage->begin(), gimage->end() );
  //   float     avg = 0.0;
  //   int        nb = 0;
  //   std::for_each( gimage->begin(), gimage->end(),
  // 		   [&avg,&nb] (float x) { avg += x; nb += 1; } );
  //   avg          /= nb;
  //   std::cout << "min=" << m << " avg=" << avg << " max=" << M << std::endl;
  // }
  // trace.endBlock();
  // trace.beginBlock ( "Make marching-cubes triangulated surface from implicit shape 3D" );
  // {
  //   params( "polynomial", "goursat" )( "gridstep", 0.25 )
  // 	( "thresholdMin", 128 )    ( "noise",    0.0 );
  //   auto ishape   = SH3::makeImplicitShape3D( params );
  //   auto fimage   = SH3::makeFloatImage     ( ishape, params );
  //   auto gimage   = SH3::makeGrayScaleImage ( fimage, params );
  //   auto trisurf  = SH3::makeTriangulatedSurface( gimage, params );
  //   bool ok       = SH3::saveOBJ            ( trisurf, "goursat.obj" );
  //   std::cout << *ishape << std::endl;
  //   std::cout << "trisurf =" << *trisurf << std::endl;
  //   auto K        = SH3::getKSpace( gimage, params );
  //   auto bimage   = SH3::makeBinaryImage ( gimage, params );
  //   auto idx_surf = SH3::makeIdxDigitalSurface( bimage, K,
  // 						params( "surfaceComponents", "All" ) );
  //   auto primal   = SH3::makePrimalPolygonalSurface( idx_surf );
  //   bool ok2      = SH3::saveOBJ            ( primal, "goursat-primal.obj" );
  // }
  // trace.endBlock();
  
  // trace.beginBlock ( "View marching-cubes surface from gray-scale image" );
  // {
  //   auto gimage   = SH3::makeGrayScaleImage( examplesPath + "samples/lobster.vol" );
  //   auto params   = SH3::defaultParameters();
  //   params( "faceSubdivision", "Centroid" )( "surfelAdjacency", 1);
  //   auto polysurf = SH3::makePolygonalSurface( gimage, params( "thresholdMin", 40 ) );
  //   std::cout << "polysurf=" << *polysurf << std::endl;
  //   //bool ok       = SH3::saveOBJ( polysurf, "lobster-40.obj" );
  //   auto trisurf  = SH3::makeTriangulatedSurface( gimage, params( "thresholdMin", 20 ) );
  //   std::cout << "trisurf =" << *trisurf << std::endl;
  //   bool ok2      = SH3::saveOBJ( trisurf, "lobster-20-tri.obj" );
  //   auto trisurf2 = SH3::makeTriangulatedSurface( polysurf, params );
  //   std::cout << "trisurf2=" << *trisurf2 << std::endl;
  //   bool ok3      = SH3::saveOBJ( trisurf2, "lobster-40-tri.obj" );
  // }
  // trace.endBlock();
  
  // trace.beginBlock ( "Save digital surface as .obj file" );
  // {
  //   ofstream objfile( "primal-al.obj" );
  //   bool ok = SH3::outputPrimalDigitalSurfaceAsObj( objfile, light_surf );
  //   std::cout << "- saving as primal-al.obj: " << ( ok ? "OK" : "ERROR" ) << std::endl;
  // }
  // {
  //   ofstream objfile( "dual-al.obj" );
  //   bool ok = SH3::outputDualDigitalSurfaceAsObj
  //     ( objfile, light_surf, params );
  //   std::cout << "- saving as dual-al.obj: " << ( ok ? "OK" : "ERROR" ) << std::endl;
  // }
  // trace.endBlock();
  // trace.beginBlock ( "Making indexed digital surface" );
  // {
  //   auto idx_surf    = SH3::makeIdxDigitalSurface
  //     ( al_capone, Kal, params( "surfaceComponents", "All" ) );
  //   auto positions   = idx_surf->positions();
  //   std::cout << "#surfels = " << idx_surf->size() << std::endl;
  //   for ( auto&& mode : traversals ) {
  //     auto surfels = SH3::getIdxSurfelRange( idx_surf, params( "surfaceTraversal", mode ) );
  //     double distance  = 0.0;
  //     for ( unsigned int i = 1; i < surfels.size(); ++i ) 
  // 	distance += ( positions[ surfels[ i-1 ] ] - positions[ surfels[ i ] ] ).norm();
  //     std::cout << "avg " << mode << " distance = " << distance / (surfels.size()-1.0) << std::endl;
  //   }
  //   auto poly_surf   = SH3::makeDualPolygonalSurface( idx_surf );
  //   std::cout << "polysurf = " << *poly_surf << std::endl;
  //   bool      okd    = SH3::saveOBJ( poly_surf, "al-idx-dual.obj" );
  // }
  // trace.endBlock();
  // trace.beginBlock ( "Save indexed-digital surface as .obj file" );
  // auto new_idx_surf = SH3::makeIdxDigitalSurface( vec_surfs );
  // {
  //   ofstream objfile( "primal-idx-al.obj" );
  //   bool ok = SH3::outputPrimalIdxDigitalSurfaceAsObj( objfile, new_idx_surf );
  //   std::cout << "- saving as primal-idx-al.obj: " << ( ok ? "OK" : "ERROR" ) << std::endl;
  // }
  // trace.endBlock();
  
  // // 2d tests
  // {
  //   typedef Shortcuts< Z2i::KSpace > SH2;
  //   auto params = SH2::defaultParameters();
  //   trace.beginBlock ( "Load and threshold gray-scale image" );
  //   auto gl_image = SH2::makeGrayScaleImage( examplesPath + "samples/contourS.pgm" );
  //   auto b_image  = SH2::makeBinaryImage( gl_image, params( "thresholdMin", 128 ) );
  //   auto ok       = SH2::saveBinaryImage( b_image, "dummy3.pgm" );
  //   std::cout << *gl_image << std::endl;
  //   trace.endBlock();
  // } 
  // // debug
  // {
  //   using namespace Z3i;
  //   typedef Shortcuts< KSpace > SH3;
  //   trace.beginBlock ( "Setting parameters" );
  //   auto params = SH3::defaultParameters();
  //   params( "faceSubdivision", "Centroid" );
  //   Domain domain( Point::diagonal(-1), Point::diagonal(2) );
  //   auto b_image = SH3::makeBinaryImage( domain );
  //   auto K       = SH3::getKSpace( b_image, params );
  //   b_image->setValue( Point( 0, 0, 0 ), true );
  //   b_image->setValue( Point( 0, 0, 1 ), true );
  //   b_image->setValue( Point( 0, 1, 1 ), true );
  //   b_image->setValue( Point( 1, 1, 1 ), true );
  //   auto light_surf = SH3::makeLightDigitalSurfaces( b_image, K, params )[ 0 ];
  //   std::cout << light_surf << std::endl;
  //   {
  //     ofstream objfile( "primal-test.obj" );
  //     bool ok = SH3::outputPrimalDigitalSurfaceAsObj( objfile, light_surf );
  //     std::cout << "- saving as primal-test.obj: " << ( ok ? "OK" : "ERROR" ) << std::endl;
  //   }
  //   {
  //     ofstream objfile( "dual-test.obj" );
  //     bool ok = SH3::outputDualDigitalSurfaceAsObj( objfile, light_surf, params );
  //     std::cout << "- saving as dual-test.obj: " << ( ok ? "OK" : "ERROR" ) << std::endl;
  //   }
  //   trace.endBlock();
  // }
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
