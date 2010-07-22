/**
 * @file testColorMaps.cpp
 * @ingroup Tests
 * @author Sebastien Fourey (\c Sebastien.Fourey@greyc.ensicaen.fr )
 * Groupe de Recherche en Informatique, Image, Automatique et Instrumentation de Caen - GREYC (CNRS, UMR 6072), ENSICAEN, France
 *
 * @date 2010/07/16
 *
 * Functions for testing the ColorMap classes.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/colormaps/GrayscaleColorMap.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/io/colormaps/ColorBrightnessColorMap.h"
#include "DGtal/io/colormaps/CyclicHueColorMap.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/colormaps/ColorMapInverter.h"
#include "Board/PSFonts.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace LibBoard;
  
///////////////////////////////////////////////////////////////////////////////
// Function template for testing ColorMap classes. 
///////////////////////////////////////////////////////////////////////////////
template <typename TColorMap>
void addColorMapSample( const char * name,
			const TColorMap & aColorMap, 
			const typename TColorMap::ValueType step,
			Board & board )
{
  typedef typename TColorMap::ValueType ValueType;
  board.translate( 0, 15 );
  for ( ValueType x = aColorMap.min(); x <= aColorMap.max(); x += step ) {
    board.setPenColor(Color::Black);
    board.setFont( LibBoard::Fonts::Courier, 12 );
    board.drawText( -150, 0, name );
    board.setPenColor(Color::None);
    board.setFillColor( aColorMap( x ) );
    board.drawRectangle( static_cast<double>( x ),
			 10,
			 static_cast<double>( step ),
			 10 );
  }
} 

bool testGrayscaleColorMap()
{
  unsigned int nbok = 0;
  unsigned int nb = 6;

  trace.beginBlock("Colormap 0..255");
  { 
    GrayscaleColorMap<unsigned char> cmap(0,255);
    
    Color c0 = cmap(0);
    trace.info(); 
    cerr << int(c0.red())
	 << "," << int(c0.green()) << "," << int(c0.blue()) << std::endl;
    nbok += ( c0 == Color::Black );
    
    Color c128 = cmap(128);
    trace.info(); 
    cerr << int(c128.red())
	 << "," << int(c128.green()) << "," << int(c128.blue()) << std::endl;
    nbok += ( c128 == Color(128,128,128) );
    
    Color c255 = cmap(255);
    trace.info();
    cerr << int(c255.red())
	 << "," << int(c255.green()) << "," << int(c255.blue()) << std::endl;
    nbok += ( c255 == Color::White );
  }
  trace.endBlock();
  
  trace.beginBlock("Colormap 64..128");
  {
    GrayscaleColorMap<unsigned char> cmap(64,128);
    Color c0 = cmap(64);
    trace.info();
    cerr << int(c0.red())
	 << "," << int(c0.green()) << "," << int(c0.blue()) << std::endl;
    nbok += ( c0 == Color::Black );

    Color c255 = cmap(128);
    trace.info(); 
    cerr << int(c255.red())
	 << "," << int(c255.green()) << "," << int(c255.blue()) << std::endl;
    nbok += ( c255 == Color::White );    
  }
  trace.endBlock();

  trace.beginBlock("Static method");
  {
    Color c = GrayscaleColorMap<unsigned char>::getColor(0,128,128);
    trace.info() << "Should be white: ";
    cerr << int(c.red())
	 << "," << int(c.green()) << "," << int(c.blue()) << std::endl;
    nbok += (c == Color::White);

    c = GrayscaleColorMap<unsigned char>::getColor(0,128,64);
    trace.info() << "Should be around 127,127,127: ";
    cerr << int(c.red())
	 << "," << int(c.green()) << "," << int(c.blue()) << std::endl;

    trace.endBlock();
  }
  return nbok == nb;
}

int main( int argc, char** argv )
{
  Board board;
  trace.beginBlock ( "Testing GrayscaleColorMap" );
  bool res1 = testGrayscaleColorMap();
  trace.emphase() << ( res1 ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  
  GrayscaleColorMap<int> cmap_gray( 0, 500);
  addColorMapSample( "Grayscale", cmap_gray, 1, board );

  HueShadeColorMap<int> cmap_hsv( 0, 500);
  addColorMapSample( "HueShade", cmap_hsv, 1, board );

  ColorBrightnessColorMap<int> cmap_red( 0, 500, Color::Red );
  addColorMapSample( "Brightness", cmap_red, 1, board );

  CyclicHueColorMap<int> cmap_cyclic5( 0, 500 );
  addColorMapSample( "CyclicHue (5x)", cmap_cyclic5, 1, board );

  CyclicHueColorMap<int> cmap_cyclic10( 0, 500, 10 );
  addColorMapSample( "CyclicHue (10x)", cmap_cyclic10, 1, board );

  GradientColorMap<int> cmap_gradient( 0, 500, Color::Yellow, Color::Red );
  addColorMapSample( "Gradient (Y->R)", cmap_gradient, 1, board );

  GradientColorMap<int> cmap_grad3( 0, 500 );
  cmap_grad3.addColor( Color::Blue );
  cmap_grad3.addColor( Color::White );
  cmap_grad3.addColor( Color::Red );
  addColorMapSample( "Gradient (B->W->R)", cmap_grad3, 1, board );

  cmap_grad3.clearColors();
  cmap_grad3.addColor( Color::Green );
  cmap_grad3.addColor( Color::Yellow );
  cmap_grad3.addColor( Color::Red );
  addColorMapSample( "Gradient (G->Y->R)", cmap_grad3, 1, board );

  GradientColorMap<int> cool_gradient( 0, 500, GradientColorMap<int>::Cool );
  addColorMapSample( "Gradient (Cool)", cool_gradient, 1, board );

  GradientColorMap<int> copper_gradient( 0, 500, GradientColorMap<int>::Copper );
  addColorMapSample( "Gradient (Copper)", copper_gradient, 1, board );

  GradientColorMap<int> hot_gradient( 0, 500, GradientColorMap<int>::Hot );
  addColorMapSample( "Gradient (Hot)", hot_gradient, 1, board );

  GradientColorMap<int> jet_gradient( 0, 500, GradientColorMap<int>::Jet );

  addColorMapSample( "Gradient (Jet)", jet_gradient, 1, board );

  addColorMapSample( "Gradient (Spring)",
		     GradientColorMap<int>( 0, 500, GradientColorMap<int>::Spring ),
		     1,
		     board );
  addColorMapSample( "Gradient (Summer)",
		     GradientColorMap<int>( 0, 500, GradientColorMap<int>::Summer ),
		     1,
		     board );
  addColorMapSample( "Gradient (Autumn)",
		     GradientColorMap<int>( 0, 500, GradientColorMap<int>::Autumn ),
		     1,
		     board );
  addColorMapSample( "Gradient (Winter)",
		     GradientColorMap<int>( 0, 500, GradientColorMap<int>::Winter ),
		     1,
		     board );

  typedef ColorBrightnessColorMap<int> BrightnessColorMapInt;
  BrightnessColorMapInt cmap_green( 0, 500, Color::Green);
  ColorMapInverter<BrightnessColorMapInt> inverted(cmap_green);
  addColorMapSample( "Brightness (Green)", cmap_green, 1, board );
  addColorMapSample( "Inverted Brightness", inverted, 1, board );

  board.saveEPS( "colormaps.eps" );

  return ( res1 ) ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
