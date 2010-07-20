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
#include "DGtal/io/colormaps/ColorMapInverter.h"
#include "DGtal/io/colormaps/CyclicHueColorMap.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace LibBoard;
  
///////////////////////////////////////////////////////////////////////////////
// Function template for testing ColorMap classes. 
///////////////////////////////////////////////////////////////////////////////
template <typename TColorMap>
void writeColorMapSample( const TColorMap & aColorMap, 
			  const typename TColorMap::ValueType step,
			  const char * filename )
{
  typedef typename TColorMap::ValueType ValueType;
  Board b;
  b.setPenColor(Color::None);
  for ( ValueType x = aColorMap.min(); x < aColorMap.max(); x += step ) {
    b.setFillColor( aColorMap( x ) );
    b.drawRectangle( static_cast<double>( x ),
		     -100,
		     static_cast<double>( step ),
		     10 );
  }
  b.saveEPS( filename );
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
  trace.beginBlock ( "Testing GrayscaleColorMap" );
  bool res1 = testGrayscaleColorMap();
  trace.emphase() << ( res1 ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  
  GrayscaleColorMap<int> cmap_gray( 0, 500);
  writeColorMapSample( cmap_gray, 1, "gray_colormap.eps" );

  HueShadeColorMap<int> cmap_hsv( 0, 500);
  writeColorMapSample( cmap_hsv, 1, "hsv_colormap.eps" );

  ColorBrightnessColorMap<int> cmap_red( 0, 500, Color::Red );
  writeColorMapSample( cmap_red, 1, "red_colormap.eps" );

  CyclicHueColorMap<int> cmap_cyclic5( 0, 500 );
  writeColorMapSample( cmap_cyclic5, 1, "cylic5_colormap.eps" );

  CyclicHueColorMap<int> cmap_cyclic10( 0, 500, 10 );
  writeColorMapSample( cmap_cyclic10, 1, "cylic10_colormap.eps" );

  typedef ColorBrightnessColorMap<int> BrightnessColorMapInt;
  BrightnessColorMapInt cmap_green( 0, 500, Color::Green);
  ColorMapInverter<BrightnessColorMapInt> inverted(cmap_green);
  writeColorMapSample( inverted, 1, "green_colormap.eps" );

  return ( res1 ) ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
