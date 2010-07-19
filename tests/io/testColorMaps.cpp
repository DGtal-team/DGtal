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

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace LibBoard;
  
///////////////////////////////////////////////////////////////////////////////
// Function for testing class GrayscaleColorMap.
///////////////////////////////////////////////////////////////////////////////

template <typename ColorMap, typename ValueType>
class ColorMapSampler {
public:  
  ColorMapSampler( ColorMap colorMap, ValueType step )
    : myColorMap( colorMap ), myStep( step )
  { 
  }
  void write( const char * filename ) const
  {
    Board b;
    b.setPenColor(Color::None);
    for ( ValueType x = myColorMap.min(); x < myColorMap.max(); x += myStep ) {
      b.setFillColor( myColorMap( x ) );
      b.drawRectangle( static_cast<double>( x ),
		       -100,
		       static_cast<double>( myStep ),
		       10 );
    }
    b.saveEPS( filename );
  } 
private:
  ColorMap myColorMap;
  ValueType myStep;
};


/**
 * Example of a test. To be completed.
 *
 */
bool testGrayscaleColorMap()
{
  unsigned int nbok = 0;
  unsigned int nb = 6;

  trace.beginBlock("Colormap 0..255");
  { 
    GrayscaleColorMap<unsigned char> cmap(0,255);
    
    Color c0 = cmap(0);
    trace.info(); cerr << int(c0.red()) << "," << int(c0.green()) << "," << int(c0.blue()) << std::endl;
    nbok += ( c0 == Color::Black );
    
    Color c128 = cmap(128);
    trace.info(); cerr << int(c128.red()) << "," << int(c128.green()) << "," << int(c128.blue()) << std::endl;
    nbok += ( c128 == Color(128,128,128) );
    
    Color c255 = cmap(255);
    trace.info(); cerr << int(c255.red()) << "," << int(c255.green()) << "," << int(c255.blue()) << std::endl;
    nbok += ( c255 == Color::White );
  }
  trace.endBlock();
  
  trace.beginBlock("Colormap 64..128");
  {
    GrayscaleColorMap<unsigned char> cmap(64,128);
    Color c0 = cmap(64);
    trace.info(); cerr << int(c0.red()) << "," << int(c0.green()) << "," << int(c0.blue()) << std::endl;
    nbok += ( c0 == Color::Black );

    Color c255 = cmap(128);
    trace.info(); cerr << int(c255.red()) << "," << int(c255.green()) << "," << int(c255.blue()) << std::endl;
    nbok += ( c255 == Color::White );    
  }
  trace.endBlock();

  trace.beginBlock("Static method");
  {
    Color c = GrayscaleColorMap<unsigned char>::getColor(0,128,128);
    trace.info() << "Should be white: ";
    cerr << int(c.red()) << "," << int(c.green()) << "," << int(c.blue()) << std::endl;
    nbok += (c == Color::White);

    c = GrayscaleColorMap<unsigned char>::getColor(0,128,64);
    trace.info() << "Should be around 127,127,127: ";
    cerr << int(c.red()) << "," << int(c.green()) << "," << int(c.blue()) << std::endl;

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

  trace.endBlock();

  GrayscaleColorMap<int> cmap_gray(0,500);
  ColorMapSampler< GrayscaleColorMap<int>, int >( cmap_gray, 1 ).write( "gray_colormap.eps" );

  HueShadeColorMap<int> cmap_hsv(0,500);
  ColorMapSampler< HueShadeColorMap<int>, int >( cmap_hsv, 1 ).write( "hsv_colormap.eps" );

  ColorBrightnessColorMap<int> cmap_red(0,500, Color::Red );
  ColorMapSampler< ColorBrightnessColorMap<int>, int >( cmap_red, 1 ).write( "red_colormap.eps" );
  
  return ( res1 ) ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
