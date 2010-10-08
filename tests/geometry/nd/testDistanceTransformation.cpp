/**
 * @file testDistanceTransformation.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/09/30
 *
 * Functions for testing class DistanceTransformation.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"


#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/images/ImageSelector.h"
#include "DGtal/geometry/nd/volumetric/SeparableMetricTraits.h"
#include "DGtal/geometry/nd/volumetric/DistanceTransformation.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/io/colormaps/GrayscaleColorMap.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DistanceTransformation.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testDistanceTransformation()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock("Testing the whole DT computation");
  
  typedef SpaceND<2> TSpace;
  typedef TSpace::Point Point;
  typedef HyperRectDomain<TSpace> Domain;
  typedef HueShadeColorMap<unsigned char,2> HueTwice;
  typedef GrayscaleColorMap<unsigned char> Gray;
  Point a ( 0, 0);
  Point b ( 15, 15);
  typedef ImageSelector<Domain, unsigned int>::Type Image;
  Image image(a,b);
  
  for(unsigned k=0; k < 49; k++)
    {
      a[0] = (k/7) +5;
      a[1] = (k%7) +5;
      image.setValue(a,128);
    }

  typedef ImageSelector<Domain, long int>::Type ImageLong;
  
  typedef SeparableMetricTraits<unsigned int, unsigned int, 2> L_2;

  DistanceTransformation<Image,ImageLong,L_2> dt;
  
  dt.checkTypesValidity(image);

  LibBoard::Board board;
  board.setUnit(LibBoard::Board::UCentimeter);
  image.selfDraw<Gray>(board,0,255);
  board.saveSVG( "image-preDT.svg" );
  //We just iterate on the Domain points and print out the point coordinates.
  std::copy ( image.begin(),
	      image.end(),
	      std::ostream_iterator<unsigned int> ( std::cout, " " ) );



  ImageLong result = dt.compute(image);
  trace.warning() << result<<endl;
   //We just iterate on the Domain points and print out the point coordinates.
  std::copy ( result.begin(),
	      result.end(),
	      std::ostream_iterator<unsigned int> ( std::cout, " " ) );

  board.clear();
  result.selfDraw<Gray>(board,0,16);
  board.saveSVG( "image-postDT.svg" );


  trace.info() << result <<endl;
  
  trace.endBlock();

  return nbok == nb;
}

/**
 * Example of a test. To be completed.
 *
 */
bool testTypeValidity()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock("Testing type checker");
  
  typedef SpaceND<2> TSpace;
  typedef TSpace::Point Point;
  typedef HyperRectDomain<TSpace> Domain;
  
  Point a ( 0, 0);
  Point b ( 15, 15);
  typedef ImageSelector<Domain, unsigned int>::Type Image;
  Image image(a,b);
  typedef ImageSelector<Domain, long int>::Type ImageLong;
  ImageLong *result;

  typedef SeparableMetricTraits<unsigned int,  unsigned int, 2> L_2;
  DistanceTransformation<Image,ImageLong,L_2> dt;
  
  //No problem should be reported on the std:cerr.
  dt.checkTypesValidity(image);

  typedef SeparableMetricTraits<unsigned int, char, 34> L_34;
  DistanceTransformation<Image,ImageLong,L_34> dt34;

  //Type problem should be reported.
  dt34.checkTypesValidity(image);

  trace.endBlock();
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class DistanceTransformation" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testDistanceTransformation() && testTypeValidity(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
