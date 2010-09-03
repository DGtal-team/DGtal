/**
 * @file testVolReader.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/25
 *
 * Functions for testing class VolReader.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/images/ImageSelector.h"
#include "DGtal/io/writers/GenericWriter.h"

#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class Generic.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testGenericWriter()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing Generic ..." );

  typedef SpaceND<int,2> TSpace;
  typedef TSpace::Point Point;
  typedef TSpace::Vector Vector;
  typedef HyperRectDomain<TSpace> Domain;
 
  Point a ( 1, 1);
  Point b ( 16, 16);
 
  typedef ImageSelector<Domain, unsigned char>::Type Image;
  Image image(a,b);
  for(unsigned int i=0 ; i < 256; i++)
    image[i] = i;

  GenericWriter<Image>::exportTXT("export-generic.txt",image);
  GenericWriter<Image>::exportBIN("export-generic.bin",image);
  GenericWriter<Image>::exportXML("export-generic.xml",image);
 

  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class Generic" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testGenericWriter(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
