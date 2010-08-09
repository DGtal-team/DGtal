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
#include "DGtal/io/readers/VolReader.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class VolReader.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testVolReader()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing VolReader ..." );

  typedef SpaceND<int,3> Space4Type;
  typedef HyperRectDomain<Space4Type> TDomain;
  typedef TDomain::Point Point;
  
  //Default image selector = STLVector
  typedef ImageSelector<TDomain, int>::Type Image;
  
  VolReader<Image> reader;
  Image image = reader.importVol("/home/dcoeurjo/Volumes/cat10.vol");
  
  trace.info() << image <<endl;
  
  nbok += true ? 1 : 0; 
  nb++;

  unsigned int nbval=0;
  for(Image::ConstIterator it=image.begin(), itend=image.end();
      it != itend;   ++it)
    if ( image(it) != 0)
      nbval++;
  
  trace.info() << "Number of points with (val!=0)  = "<<nbval<<endl;

  nbok += ( nbval == 8043)  ? 1 : 0; 
  nb++;

  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class VolReader" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testVolReader(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
