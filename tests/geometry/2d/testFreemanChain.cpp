/**
 * @file testFreemanChain.cpp
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2010/07/05
 *
 * Functions for testing class FreemanChain.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <sstream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/2d/FreemanChain.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class FreemanChain.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 * @ingroup Tests
 */
bool testFreemanChain(stringstream & ss)
{
  unsigned int nbok = 0;
  unsigned int nb = 4;
  
  
  trace.beginBlock ( "Testing FreemanChain " );
  
  FreemanChain::FreemanChain fc;
  FreemanChain::read(ss, fc);
  nbok += 1;   
  trace.info()<< "Freeman chain set to " << ss.str() << endl; 
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Reading FreemanChain" << std::endl;
  
  
  trace.info() << "isClosed():" << fc.isClosed()<< endl;
  nbok += 1;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Test is Closed" << std::endl;
  
  
  int minX, maxX, minY, maxY;
  fc.computeBoundingBox(minX, minY, maxX, maxY);  
  trace.info()<< "Freeman chain bounding box: " << minX << " " << minY << " " << maxX << " " << maxY << endl ; 
  nbok += 1;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Test chain bounding box" << std::endl;
  
  
  vector<FreemanChain::PointI2D> aContourPointVector; 
  fc.getContourPoints(fc, aContourPointVector);
  trace.info() << "List of point: ";
  for (int i =0; i <aContourPointVector.size(); i++){
    trace.info()<< "(" << aContourPointVector.at(i).at(0) << "," << aContourPointVector.at(i).at(1) << ")";
  }
  trace.info()<< endl;
  nbok+=1;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Test extracting list of contour point" << std::endl;
    
  
  
  
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class FreemanChain" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;
  std::stringstream ss (stringstream::in | stringstream::out);
  
  //  std::istream aStream;
  ss << "0 0 00001111222233" << endl;
  bool res = testFreemanChain(ss); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
