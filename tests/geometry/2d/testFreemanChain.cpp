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
//LICENSE-END
/**
 * @file testFreemanChain.cpp
 * @ingroup Tests
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
#include <fstream>
#include <sstream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/2d/FreemanChain.h"
#include "Board/Board.h"
#include <boost/program_options.hpp>
#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace LibBoard;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class FreemanChain.
///////////////////////////////////////////////////////////////////////////////

/**
 * Example of a test. To be completed.
 *
 */
bool testFreemanChain(stringstream & ss)
{
  unsigned int nbok = 0;
  unsigned int nb = 4;
  
  
  trace.beginBlock ( "Testing FreemanChain " );
  
  FreemanChain fc(ss);

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
  
  
  vector<FreemanChain::PointI2> aContourPointVector; 
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




/**
 * Example of a test. To be completed.
 *
 */
bool testDisplayFreemanChain(const string &file)
{
  fstream fst;
  fst.open (file.c_str(), ios::in);
  FreemanChain fc(fst);  
  LibBoard::Board aBoard;
  aBoard.setUnit(Board::UMillimeter);
  fc.selfDraw(aBoard);
  aBoard.saveEPS( "testDisplayFC.eps" );
 
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
  ss << "0 0 00001111222233" << endl;
  bool res = testFreemanChain(ss); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;

  std::string filename = testPath + "samples/klokan.fc";
  testDisplayFreemanChain(filename);
  trace.endBlock();
  
return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
