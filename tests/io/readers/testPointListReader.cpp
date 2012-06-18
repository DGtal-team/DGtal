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
 * @file testPointListReader.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/04/01
 *
 * Functions for testing class PointListReader.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/readers/PointListReader.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/curves/FreemanChain.h" 

#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class PointListReader.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testPointListReader()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing reading point list ..." );
  
  std::string filename = testPath + "samples/pointList1.pl";
  std::vector<unsigned int> vectPos;
  vectPos.push_back(1);
  vectPos.push_back(2);
  vector<Z2i::Point> vectPoints = PointListReader<Z2i::Point>::getPointsFromFile(filename,
                     vectPos);
  for(unsigned int k=0;k < vectPoints.size(); k++){
    trace.info() << " pt: "<< vectPoints.at(k)<< endl;
  }
  nbok += (vectPoints.size()==4) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "<< std::endl;
  trace.endBlock();
  trace.beginBlock ( "Testing reading point list ..." );
  std::string filenameFC = testPath + "samples/freemanChainSample.fc";
  std::vector< FreemanChain< int > > vectFC = PointListReader< Z2i::Point>:: getFreemanChainsFromFile<int> (filenameFC); 
  for(unsigned int i=0; i< vectFC.size(); i++){
    FreemanChain<int> fc = vectFC.at(i);
    trace.info() << "Freeman chain " << i << ": " << fc.x0 << " " << fc.y0 << " " << fc.chain << endl;
  }
  nbok += (vectFC.size()==5) ? 1 : 0; 
  nb++;
  trace.endBlock();
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class PointListReader" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;
  
  
  bool res = testPointListReader(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
