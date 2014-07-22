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
 * @file testContourHelper.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2014/06/11
 *
 * Functions for testing class ContourHelper.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtal/geometry/helpers/ContourHelper.h"
#include "DGtal/helpers/StdDefs.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ContourHelper.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testContourHelper()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Test Contour Helper" );
  std::vector<Z2i::Point> aContour; 
  aContour.push_back(Z2i::Point(0,0));
  aContour.push_back(Z2i::Point(10,0));
  aContour.push_back(Z2i::Point(10,4));
  aContour.push_back(Z2i::Point(0,4));
  Z2i::Point midPoint = ContourHelper::getMeanPoint(aContour);   

  std::vector<Z2i::Point> aContour2; 
  aContour2.push_back(Z2i::Point(0,0));
  aContour2.push_back(Z2i::Point(0,10));
  aContour2.push_back(Z2i::Point(5,10));

  nbok += midPoint==Z2i::Point(5,2) ? 1 : 0; 
  nb++;
  nbok += ContourHelper::isCounterClockWise(aContour) ? 1 : 0; 
  nb++;
  nbok += ContourHelper::isCounterClockWise(aContour2) ? 0 : 1; 
  nb++;

  trace.info() << "(" << nbok << "/" << nb << ") "<< std::endl;
  

  
 
  trace.endBlock();
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class ContourHelper" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testContourHelper(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
