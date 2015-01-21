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
 * @file testFuzzySegmentComputer.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2015/01/07
 *
 * Functions for testing class FuzzySegmentComputer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/curves/FuzzySegmentComputer.h"
#include <DGtal/io/boards/Board2D.h>
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class FuzzySegmentComputer.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testFuzzySegmentComputer()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  Board2D aBoard;
  typedef  FuzzySegmentComputer<Z2i::Space, Z2i::RealPoint, int> FuzzySegmentComputer2D;
  trace.beginBlock ( "Testing Fuzzy segment ..." );
  FuzzySegmentComputer<Z2i::Space, Z2i::RealPoint, int> aFuzzySegmentComp;
  std::set<Z2i::RealPoint> aContour;
  aContour.insert(Z2i::RealPoint(0.2, 0.3));
  aContour.insert(Z2i::RealPoint(2.2, 0.1));
  aContour.insert(Z2i::RealPoint(3.1, 0.5));
  aContour.insert(Z2i::RealPoint(1.1, 1.1));
  aContour.insert(Z2i::RealPoint(4.1, -0.4));
  aContour.insert(Z2i::RealPoint(6.1, 0.9));
  aContour.insert(Z2i::RealPoint(8.1, 2.1));
  aContour.insert(Z2i::RealPoint(4.1, 1.1));
  aContour.insert(Z2i::RealPoint(5.1, 4.1));
  aContour.insert(Z2i::RealPoint(10.1, 2.1));  
  aFuzzySegmentComp.init(30.0, aContour.begin());
  for (int i=0; i<aContour.size()-1; i++){ 
    bool res = aFuzzySegmentComp.extendFront();
    if (!res) {
      break;
    }
  }
  
  // Display the segment
  for (FuzzySegmentComputer2D::ConstIterator it = aFuzzySegmentComp.begin(); 
       it != aFuzzySegmentComp.end(); it++){
    aBoard << *it;
  }
  

  // Display convexhull
  std::vector<Z2i::RealPoint> aVect = aFuzzySegmentComp.getConvexHull();
  for (unsigned int i = 0; i < aVect.size(); i++){
    aBoard.drawLine(aVect.at(i)[0], aVect.at(i)[1], 
                    aVect.at((i+1)%aVect.size())[0],
                    aVect.at((i+1)%aVect.size())[1]);    
  } 
  for (FuzzySegmentComputer2D::ConstIterator it = aFuzzySegmentComp.begin(); 
       it != aFuzzySegmentComp.end(); it++){
    aBoard << *it;
  }
  
  
  aBoard.saveEPS("testFuzzySegmentComputer_Convexhull.eps"); 
  trace.info() << aFuzzySegmentComp;
  
  





  nbok += true ? 1 : 0; 
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
  trace.beginBlock ( "Testing class FuzzySegmentComputer" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testFuzzySegmentComputer(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
