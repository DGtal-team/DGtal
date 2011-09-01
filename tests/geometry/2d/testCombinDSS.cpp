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
 * @file testCombinDSS.cpp
 * @brief Tests for the class CombinatorialDSS
 * @ingroup Tests
 * @author Xavier Provençal (\c xavier.provencal@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2011/05/02
 *
 * Functions for testing class CombinDSS.
 *
 * This file is part of the DGtal library.
 *
 * @see CombiantorialDSS.h
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/2d/CombinatorialDSS.h"
#include "DGtal/geometry/2d/ArithmeticalDSS.h"
#include "ConfigTest.h"
#include <time.h>

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class CombinDSS.
///////////////////////////////////////////////////////////////////////////////





/**
 * Test exted and retract function on a complex chape
 */
bool testCombinatorialDSS() 
{
  typedef int Coordinate;
  typedef FreemanChain<Coordinate> ContourType; 
  typedef ContourType::ConstIterator ConstIterator;
  typedef CombinatorialDSS<Coordinate> CombinatorialDSS;

  std::string filename = testPath + "samples/france.fc";
  std::fstream fst;
  fst.open (filename.c_str(), std::ios::in);
  ContourType theContour(fst);


  int nbRetract = 0;
  CombinatorialDSS C(theContour.begin());
  while ( *C.end() != *theContour.end() ) 
  {
    if ( ! C.extend() )  {
      C.retract();
      ++nbRetract;
    }
  }
  return (nbRetract == 3485) ;
}
  

/**
 * Builds CombinatorialDSS and ArithmeticalDSS in the fourth quadrants and
 * compares them, if both are equals, the test is passed.
 *
 */
bool CompareToArithmetical()
{
  typedef int Coordinate;
  typedef FreemanChain<Coordinate> ContourType; 
  typedef ArithmeticalDSS<ContourType::ConstIterator,Coordinate,4> ReferenceType;
  typedef CombinatorialDSS<Coordinate> TestedType;
  std::string filename = testPath + "samples/manche.fc";
  std::fstream fst;
  fst.open (filename.c_str(), std::ios::in);
  ContourType theContour(fst);
  ContourType::ConstIterator it = theContour.begin();
  for (int i=0; i<10; i++) it++;
  CombinatorialDSS<int> C(it);
  ArithmeticalDSS<FreemanChain<int>::ConstIterator, int, 4> A(it);
  A.extend(); 
  int nbPts = 2;
  bool a,c;
  bool res = true;
  while ( *C.end() != *theContour.end() ) {
    double d = ((double) rand()) / ((double) RAND_MAX );
    if ( (d < 0.15)  && (nbPts > 2) ) {
      a = A.retract();
      c = C.retract();
      if (a && c) --nbPts;
    }
    else if ( (d < 0.30)  && (nbPts > 2) ) {
      a = A.retractOppositeEnd();
      c = C.retractOppositeEnd();
      if (a && c) --nbPts;
    }
    else  {
      a = A.extend();
      c = C.extend();
      if (a && c) ++nbPts;
    }
    if ( !C.isValid() || (a xor c) || (C != A) ) {
      res = false;
      break;
    }
  }
  return res;
}




///////////////////////////////////////////////////////////////////////////////
// Standard services - public :
int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class CombinDSS" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  //bool res = testCombinDSS() && testEquality(); // && ... other tests
  bool res = testCombinatorialDSS() && CompareToArithmetical();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();

  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
