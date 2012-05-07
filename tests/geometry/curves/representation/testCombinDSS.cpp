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
#include "DGtal/geometry/curves/representation/CombinatorialDSS.h"
#include "DGtal/geometry/curves/representation/ArithmeticalDSS.h"
#include "ConfigTest.h"
#include "DGtal/geometry/curves/representation/CBidirectionalSegmentComputer.h"
#include <time.h>
#include <list>
#include <vector>
#include "DGtal/geometry/curves/representation/GreedySegmentation.h"

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
  typedef string::const_iterator codeIterator;
  //typedef CombinatorialDSS<codeIterator, Coordinate> CombinatorialDSS;
  typedef CombinatorialDSS< list<char>::iterator, Coordinate> CombinatorialDSS_list;
  typedef CombinatorialDSS<codeIterator, Coordinate> CombinatorialDSS_string;

  BOOST_CONCEPT_ASSERT(( CBidirectionalSegmentComputer<CombinatorialDSS_list> ));
  BOOST_CONCEPT_ASSERT(( CBidirectionalSegmentComputer<CombinatorialDSS_string> ));
  //BOOST_CONCEPT_ASSERT(( CSegment<CombinatorialDSS> ));

  trace.beginBlock ( "Test \'extend\' and \'retract\'" );

  std::string filename = testPath + "samples/france.fc";
  std::fstream fst;
  fst.open (filename.c_str(), std::ios::in);
  ContourType theContour(fst);
  //ContourType theContour("0003003003");

  list<char> l;
  //for (unsigned int i=0; i<theContour.size(); ++i )
  for ( string::const_iterator it = theContour.chain.begin(); it != theContour.chain.end(); ++it )
    {
      l.push_back( *it );
    }

  list<char>::iterator it = l.begin();

  CombinatorialDSS_list C1;
  C1.init( it, theContour.firstPoint() );

  CombinatorialDSS_list C2;
  C2.init( C1.begin() );

  CombinatorialDSS_string C3;
  C3.init( theContour );

  CombinatorialDSS_string C4;
  C4.init( theContour.begin() );

  int nbRetract = 0;
  while ( C3.end() != theContour.chain.end() )
    {
      bool b1 = C1.extendForward();
      bool b2 = C2.extendForward();
      bool b3 = C3.extendForward();
      bool b4 = C4.extendForward();
      if ( b1 && b2 && b3 && b4 )
        {
        }
      else if ( !b1 && !b2 && !b3 && !b4 )
        {
          C1.retractForward();
          C2.retractForward();
          C3.retractForward();
          C4.retractForward();
          ++nbRetract;
        }
      else
        {
          cout << b1 << " " << b2 << " " << b3 << " " << b4 << endl;
          cout << C1 << endl;
          cout << C2 << endl;
          cout << C3 << endl;
          cout << C4 << endl;
          return false;
        }
    }
  trace.endBlock();
  return (nbRetract == 3485) ;
}


/**
 * Builds CombinatorialDSS and ArithmeticalDSS in the fourth quadrants and
 * compares them, if both are equals, the test is passed.
 */
bool CompareToArithmetical()
{
  typedef int Coordinate;
  typedef FreemanChain<Coordinate> ContourType; 
  typedef ArithmeticalDSS<ContourType::ConstIterator,Coordinate,4> ReferenceType;
  typedef string::const_iterator codeIterator;
  typedef CombinatorialDSS<codeIterator, Coordinate> TestedType;

  trace.beginBlock ( "Comparing to ArithmeticalDSS" );

  std::string filename = testPath + "samples/manche.fc";
  std::fstream fst;
  fst.open (filename.c_str(), std::ios::in);
  ContourType theContour(fst);
  ContourType::ConstIterator it = theContour.begin();
  TestedType C;
  C.init( it );
  ArithmeticalDSS<FreemanChain<int>::ConstIterator, int, 4> A(it);
  A.extendForward(); 
  bool res = true;
  while ( C.end() != theContour.chain.end() ) 
    {
      bool a = A.extendForward();
      bool c = C.extendForward();
      if ( a ^ c )
        {
          res = false;
          cout << "Extension test error\n";
          break;
        }
      else if ( ! a )
        {
          A.retractForward();
          C.retractForward();
        }
      if ( C != A )
        {
          res = false;
          cout << "Equality test error\n";
          break;
        }
      if ( ( C.getA() != A.getA() ) || ( C.getB() != A.getB() ) ||
           ( C.getMu() != A.getMu() ) || ( C.getOmega() != A.getOmega() ) ||
           ( C.getUf() != A.getUf() ) || ( C.getUl() != A.getUl() ) ||
           ( C.getLf() != A.getLf() ) || ( C.getLl() != A.getLl() ) 
         )
        {
          cout << "Arithmetic parameters error\n";
          cout <<  C << endl;
          cout <<  A << endl;
          cout << "getA()    " <<  C.getA()     << " --- " <<  A.getA() << "\n";
          cout << "getB()    " <<  C.getB()     << " --- " <<  A.getB() << "\n";
          cout << "getMu()   " <<  C.getMu()    << " --- " <<  A.getMu() << "\n";
          cout << "getOmega()" <<  C.getOmega() << " --- " <<  A.getOmega() << "\n";
          cout << "getUf()   " <<  C.getUf()    << " --- " <<  A.getUf() << "\n";
          cout << "getUl()   " <<  C.getUl()    << " --- " <<  A.getUl() << "\n";
          cout << "getLf()   " <<  C.getLf()    << " --- " <<  A.getLf() << "\n";
          cout << "getLl()   " <<  C.getLl()    << " --- " <<  A.getLl() << endl;
          res = false;
          break;
        }
    }
  trace.endBlock();
  return res;
}



bool testInGreedySegmentation( )
{
  typedef FreemanChain<int> Contour; 
  typedef Contour::Point Point;
  typedef Contour::Vector Vector;
  typedef CombinatorialDSS<string::const_iterator, int> combinDSS;
  typedef GreedySegmentation<combinDSS> combinDecomposition;

  std::string filename = testPath + "samples/BigBall.fc";
  std::fstream fst;
  fst.open (filename.c_str(), std::ios::in);
  Contour theContour(fst);

  trace.beginBlock ( "Test CombinatorialDSS in greedy segmentation" );
  combinDecomposition combin_dec( theContour.chain.begin(), theContour.chain.end(), combinDSS() );
  vector<combinDSS> theCombinDSS;
  for ( combinDecomposition::SegmentComputerIterator i = combin_dec.begin();
       i != combin_dec.end(); ++i ) 
    {
      combinDSS c( *i );
      theCombinDSS.push_back( c );
    } 
  bool ok = ( theCombinDSS.size() == 1593 );
  trace.endBlock();

  return ok;
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

  bool res = testCombinatorialDSS()     
    && CompareToArithmetical() 
    && testInGreedySegmentation();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();

  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
