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
 * @file testCirculator.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/07/05
 *
 * Functions for testing class Circulator.h.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/base/Circulator.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class Circulator.
///////////////////////////////////////////////////////////////////////////////

/**
 * Example of a test. To be completed.
 *
 */
template<typename Iterator>
bool testCirculator(const Iterator& itb, const Iterator& ite, const vector<int>& groundTruth)
{

  //list
  cout << endl;
  copy(itb,ite,ostream_iterator<int>(cout, " ")); 
  cout << " => ";

  //use of Circulators
  vector<int> v; 
  Circulator<Iterator> cb( itb, itb, ite );
  Circulator<Iterator> c( ++cb );
  do {
    v.push_back(*c);
    c++;
  } while (c != cb);

  //offset list
  copy(v.begin(),v.end(),ostream_iterator<int>(cout, " ")); 

  //ground truth
  cout << " ( == ";
  copy(groundTruth.begin(),groundTruth.end(),ostream_iterator<int>(cout, " ")); 
  cout << ")" << endl;

  return equal( v.begin(),v.end(),groundTruth.begin() );
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class BasicBoolFunctions" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  vector<int> v;
  v.push_back(1);
  v.push_back(2);
  v.push_back(3);
  v.push_back(4);
  v.push_back(5);

  vector<int> v2;
  v2.push_back(2);
  v2.push_back(3);
  v2.push_back(4);
  v2.push_back(5);
  v2.push_back(1);

  vector<int> v3;
  v3.push_back(4);
  v3.push_back(3);
  v3.push_back(2);
  v3.push_back(1);
  v3.push_back(5);

  bool res = testCirculator<vector<int>::iterator>(v.begin(),v.end(), v2)
  && testCirculator<vector<int>::reverse_iterator>(v.rbegin(),v.rend(), v3)
  && testCirculator<vector<int>::const_iterator>(v.begin(),v.end(), v2)
  && testCirculator<vector<int>::const_reverse_iterator>(v.rbegin(),v.rend(), v3)
 // && ... other tests
;
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
