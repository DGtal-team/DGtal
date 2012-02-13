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
 * @file testOutputIteratorAdapter.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/09/01
 *
 * Functions for testing class OutputIteratorAdapter.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <map>
#include <vector>

#include "DGtal/base/Common.h"
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/base/OutputIteratorAdapter.h"


///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class OutputIteratorAdapter.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testPairs()
{
  typedef map<string,string> Map; 
  typedef pair<string,string> Pair; 

  Map m; 
  m.insert( Pair("Proudhon", "Pierre-Joseph" ) ); 
  m.insert( Pair("Reclus", "Élisée" ) ); 
  m.insert( Pair("Zamenhof", "Ludwik Lejzer" ) ); 

  typedef vector<string> Vec; 
  Vec v; 
  v.push_back( "P-J." ); 
  v.push_back( "E." ); 
  v.push_back( "L. L." ); 

  trace.beginBlock ( "Update..." );

  typedef OutputIteratorAdapter<Map::iterator, Pair2ndMutator<string>, string> Adapter; 
  BOOST_CONCEPT_ASSERT(( boost::OutputIterator<Adapter, string> ));
  Adapter  a( m.begin(), Pair2ndMutator<string>() ); 

  copy(v.begin(), v.end(), a); 

  trace.endBlock();

  Map m2; 
  m2.insert( Pair("Proudhon", "P-J." ) ); 
  m2.insert( Pair("Reclus", "E.") ); 
  m2.insert( Pair("Zamenhof", "L. L." ) ); 

  bool flag; 
  trace.beginBlock ( "Comparison..." );

  flag = std::equal( m.begin(), m.end(), m2.begin() ); 

  trace.endBlock();
    
  return flag;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class OutputIteratorAdapter" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testPairs()
  ; // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
