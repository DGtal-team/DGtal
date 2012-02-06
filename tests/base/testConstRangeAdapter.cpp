/*
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


 * @file testConstRangeAdapter.cpp
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 *
 * @date 2012/02/06
 *
 * Functions for testing class ConstRangeAdapter
 *
 * This file is part of the DGtal library.
 */

#include <iostream>
#include <fstream>
#include <sstream>

#include "DGtal/base/Common.h"
#include "DGtal/base/CConstRange.h"
#include "DGtal/base/ConstRangeAdapter.h"




using namespace std;
using namespace DGtal;


/*
 * Ranges
 *
 */
template <typename Range>
bool testRange(const Range &aRange)
{

  trace.info() << endl;
  trace.info() << "Testing Range" << endl;
  
  typedef typename IteratorCirculatorTraits<typename Range::ConstIterator>::Value Value; 
  std::vector<Value> v1,v2,v3,v4; 
  
  {
    trace.info() << "Forward" << endl;
    typename Range::ConstIterator i = aRange.begin();
    typename Range::ConstIterator end = aRange.end();
    for ( ; i != end; ++i) {
      cout << *i << endl;
      v1.push_back(*i); 
    }
  }
  {
    trace.info() << "Backward" << endl;
    typename Range::ConstReverseIterator i = aRange.rbegin();
    typename Range::ConstReverseIterator end = aRange.rend();
    for ( ; i != end; ++i) {
      cout << *i << endl;
      v2.push_back(*i); 
    }
  }
  {
    trace.info() << "Circulator" << endl;
    typename Range::ConstCirculator c = aRange.c();
    typename Range::ConstCirculator cend = aRange.c();
    if (isNotEmpty(c,cend)) 
      {
	do 
	  {
	    cout << *c << endl;
	    v3.push_back(*c);
	    c++;
	  } while (c!=cend); 
      }
  }
  
  {
    trace.info() << "Reverse Circulator" << endl;
    typename Range::ConstReverseCirculator c = aRange.rc();
    typename Range::ConstReverseCirculator cend = aRange.rc();
    if (isNotEmpty(c,cend)) 
      {
	do 
	  {
	    cout << *c << endl;
	    v4.push_back(*c);
	    c++;
	  } while (c!=cend); 
      }
  }

  return ( std::equal(v1.begin(),v1.end(),v3.begin())
	   && std::equal(v2.begin(),v2.end(),v4.begin())
	   && std::equal(v1.begin(),v1.end(),v2.rbegin())
	   && std::equal(v3.begin(),v3.end(),v4.rbegin()) );
}


template <typename Range>
void testRangeConceptChecking()
{
  BOOST_CONCEPT_ASSERT(( CConstRange<Range> ));
}

/*
 * Standard services - public :
 */

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class ConstRangeAdapter" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  const int n = 10; 
  std::vector<int> v;
  std::back_insert_iterator<std::vector<int> > ito(v); 
  for (int i = 1; i < n; ++i) 
      *ito++ = i;

  typedef ConstRangeAdapter<std::vector<int>::iterator > SimpleRange; 
  SimpleRange r1(v.begin(), v.end()); 

  Thresholder<int> t(n/2);  
  typedef ConstRangeAdapter<std::vector<int>::iterator, Thresholder<int>, bool > BoolRange; 
  BoolRange r2(v.begin(), v.end(), t); 

  /////////// concept checking
  testRangeConceptChecking<SimpleRange>();
  testRangeConceptChecking<BoolRange>();

  /////////// iterators tests
  bool res = testRange(r1) && testRange(r2); 

  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();  
  return res ? 0 : 1;
}
