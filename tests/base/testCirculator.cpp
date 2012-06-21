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
 * Iteration accross the end of a range
 *
 */
template<typename Iterator>
bool testOffset(const Iterator& itb, const Iterator& ite, const vector<int>& groundTruth)
{

  BOOST_CONCEPT_ASSERT(( boost::BidirectionalIterator<Iterator> ));
  BOOST_CONCEPT_ASSERT(( boost::BidirectionalIterator< Circulator<Iterator> > ));

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

/**
 * Comparison tests
 *
 */
template<typename Type1, typename Type2>
bool 
testComparison(const Type1& a, const Type2& b) {
  return (a == b);
}

/**
 * validity of the range
 *
 */
template<typename Iterator>
bool testIsNotEmpty(const Iterator& itb, const Iterator& ite, const bool& aFlagIsNotEmpty)
{
  return (isNotEmpty(itb,ite) == aFlagIsNotEmpty ); 
}

template< typename IC> 
inline
bool getGeneralTag( const IC& /*ic*/){
  cout << typeid( typename IteratorCirculatorTraits<IC>::Category() ).name() << endl; 
  return true; 
}

template< typename IC> 
inline
bool getSpecificTag( const IC& /*ic*/){
  cout << typeid( typename IC::iterator_category() ).name() << endl; 
  return true; 
}


template< typename IC > 
inline
bool getType( const IC& , IteratorType ) {
  cout << "IteratorType" << endl;
  return true;
}

template< typename IC > 
inline
bool getType( const IC& , CirculatorType ) {
  cout << "CirculatorType" << endl;
  return true;
}

template< typename IC> 
inline
bool getType( const IC& ic){
  return getType<IC>( ic, typename IteratorCirculatorTraits<IC>::Type() );
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

//incrementation / decrementation
  trace.info() << endl;
  trace.info() << "Iterate" << endl;
  bool res = testOffset<vector<int>::iterator>(v.begin(),v.end(), v2)
  && testOffset<vector<int>::reverse_iterator>(v.rbegin(),v.rend(), v3)
  && testOffset<vector<int>::const_iterator>(v.begin(),v.end(), v2)
  && testOffset<vector<int>::const_reverse_iterator>(v.rbegin(),v.rend(), v3);

//comparisons
  trace.info() << endl;
  trace.info() << "Compare" << endl;
  trace.info() << "(const / not const)" << endl;
  Circulator<vector<int>::iterator> c1( v.begin(), v.begin(), v.end() );
  Circulator<vector<int>::iterator> c2( c1 );
  Circulator<vector<int>::const_iterator> c3( c2 );
  res = res && testComparison<Circulator<vector<int>::iterator>,Circulator<vector<int>::iterator> >(c1,c2)
  && testComparison<Circulator<vector<int>::iterator>,Circulator<vector<int>::const_iterator> >(c1,c3);

  trace.info() << "(reverse_iterator<Circulator> / Circulator<reverse_iterator>)" << endl;
  std::reverse_iterator<Circulator<vector<int>::iterator> > rc1( c1 );
  Circulator <vector<int>::reverse_iterator> c4(  v.rend(), v.rbegin(), v.rend() );
  res = res && testComparison<vector<int>::iterator,vector<int>::iterator>(rc1.base().base(), c4.base().base());
  trace.info() << "first element: (" << *--rc1 << " == " << *--c4 << ")" << endl;
  res = res && testComparison<int,int>(*rc1, *c4);

//tags
  trace.info() << endl;
  trace.info() << "Tags for classic iterators" << endl;
  getGeneralTag< vector<int>::iterator > ( v.begin() ); 
  getSpecificTag< vector<int>::iterator > ( v.begin() ); 
  getGeneralTag< Circulator<vector<int>::iterator> > ( c2 ); 
  getSpecificTag< Circulator<vector<int>::iterator> > ( c2 ); 
  getType< vector<int>::iterator > ( v.begin() ); 
  getType< Circulator<vector<int>::iterator> > ( c2 ); 



  trace.info() << "Tags for pointers" << endl;
  int t[5] = {1, 2, 3, 4, 5};
  getGeneralTag< int* > ( t ); 
  getType< int* > ( t ); 

  Circulator<int*> tc(t, t, t+5); 
  trace.info() << *tc++  << *tc++ << *tc++ <<  *tc++ <<  *tc++ <<  *tc++ <<  *tc++ << *tc++ << endl;

  getSpecificTag< Circulator<int*> > ( tc ); 
  getGeneralTag< Circulator<int*> > ( tc ); 
  getType< Circulator<int*> > ( tc ); 

//range validity
  trace.info() << endl;
  trace.info() << "Function isNotEmpty" << endl;
  res = res && testIsNotEmpty<vector<int>::iterator>(v.begin(),v.end(),true)
 && testIsNotEmpty<vector<int>::iterator>(v.end(),v.end(),false); 
  Circulator<vector<int>::iterator> validC( v.begin(), v.begin(), v.end() );
  Circulator<vector<int>::iterator> validC2( v.end(), v.begin(), v.end() );
  Circulator<vector<int>::iterator> notValidC( v.begin(), v.begin(), v.begin() );
  res = res && testIsNotEmpty<Circulator<vector<int>::iterator> >(validC,validC,true)
 && testIsNotEmpty<Circulator<vector<int>::iterator> >(validC,notValidC,false)
 && testIsNotEmpty<Circulator<vector<int>::iterator> >(validC,validC2,true)
; 

  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
