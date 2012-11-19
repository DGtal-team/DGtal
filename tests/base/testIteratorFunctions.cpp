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
 * @file testIteratorFunctions.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/11/19
 *
 * Functions for testing class IteratorFunctions.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/IteratorFunctions.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class IteratorFunctions.
///////////////////////////////////////////////////////////////////////////////
template<typename Container, typename T>
struct Tool
{
  static void add(Container& c, const T& obj)
  {
    c.push_back(obj); 
  }
};
#ifdef CPP11_FORWARD_LIST 
#include <forward_list>
//specialization for forward lists
template<typename T>
struct Tool<std::forward_list<int>, T>
{
  static void add(std::forward_list<int>& c, const T& obj)
  {
    c.push_front(obj); 
  }
};
#endif

/**
 * Comparison between the result of the rangeMiddle function
 * and the ground truth
 * @param a begin iterator
 * @param b end iterator
 * @param res middle iterator (ground truth)
 * @tparam IC iterator or circulator type
 */
template<typename IC>
bool testMiddleComparison(const IC& a, const IC& b, const IC& res)
{
  return ( (rangeMiddle(a, b)) == res);  
}

/**
 * Test of the advance function
 * @param c any container
 * @tparam Container model of iterable and pushable container
 */
template<typename Container>
bool testAdvance(Container c)
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Advance..." );

  ///////////////
  typedef typename Container::iterator I;
  typedef typename IteratorCirculatorTraits<I>::Category Category;  
  trace.info() << typeid(Category()).name() << std::endl;

  ///////////////
  Tool<Container,int>::add(c,1);  
  Tool<Container,int>::add(c,2);  
  Tool<Container,int>::add(c,3);  
  Tool<Container,int>::add(c,4);  
  Tool<Container,int>::add(c,5);  
  Tool<Container,int>::add(c,6);  

  I res = c.begin(); 
  while (nb < 6)
    {
      I i = c.begin(); 
      DGtal::advanceIterator(i, nb); 
      if ( i == res )
	nbok++;  
      nb++; 
      trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;
      ++res; 
    }

  trace.endBlock();
  
  return (nbok == nb);
}

/**
 * Test of the rangeSize function
 * @param c any container
 * @tparam Container model of iterable and pushable container
 */
template<typename Container>
bool testSize(Container c)
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Size..." );

  ///////////////
  typedef typename Container::iterator I;
  typedef typename IteratorCirculatorTraits<I>::Category Category;  
  trace.info() << typeid(Category()).name() << std::endl;

  trace.info() << "empty underlying range" << std::endl; 
  if ( rangeSize(c.begin(), c.end()) == 0 )
    nbok++;  
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  ///////////////
  Tool<Container,int>::add(c,5);  

  trace.info() << "underlying range of one element" << std::endl; 
  if ( rangeSize(c.begin(), c.end()) == 1 )
    nbok++;  
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  ///////////////
  Tool<Container,int>::add(c,1);  
  Tool<Container,int>::add(c,2);  
  Tool<Container,int>::add(c,3);  
  Tool<Container,int>::add(c,4);  
  Tool<Container,int>::add(c,5);  
  Tool<Container,int>::add(c,6);  

  trace.info() << "two equal iterators" << std::endl; 
  if ( rangeSize(c.begin(), c.begin()) == 0 )
    nbok++;  
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  trace.info() << "whole range (of 7 elements)" << std::endl; 
  if ( rangeSize(c.begin(), c.end()) == 7 )
    nbok++;  
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  /////////////////
  I itb = c.begin(); ++itb; 
  I ite = itb; 
  ite++; ite++; ite++;  ite++; 

  trace.info() << "subrange (of 4 elements)" << std::endl; 
  if ( rangeSize(itb, ite) == 4 )
    nbok++;  
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;


  trace.endBlock();
  
  return (nbok == nb);
}


/**
 * Test of the rangeMiddle function
 * @param c any container
 * @tparam Container model of iterable and pushable container
 */
template<typename Container>
bool testMiddle(Container c)
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Middle iterator..." );

  ///////////////
  typedef typename Container::iterator I;
  typedef typename IteratorCirculatorTraits<I>::Category Category;  
  trace.info() << typeid(Category()).name() << std::endl;

  trace.info() << "empty underlying range" << std::endl; 
  if ( testMiddleComparison(c.begin(), c.end(), c.begin()) )
    nbok++;  
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  ///////////////
  Tool<Container,int>::add(c,5);  

  trace.info() << "underlying range of one element" << std::endl; 
  if ( testMiddleComparison(c.begin(), c.end(), c.begin()) )
    nbok++;  
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  ///////////////
  Tool<Container,int>::add(c,1);  
  Tool<Container,int>::add(c,2);  
  Tool<Container,int>::add(c,3);  
  Tool<Container,int>::add(c,4);  
  Tool<Container,int>::add(c,5);  
  Tool<Container,int>::add(c,6);  
  Tool<Container,int>::add(c,5);  

  trace.info() << "two equal iterators" << std::endl; 
  if ( testMiddleComparison(c.begin(), c.begin(), c.begin()) )
    nbok++;  
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  ///////////////
  I itb = c.begin();
  I ite = itb; 
  ite++; ite++; ite++;  
  ite++; ite++; ite++; ite++;  
  I res1 = itb; 
  res1++; res1++; res1++; 
  I res2 = res1; 
  res1++;  

  trace.info() << "whole range with odd number of elements" << std::endl; 
  if ( testMiddleComparison(c.begin(), ite, res2) )
    nbok++;  
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  trace.info() << "whole range with even number of elements" << std::endl; 
  if ( testMiddleComparison(c.begin(), c.end(), res1) )
    nbok++;  
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  ///////////////
  ite = itb; 
  itb++;  
  ite++; ite++; ite++;  
  ite++; ite++; ite++;  
 
  trace.info() << "subrange with odd number of elements" << std::endl; 
  if ( testMiddleComparison(itb, ite, res2) )
    nbok++;  
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  ///////////////
  ite++; 

  trace.info() << "subrange with even number of elements" << std::endl; 
  if ( testMiddleComparison(itb, ite, res1) )
    nbok++;  
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  trace.endBlock();
  
  return (nbok == nb);
}


///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class IteratorFunctions" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

#ifdef CPP11_FORWARD_LIST 
  std::forward_list<int> fl;
#endif 
  std::list<int> bl; 
  std::vector<int> v; 

  bool res = 
#ifdef CPP11_FORWARD_LIST 
    testAdvance(fl) && 
#endif
    testAdvance(bl) && 
    testAdvance(v) &&
#ifdef CPP11_FORWARD_LIST 
    testMiddle(fl) && 
#endif
    testMiddle(bl) && 
    testMiddle(v) &&
#ifdef CPP11_FORWARD_LIST 
    testSize(fl) && 
#endif
    testSize(bl) && 
    testSize(v);
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
