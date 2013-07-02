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
 * @file testArithmeticalDSS.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/06/28
 *
 * Functions for testing class ArithmeticalDSS.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/curves/ArithmeticalDSS.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ArithmeticalDSS.
///////////////////////////////////////////////////////////////////////////////
/**
 * Test of the main operators
 * @tparam DSS a model of arithmetical DSS, 
 * either naive or standard 
 */
template <typename DSS>
bool mainTest()
{
  typedef typename DSS::Point Point; 

  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Main operators..." );

  //operateur constructeur, copie, affectation
  trace.info() << "constructor, copy, assignement, equality" << std::endl; 

  DSS dss(0, 1, 0,  
	  Point(0,0), Point(1,0), 
	  Point(0,0), Point(1,0),
	  Point(0,0), Point(1,0) ); 
  DSS dss2 = dss; 
  DSS dss3(1, 1, 0,  
	  Point(0,0), Point(1,1), 
	  Point(0,0), Point(1,1),
	  Point(0,0), Point(1,1) );
  DSS dss4 = dss3; 
  dss3 = dss2 = dss; 

  //egalite, difference
  DSS dss5(0, -1, 0, 
	  Point(1,0), Point(0,0), 
	  Point(1,0), Point(0,0),
	  Point(1,0), Point(0,0) ); 
  
  if ( (dss == dss2)
       &&(dss == dss3)
       &&(dss != dss4)
       &&(dss == dss5) )
    nbok++; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << std::endl;

  //validite
  trace.info() << "valid dss" << std::endl; 
  if ( dss.isValid() && dss3.isValid() && dss5.isValid() ) 
    nbok++; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << std::endl;

  DSS dss6(0, 1, 0, 1, 
	  Point(1,0), Point(0,0), 
	  Point(1,0), Point(0,0),
	  Point(1,0), Point(0,0) ); 

  trace.info() << "not valid dss" << std::endl; 
  if (!dss6.isValid()) 
    nbok++; 
  nb++;

  trace.info() << "(" << nbok << "/" << nb << ") "
	       << std::endl;

  //accessors
  trace.info() << "a,b,mu,omega accessors" << std::endl; 

  if ( (dss.a() == 0)&&(dss.b() == 1)&&(dss.mu() == 0)&&(dss.omega() == 1) )
    nbok++; 
  nb++; 

  trace.info() << "(" << nbok << "/" << nb << ") "
	       << std::endl;

  trace.info() << "points accessors" << std::endl; 
  if ( (dss.front() == Point(1,0))&&(dss.back() == Point(0,0)) )
    nbok++; 
  nb++; 
  if ( (dss.Ul() == Point(1,0))&&(dss.Uf() == Point(0,0)) )
    nbok++; 
  nb++; 
  if ( (dss.Ll() == Point(1,0))&&(dss.Lf() == Point(0,0)) )
    nbok++; 
  nb++; 

  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << std::endl;

  DSS dss7(Point(0,0), Point(8,5)); 
  
  trace.info() << "remainder, position, tests" << std::endl; 
  trace.info() << dss7 << std::endl; 

  if ( (dss7.isValid()) 
       && (dss7.remainder( Point(8,5) ) == 0)
       &&(dss7.remainder( Point(16,10) ) == 0)
       &&(dss7.remainder( Point(3,2) ) == -1)
       &&(dss7.remainder( Point(5,3) ) == 1) )
    nbok++; 
  nb++; 

  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << std::endl;
 
  if ( (dss7.orthogonalPosition( Point(0,0) ) == 0)
       &&(dss7.orthogonalPosition( Point(8,5) ) == 89)
       &&(dss7.orthogonalPosition( Point(1,0) ) == 8)
       &&(dss7.orthogonalPosition( Point(-1,0) ) == -8) )
    nbok++; 
  nb++; 

  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << std::endl;

  if ( (dss7.isInDSL( Point(0,0) ))
       &&(dss7.isInDSL( Point(16,10) ))
       &&(dss7.isInDSL( Point(5,3) ))
       &&(!dss7.isInDSL( Point(3,2) )) )
    nbok++; 
  nb++; 

  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << std::endl;

  if ( (dss7( Point(0,0) ))
       &&(!dss7( Point(16,10) ))
       &&(dss7( Point(5,3) ))
       &&(!dss7( Point(3,2) ))
       &&(!dss7( Point(-1,0) )) )
    nbok++; 
  nb++; 

  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << std::endl;
            
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * Test of the range services 
 * @param dss an instance of DSS
 * @tparam DSS a model of arithmetical DSS, 
 * either naive or standard 
 */
template <typename DSS>
bool rangeTest(const DSS& dss)
{
  typedef typename DSS::Point Point; 

  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Range/Iterator services..." );
  trace.info() << dss << std::endl; 

  if (dss.isValid())
    nbok++; 
  nb++; 

  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << std::endl;

  {//forward pass  
    typedef typename DSS::ConstIterator I; 
    BOOST_CONCEPT_ASSERT(( boost_concepts::ReadableIteratorConcept<I> )); 
    BOOST_CONCEPT_ASSERT(( boost_concepts::BidirectionalTraversalConcept<I> ));
    bool res = true; 
    int c = 0; 
    for (I it = dss.begin(), itEnd = dss.end(); 
	 ( (it != itEnd)&&(res)&&(c<100) ); 
	 ++it, ++c)
      {
	trace.info() << *it << " ";  
	if ( !dss(*it) )
	  res = false; 
      }
    trace.info() << " : " << c << " points " << std::endl; 
    trace.info() << std::endl; 

    if ( (res)&&(c == (dss.omega()+1))
	 &&(*dss.begin() == dss.back())
	 &&(*--dss.end() == dss.front()) ) 
      nbok++; 
    nb++;

  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << std::endl;
  }

  {//backward pass
    typedef typename DSS::ConstReverseIterator I; 
    bool res = true; 
    int c = 0; 
    for (I it = dss.rbegin(), itEnd = dss.rend(); 
	 ( (it != itEnd)&&(res)&&(c<100) ); 
	 ++it, ++c)
      {
	trace.info() << *it << " ";  
	if ( !dss(*it) )
	  res = false; 
      }
    trace.info() << " : " << c << " points " << std::endl; 
    trace.info() << std::endl; 

    if ( (res)&&(c == (dss.omega()+1))
	 &&(*dss.rbegin() == dss.front())
	 &&(*--dss.rend() == dss.back()) ) 
      nbok++; 
    nb++;

  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << std::endl;
  }

  trace.endBlock();
  
  return nbok == nb;
}


///////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class ArithmeticalDSS" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  //main operator
  bool res = mainTest<DGtal::ArithmeticalDSS<DGtal::int32_t> >()
    && mainTest<DGtal::ArithmeticalDSS<DGtal::int32_t, DGtal::BigInteger, 4> >(); 

  //range services for 8 adjacency
  {
    typedef DGtal::ArithmeticalDSS<DGtal::int32_t> DSS8; 
    typedef DSS8::Point Point; 

    DSS8 dss0(Point(0,0), Point(8,5)); 
    DSS8 dss1(Point(0,0), Point(5,8)); 
    DSS8 dss2(Point(0,0), Point(-5,8)); 
    DSS8 dss3(Point(0,0), Point(-8,5));
    DSS8 dss4(Point(0,0), Point(-8,-5)); 
    DSS8 dss5(Point(0,0), Point(-5,-8)); 
    DSS8 dss6(Point(0,0), Point(5,-8)); 
    DSS8 dss7(Point(0,0), Point(8,-5)); 

    res = res 
      && rangeTest(dss0)
      && rangeTest(dss1)
      && rangeTest(dss2)
      && rangeTest(dss3)
      && rangeTest(dss4)
      && rangeTest(dss5)
      && rangeTest(dss6)
      && rangeTest(dss7)
      ;
  }

  //range services for 4 adjacency
  {
    typedef DGtal::ArithmeticalDSS<DGtal::int32_t, DGtal::int32_t, 4> DSS4; 
    typedef DSS4::Point Point; 
    
    DSS4 dss0(Point(0,0), Point(8,5)); 
    DSS4 dss1(Point(0,0), Point(5,8)); 
    DSS4 dss2(Point(0,0), Point(-5,8)); 
    DSS4 dss3(Point(0,0), Point(-8,5));
    DSS4 dss4(Point(0,0), Point(-8,-5)); 
    DSS4 dss5(Point(0,0), Point(-5,-8)); 
    DSS4 dss6(Point(0,0), Point(5,-8)); 
    DSS4 dss7(Point(0,0), Point(8,-5)); 

    res = res 
      && rangeTest(dss0)
      && rangeTest(dss1)
      && rangeTest(dss2)
      && rangeTest(dss3)
      && rangeTest(dss4)
      && rangeTest(dss5)
      && rangeTest(dss6)
      && rangeTest(dss7)
      ;
  }

  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
