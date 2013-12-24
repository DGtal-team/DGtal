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
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/base/CConstBidirectionalRange.h"
#include "DGtal/geometry/curves/ArithmeticalDSS.h"
#include "DGtal/geometry/curves/ArithmeticalDSSFactory.h"
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
  BOOST_CONCEPT_ASSERT(( CPointPredicate<DSS> ));
  BOOST_CONCEPT_ASSERT(( CConstBidirectionalRange<DSS> ));

  typedef typename DSS::Point Point;
  typedef typename DSS::Vector Vector;

  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Main operators..." );

  //operateur constructeur, copie, affectation
  trace.info() << "constructor, copy, assignement, equality" << std::endl;

  DSS dss(0, 1,
	  Point(0,0), Point(1,0),
	  Point(0,0), Point(1,0),
	  Point(0,0), Point(1,0) );
  DSS dss2 = dss;
  DSS dss3(Point(0,0), Point(1,1), true);
  DSS dss4 = dss3;
  dss3 = dss2 = dss;

  //egalite, difference
  DSS dss5(0, -1,
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

  DSS dss6(0, 1,
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

  DSS dss7(Point(0,0), Point(8,5), true);

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

  trace.info() << "shift" << std::endl;
  if (dss.remainder(dss.shift()) == dss.omega())
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
/**
 * Unit test of the extension service
 * @param dss an instance of DSS
 * @tparam DSS a model of arithmetical DSS,
 * either naive or standard
 * @param newPointToFront point to add to the dss front
 * @param newPointToBack point to add to the dss back
 * @param nbok (returned) number of passed tests
 * @param nb (returned) number of tests
 * @param code index of the tested configuration
 */
template <typename DSS>
void extensionTest(const DSS& dss,
		   typename DSS::Point newPointToFront,
		   typename DSS::Point newPointToBack,
		   unsigned int& nbok, unsigned int& nb,
		   const unsigned short int& code = 0)
{
  trace.info() << dss << std::endl;
  if (dss.isValid())
    nbok++;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  trace.info() << "to front " << newPointToFront << std::endl;
  if (dss.isExtendableFront( newPointToFront ) == code)
    nbok++;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  DSS mdss = dss; //local and modifiable copy
  if (code == 0)
    {
      if ( (!mdss.extendFront(newPointToFront)) )
	nbok++;
      nb++;
    }
  else
    {
      if ( (mdss.extendFront(newPointToFront))&&(mdss.isValid()) )
	nbok++;
      nb++;
      std::cerr << mdss.isValid() << std::endl;
    }
  trace.info() << mdss << std::endl;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  trace.info() << "to back " << newPointToBack << std::endl;
  if (dss.isExtendableBack( newPointToBack ) == code)
    nbok++;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  mdss = dss; //local and modifiable copy
  if (code == 0)
    {
      if ( (!mdss.extendBack(newPointToBack)) )
	nbok++;
      nb++;
    }
  else
    {
      if ( (mdss.extendBack(newPointToBack))&&(mdss.isValid()) )
	nbok++;
      nb++;
      std::cerr << mdss.isValid() << std::endl;
    }
  trace.info() << mdss << std::endl;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;
}

///////////////////////////////////////////////////////////////////////////////
/**
 * Unit test of the retraction service
 * @param dss an instance of DSS
 * @tparam DSS a model of arithmetical DSS,
 * either naive or standard
 * @param nbok (returned) number of passed tests
 * @param nb (returned) number of tests
 * @param res result of the retraction:
 * 'true' if done, 'false' otherwise
 */
template <typename DSS>
void retractionTest(const DSS& dss,
		    unsigned int& nbok,
		    unsigned int& nb,
		    bool res = true)
{
  typedef typename DSS::Point Point;

  trace.info() << dss << std::endl;
  if (dss.isValid())
    nbok++;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  //local and modifiable copy
  DSS mdss = dss;

  //forward test
  Point first = mdss.back();
  trace.info() << "remove " << first << std::endl;
  if ( ( (mdss.retractBack())
	 && (mdss.isValid())
	 && (mdss(first) == false) ) == res )
    nbok++;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  if (res)
    {
      if ( (mdss.extendBack(first))
	   && (mdss.isValid()) && (mdss == dss) )
	nbok++;
      nb++;
      trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;
    }

  //backward test
  Point last = mdss.front();
  trace.info() << "remove " << last << std::endl;
  if ( ( (mdss.retractFront())
	 && (mdss.isValid())
	 && (mdss(last) == false) ) == res )
    nbok++;
  nb++;
  trace.info() << mdss << std::endl;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;

  if (res)
    {
      if ( (mdss.extendFront(last))
	   && (mdss.isValid()) && (mdss == dss) )
	nbok++;
      nb++;
      trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;
    }

}

/**
 * Test of the update services
 * @tparam DSS a model of arithmetical DSS,
 * either naive or standard
 */
template <typename DSS>
bool updateTest()
{
  typedef typename DSS::Point Point;
  typedef typename DSS::Vector Vector;

  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Extension services..." );

  if (nbok == nb)
    {
      trace.info() << "not connected point" << std::endl;
      DSS dss(Point(0,0), Point(8,5), true);
      extensionTest( dss, Point(9,7), Point(-2,1), nbok, nb );
    }

  if (nbok == nb)
    {
      trace.info() << "not compatible second step" << std::endl;
      DSS dss(Point(0,0), Point(1,1), true);
      extensionTest( dss, Point(0,2), Point(-1,1), nbok, nb );
    }

  if (nbok == nb)
    {
      trace.info() << "a third step" << std::endl;
      DSS dss(Point(0,0), Point(2,1), true);
      extensionTest( dss, Point(2,2), Point(0,1), nbok, nb );
    }

  if (nbok == nb)
    {
      trace.info() << "strongly exterior" << std::endl;
      DSS dss(Point(0,0), Point(8,5), true);
      extensionTest( dss, Point(9,6), Point(-1,0), nbok, nb );
    }

  if (nbok == nb)
    {
      trace.info() << "confounded points" << std::endl;
      DSS dss(Point(0,0), Point(8,5), true);
      extensionTest( dss, Point(8,5), Point(0,0), nbok, nb, 9 );
    }

  if (nbok == nb)
    {
      trace.info() << "strongly interior points" << std::endl;
      DSS dss0(Point(0,0), Point(8,5), true);
      DSS dss(5, 8, Point(-2,-2), Point(8,5),
	      dss0.Uf(), dss0.Ul(),
	      dss0.Lf(), dss0.Ll() );
      extensionTest( dss, Point(9,5), Point(-3,-2), nbok, nb, 9 );
    }

  if (nbok == nb)
    {
      trace.info() << "weakly interior points on the left" << std::endl;
      DSS dss0(Point(0,0), Point(8,5), true);
      Point newPointToBack = dss0.Lf()-Vector(8,5);
      Point newPointToFront = dss0.Ul()+Vector(8,5);
      DSS dss(5, 8,
	      newPointToBack+dss0.steps().second,
	      newPointToFront-dss0.steps().second,
	      dss0.Uf(), dss0.Ul(),
	      dss0.Lf(), dss0.Ll()+Vector(8,5) );
      extensionTest( dss, newPointToFront, newPointToBack, nbok, nb, 5 );
    }

  if (nbok == nb)
    {
      trace.info() << "weakly exterior points on the left" << std::endl;
      DSS dss0(Point(0,0), Point(8,5), true);
      Point newPointToBack = dss0.Uf()+dss0.shift()-Vector(8,5);
      Point newPointToFront = dss0.Ll()-dss0.shift()+Vector(8,5);
      DSS dss(5, 8,
	      newPointToBack+dss0.steps().second,
	      newPointToFront-dss0.steps().second,
	      dss0.Uf(), dss0.Ul(),
	      dss0.Lf()-Vector(8,5), dss0.Ll() );
      extensionTest( dss, newPointToFront, newPointToBack, nbok, nb, 7 );
    }

  if (nbok == nb)
    {
      trace.info() << "weakly interior points on the right" << std::endl;
      DSS dss0(Point(0,0), Point(8,5), true);
      Point newPointToBack = dss0.Uf()-Vector(8,5);
      Point newPointToFront = dss0.Ll()+Vector(8,5);
      DSS dss(5, 8,
	      newPointToBack+dss0.steps().first,
	      newPointToFront-dss0.steps().first,
	      dss0.Uf(), dss0.Ul(),
	      dss0.Lf()-Vector(8,5), dss0.Ll() );
      extensionTest( dss, newPointToFront, newPointToBack, nbok, nb, 6 );
    }

  if (nbok == nb)
    {
      trace.info() << "weakly exterior points on the right" << std::endl;
      DSS dss0(Point(0,0), Point(8,5), true);
      Point newPointToBack = dss0.Lf()-Vector(8,5)-dss0.shift();
      Point newPointToFront = dss0.Ul()+Vector(8,5)+dss0.shift();
      DSS dss(5, 8,
	      newPointToBack+dss0.steps().first,
	      newPointToFront-dss0.steps().first,
	      dss0.Uf(), dss0.Ul(),
	      dss0.Lf(), dss0.Ll()+Vector(8,5) );
      extensionTest( dss, newPointToFront, newPointToBack, nbok, nb, 8 );
    }

  if (nbok == nb)
    {
      trace.info() << "first step" << std::endl;
      DSS dss( Point(0,0), Point(0,0) );
      extensionTest( dss, Point(1,0), Point(-1,0), nbok, nb, 1 );
    }

  if (nbok == nb)
    {
      trace.info() << "first step repetition" << std::endl;
      DSS dss(Point(0,0), Point(1,0), true);
      extensionTest( dss, Point(2,0), Point(-1,0), nbok, nb, 2 );
    }

  if (nbok == nb)
    {
      trace.info() << "second step (above)" << std::endl;
      DSS dss0(Point(0,0), Point(2,1), true);
      DSS dss(Point(0,0), Point(2,1) - dss0.steps().second);
      Point newPointToBack = Point(0,0) - dss0.steps().second;
      extensionTest( dss, Point(2,1), newPointToBack, nbok, nb, 3 );
    }

  if (nbok == nb)
    {
      trace.info() << "second step (below)" << std::endl;
      DSS dss0a(Point(0,0), Point(2,-1), true);
      DSS dss0b(Point(0,0), Point(2,1), true);
      DSS dss(Point(0,0), Point(2,-1) - dss0a.steps().first);
      Point newPointToBack = Point(0,0) - dss0a.steps().first;
      extensionTest( dss, Point(2,-1), newPointToBack, nbok, nb, 4 );
    }

  trace.endBlock();

  if (nbok == nb)
    {
      trace.beginBlock ( "Retraction services..." );

      {
	trace.info() << "upper leaning points" << std::endl;
	DSS dss(Point(0,0), Point(8,5), true);
	retractionTest( dss, nbok, nb );
      }

      if (nbok == nb)
	{
	  trace.info() << "lower leaning points" << std::endl;
	  DSS dss0(Point(0,0), Point(8,5), true);
	  Point first = dss0.Lf();
	  Point last = dss0.Lf() + Vector(8,5);
	  DSS dss(5, 8, first, last,
		  Point(8,5), Point(8,5),
		  first, last );
	  retractionTest( dss, nbok, nb );
	}

      if (nbok == nb)
	{
	  trace.info() << "upper leaning points (repetitions)" << std::endl;
	  DSS dss(Point(0,0), Point(16,10), true);
	  retractionTest( dss, nbok, nb );
	}

      if (nbok == nb)
	{
	  trace.info() << "lower leaning points (repetitions)" << std::endl;
	  DSS dss0(Point(0,0), Point(16,10), true);
	  Point first = dss0.Lf();
	  Point last = dss0.Lf() + Vector(16,10);
	  DSS dss(5, 8, first, last,
		  Point(8,5), Point(16,10),
		  first, last );
	  retractionTest( dss, nbok, nb );
	}

      if (nbok == nb)
	{
	  trace.info() << "no change" << std::endl;
	  DSS dss0(Point(0,0), Point(21,13), true);
	  typename DSS::ConstIterator itb = dss0.begin();
	  --itb; --itb; --itb;
	  typename DSS::ConstIterator ite = dss0.end();
	  ++ite; ++ite; ++ite;
	  DSS dss(dss0.a(), dss0.b(), *itb, *ite,
		  dss0.Uf(), dss0.Ul(), dss0.Lf(), dss0.Ll() );
	  retractionTest( dss, nbok, nb );
	}

      if (nbok == nb)
	{
	  trace.info() << "one point" << std::endl;
	  DSS dss(Point(0,0), Point(0,0), true);
	  retractionTest( dss, nbok, nb, false );
	}

      if (nbok == nb)
	{
	  trace.info() << "two points" << std::endl;
	  DSS dss(Point(0,0), Point(1,0), true);
	  retractionTest( dss, nbok, nb );
	}

      if (nbok == nb)
	{
	  trace.info() << "from two steps to one step" << std::endl;
	  DSS dss(Point(0,0), Point(1,1), true);
	  retractionTest( dss, nbok, nb );
	}


      trace.endBlock();
    }

  return nbok == nb;
}

/**
 * Test of the directional position
 * and the checks of the steps
 * @tparam DSS a model of arithmetical DSS,
 * either naive or standard
 */
template <typename DSS>
bool compatibleStepsTest(const DSS& dss)
{
  typedef typename DSS::Point Point;
  typedef typename DSS::Vector Vector;

  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "directional Position..." );

  trace.info() << "shift: " << dss.shift()
	       << ", front pos: " << dss.position( dss.front() )
	       << ", back pos:  " << dss.position( dss.back() ) << std::endl;
  if ( dss.position( dss.front() )
       > dss.position( dss.back() ) )
    nbok++;
  nb++;

  trace.info() << "(" << nbok << "/" << nb << ") "
	       << std::endl;

  trace.endBlock();

  trace.beginBlock ( "Compatible steps..." );

  ////////////////// forward extension
  DSS mdss = dss;
  if ( mdss.extendFront(mdss.front()-dss.shift()+dss.steps().first) )
    nbok++;
  nb++;

  trace.info() << "(" << nbok << "/" << nb << ") "
	       << std::endl;
  mdss = dss;
  if ( !mdss.extendFront(mdss.front()-dss.shift()) )
    nbok++;
  nb++;

  trace.info() << "(" << nbok << "/" << nb << ") "
	       << std::endl;

  mdss = dss;
  if ( !mdss.extendFront(mdss.front()-dss.shift()-dss.steps().first) )
    nbok++;
  nb++;

  trace.info() << "(" << nbok << "/" << nb << ") "
	       << std::endl;

  ////////////////// backward extension
  mdss = dss;
  if ( mdss.extendBack(mdss.back()+dss.shift()-dss.steps().first) )
    nbok++;
  nb++;

  trace.info() << "(" << nbok << "/" << nb << ") "
	       << std::endl;
  mdss = dss;
  if ( !mdss.extendBack(mdss.back()+dss.shift()) )
    nbok++;
  nb++;

  trace.info() << "(" << nbok << "/" << nb << ") "
	       << std::endl;

  mdss = dss;
  if ( !mdss.extendBack(mdss.back()+dss.shift()+dss.steps().first) )
    nbok++;
  nb++;

  trace.info() << "(" << nbok << "/" << nb << ") "
	       << std::endl;

  trace.endBlock();

  return nbok == nb;
}

/**
 * Test of the main constructors
 * @tparam DSS a model of arithmetical DSS,
 * either naive or standard
 */
template <typename DSS>
bool constructorsTest()
{
  BOOST_CONCEPT_ASSERT(( CPointPredicate<DSS> ));
  BOOST_CONCEPT_ASSERT(( CConstBidirectionalRange<DSS> ));

  typedef typename DSS::Point Point;
  typedef typename DSS::Vector Vector;

  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "constructors..." );

  {
    //pattern
    DSS dss0( Point(0,0), Point(8,5) );
    trace.info() << dss0 << std::endl;

    //construction by points range
    DSS dss( dss0.begin(), dss0.end() );
    trace.info() << dss << std::endl;

    if ( (dss0.isValid())
	 &&(dss.isValid())
	 && (dss0 == dss)
	 && (dss.Lf() == dss.Ll())
	 && (dss.Uf() == dss.back())
	 && (dss.Ul() == dss.front())
	 && (dss.back() != dss.front()) )
      nbok++;
    nb++;
    trace.info() << "(" << nbok << "/" << nb << ") "
		 << std::endl;

    //reversed pattern
    DSS rdss0( Point(0,0), Point(8,5), false );
    trace.info() << rdss0 << std::endl;

    //construction by points range
    DSS rdss( rdss0.begin(), rdss0.end() );
    trace.info() << rdss << std::endl;

    if ( (rdss0.isValid())
	 &&(rdss.isValid())
	 && (rdss0 == rdss)
	 && (rdss.Uf() == rdss.Ul())
	 && (rdss.Lf() == rdss.back())
	 && (rdss.Ll() == rdss.front())
	 && (rdss.back() != rdss.front())
	 && (rdss != dss) )
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

  //main operators
  bool res = mainTest<DGtal::ArithmeticalDSS<DGtal::int32_t> >()
#ifdef WITH_BIGINTEGER
    && mainTest<DGtal::ArithmeticalDSS<DGtal::int32_t, DGtal::BigInteger, 4> >()
#endif
    && mainTest<DGtal::NaiveDSS8<DGtal::int32_t> >()
    && mainTest<DGtal::StandardDSS4<DGtal::int32_t> >()
    ;

  {   //range services for 8 adjacency
    typedef DGtal::ArithmeticalDSS<DGtal::int32_t> DSS8;
    typedef DGtal::ArithmeticalDSSFactory<DGtal::int32_t> Factory;
    typedef DSS8::Point Point;

    res = res
      && rangeTest( Factory::createPattern(Point(0,0), Point(8,5)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(5,8)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(-5,8)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(-8,5)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(-8,-5)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(-5,-8)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(5,-8)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(8,-5)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(1,0)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(-1,0)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(0,1)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(0,-1)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(1,1)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(-1,1)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(1,-1)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(-1,-1)) )
      && rangeTest( Factory::createReversedPattern(Point(0,0), Point(8,5)) )
      && rangeTest( Factory::createReversedPattern(Point(0,0), Point(5,8)) )
      ;
  }


  {  //range services for 4 adjacency
    typedef DGtal::ArithmeticalDSS<DGtal::int32_t, DGtal::int32_t, 4> DSS4;
    typedef DGtal::ArithmeticalDSSFactory<DGtal::int32_t, DGtal::int32_t, 4> Factory;
    typedef DSS4::Point Point;

    res = res
      && rangeTest( Factory::createPattern(Point(0,0), Point(8,5)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(5,8)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(-8,-5)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(-5,-8)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(5,-8)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(8,-5)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(1,0)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(-1,0)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(0,1)) )
      && rangeTest( Factory::createPattern(Point(0,0), Point(0,-1)) )
      && rangeTest( Factory::createReversedPattern(Point(0,0), Point(8,5)) )
      ;
  }

  {
    typedef DGtal::ArithmeticalDSS<DGtal::int32_t> DSS8;
    typedef DGtal::ArithmeticalDSSFactory<DGtal::int32_t> Factory;
    typedef DSS8::Point Point;
    res = res
      && compatibleStepsTest( Factory::createPattern(Point(0,0), Point(5,0)) )
      && compatibleStepsTest( Factory::createPattern(Point(0,0), Point(-5,0)) )
      && compatibleStepsTest( Factory::createPattern(Point(0,0), Point(0,5)) )
      && compatibleStepsTest( Factory::createPattern(Point(0,0), Point(0,-5)) )
      && compatibleStepsTest( Factory::createPattern(Point(0,0), Point(5,5)) )
      && compatibleStepsTest( Factory::createPattern(Point(0,0), Point(5,-5)) )
      && compatibleStepsTest( Factory::createPattern(Point(0,0), Point(-5,5)) )
      && compatibleStepsTest( Factory::createPattern(Point(0,0), Point(-5,-5)) )
      ;
  }

  {
    typedef DGtal::ArithmeticalDSS<DGtal::int32_t, DGtal::int32_t, 4> DSS4;
    typedef DGtal::ArithmeticalDSSFactory<DGtal::int32_t, DGtal::int32_t, 4> Factory;
    typedef DSS4::Point Point;
    res = res
      && compatibleStepsTest( Factory::createPattern(Point(0,0), Point(5,0)) )
      && compatibleStepsTest( Factory::createPattern(Point(0,0), Point(-5,0)) )
      && compatibleStepsTest( Factory::createPattern(Point(0,0), Point(0,5)) )
      && compatibleStepsTest( Factory::createPattern(Point(0,0), Point(0,-5)) )
      ;
  }

  res = res
    && updateTest<DGtal::ArithmeticalDSS<DGtal::int32_t> >()
#ifdef WITH_BIGINTEGER
    && updateTest<DGtal::ArithmeticalDSS<DGtal::int32_t, DGtal::BigInteger, 4> >()
#endif
    ;

  res = res
    && constructorsTest<DGtal::ArithmeticalDSS<DGtal::int32_t> >()
#ifdef WITH_BIGINTEGER
    && constructorsTest<DGtal::ArithmeticalDSS<DGtal::int32_t, DGtal::BigInteger, 4> >()
#endif
    && constructorsTest<DGtal::NaiveDSS8<DGtal::int32_t> >()
    && constructorsTest<DGtal::StandardDSS4<DGtal::int32_t> >()
    ;

  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
