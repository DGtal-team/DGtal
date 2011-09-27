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
 * @file testGeometricalDSS.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/09/26
 *
 * @brief Functions for testing class GeometricalDSS.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/geometry/2d/GridCurve.h"
#include "DGtal/geometry/2d/GeometricalDSS.h"

#include "DGtal/geometry/CBidirectionalSegmentComputer.h"
#include "DGtal/io/boards/CDrawableWithBoard2D.h"

#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class GeometricalDSS.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
template <typename TCurve>
bool testGeometricalDSS(const TCurve& c)
{

  typedef typename TCurve::IncidentPointsRange Range; //range
  typedef typename Range::ConstIterator ConstIterator; //iterator
  typedef typename Range::ConstReverseIterator ConstReverseIterator; //reverse iterator

  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing block ..." );

  Range r = c.getIncidentPointsRange(); //range

  GeometricalDSS<ConstIterator> s;
  typename GeometricalDSS<ConstIterator>::Reverse rs = s.getReverse(); 

  trace.info() << "forward extension " << endl; 
  ConstIterator itBegin (r.begin()); 
  ConstIterator itEnd (r.end()); 
  s.init( itBegin );
  while ( (s.end() != itEnd) && (s.isExtendable()) && (s.extend()) ) {}
  trace.info() << s << endl; 

  trace.info() << "backward extension " << endl; 
  ConstReverseIterator ritBegin (r.rbegin()); //iterators
  ConstReverseIterator ritEnd (r.rend()); 
  rs.init( ritBegin );
  while ( (rs.end() != ritEnd) && (rs.isExtendable()) && (rs.extend()) ) {}
  trace.info() << rs << endl; 

  nbok += true ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "true == true" << endl;
  trace.endBlock();
  
  return nbok == nb;
}

void testGeometricalDSSConceptChecking()
{
   typedef std::pair<PointVector<2,int>, PointVector<2,int> > Pair; 
   typedef std::vector<Pair>::const_iterator ConstIterator; 
   typedef GeometricalDSS<ConstIterator> GeomDSS; 
   BOOST_CONCEPT_ASSERT(( CDrawableWithBoard2D<GeomDSS> ));
   BOOST_CONCEPT_ASSERT(( CBidirectionalSegmentComputer<GeomDSS> ));
}


///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class GeometricalDSS" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  testGeometricalDSSConceptChecking();

  std::string filename = testPath + "samples/DSS.dat";
  ifstream instream; // input stream
  instream.open (filename.c_str(), ifstream::in);
  
  typedef KhalimskySpaceND<2,int> KSpace; 
  GridCurve<KSpace> c; //grid curve
  c.initFromVectorStream(instream);

  bool res = testGeometricalDSS(c)
; // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
