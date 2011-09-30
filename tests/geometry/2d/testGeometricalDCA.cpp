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
 * @file testGeometricalDCA.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/09/26
 *
 * @brief Functions for testing class GeometricalDCA.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/geometry/2d/GridCurve.h"

#include "DGtal/geometry/CBidirectionalSegmentComputer.h"
#include "DGtal/io/boards/CDrawableWithBoard2D.h"

#include "DGtal/geometry/2d/GeometricalDCA.h"

#include "DGtal/geometry/2d/GreedySegmentation.h"
#include "DGtal/geometry/2d/SaturatedSegmentation.h"


#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class GeometricalDCA.
///////////////////////////////////////////////////////////////////////////////
/**
 * Basic methods
 */
template <typename TCurve>
bool testGeometricalDCA(const TCurve& curve)
{

  typedef typename TCurve::IncidentPointsRange Range; //range
  typedef typename Range::ConstIterator ConstIterator; //iterator
  typedef typename Range::ConstReverseIterator ConstReverseIterator; //reverse iterator

  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Constructors, copy, assignement" );
  {
    Range r = curve.getIncidentPointsRange(); //range

    GeometricalDCA<ConstIterator> s1, s2, s3;
    s2.init(r.begin()); 
    s3.init(++r.begin()); 
    GeometricalDCA<ConstIterator> s4(s2); 
    GeometricalDCA<ConstIterator> s5(s3);
    s3 = s1; 
    
    trace.info() << s1.isValid() << s1 << endl; 
    trace.info() << s2.isValid() << s2 << endl; 
    trace.info() << s3.isValid() << s3 << endl; 
    trace.info() << s4.isValid() << s4 << endl; 
    trace.info() << s5.isValid() << s5 << endl; 

    bool myFlag = (!s1.isValid())&&(!s3.isValid())
    &&(s2.isValid())&&(s4.isValid())&&(s5.isValid())
    &&(s2 == s4)&&(s3 != s5)&&(s1 == s3)&&(s2 != s5);

    nbok += myFlag ? 1 : 0; 
    nb++;
  }
  trace.endBlock();
  /*
  trace.beginBlock ( "Extension operations" );
  {
    Range r = curve.getIncidentPointsRange(); //range

    GeometricalDCA<ConstIterator> s, t;

    trace.info() << "forward extension " << endl; 
    ConstIterator itBegin (r.begin()); 
    ConstIterator itEnd (r.end()); 
    s.init( itBegin+1 );
    while ( (s.end() != itEnd) && (s.isExtendable()) && (s.extend()) ) {}
    trace.info() << s << endl; 
    double a, b, c; 
    s.getParameters(a,b,c); 
    trace.info() << a << " " << b << " " << c << endl; 

    t.init( (itBegin + (itEnd - itBegin)/2) ); 
    while ( (t.end() != itEnd) && (t.extend()) 
         && (t.begin() != itBegin) && (t.extendOppositeEnd()) ) {}
    trace.info() << t << endl; 

    trace.info() << "backward extension " << endl; 
    typename GeometricalDCA<ConstIterator>::Reverse rs = s.getReverse(); 
    ConstReverseIterator ritBegin (t.end()); 
    ConstReverseIterator ritEnd (r.rend()); 
    rs.init( ritBegin );
    while ( (rs.end() != ritEnd) && (rs.isExtendable()) && (rs.extend()) ) {}
    trace.info() << rs << endl; 
    double ap, bp, cp; 
    rs.getParameters(ap,bp,cp); 
    trace.info() << ap << " " << bp << " " << cp << endl; 

    typename GeometricalDCA<ConstIterator>::Reverse rt = t.getReverse(); 
    rt.init( (ritBegin + (ritEnd - ritBegin)/2) ); 
    while ( (rt.begin() != ritBegin) && (rt.extendOppositeEnd())
         && (rt.end() != ritEnd) && (rt.extend()) ) {}
    trace.info() << rt << endl; 

    trace.info() << "comparison... " << endl; 
    bool myFlag = ( (s == t)&&(rs == rt) )
    && ( s.getUf() == rs.getUf() )
    && ( s.getUl() == rs.getUl() )
    && ( s.getLf() == rs.getLf() )
    && ( s.getLl() == rs.getLl() )
    && (a == ap)
    && (b == bp)
    && (c == cp)
    ; 

    nbok += myFlag ? 1 : 0; 
    nb++;
  }
  trace.endBlock();
  */
  trace.info() << "(" << nbok << "/" << nb << ") " << endl;
  return nbok == nb;
}

/*
* simple drawing
*/

template <typename TCurve>
bool drawingTestGeometricalDCA(const TCurve& curve)
{

  typedef typename TCurve::IncidentPointsRange Range; //range
  typedef typename Range::ConstIterator ConstIterator; //iterator

  Range r = curve.getIncidentPointsRange(); //range

  GeometricalDCA<ConstIterator> s;
  ConstIterator itEnd (r.end()); 
  s.init( r.begin() );
  while ( (s.end() != itEnd) && (s.extend()) ) {}

  trace.info() << s << endl; 

  Board2D board; 
  board << r << s; 
  board.saveEPS("GeometricalDCAdrawingTest.eps"); 

  return true; 
}

void testGeometricalDCAConceptChecking()
{
   typedef std::pair<PointVector<2,int>, PointVector<2,int> > Pair; 
   typedef std::vector<Pair>::const_iterator ConstIterator; 
   typedef GeometricalDCA<ConstIterator> GeomDSS; 
   BOOST_CONCEPT_ASSERT(( CDrawableWithBoard2D<GeomDSS> ));
   BOOST_CONCEPT_ASSERT(( CBidirectionalSegmentComputer<GeomDSS> ));
}

template <typename TCurve>
bool testSegmentation(const TCurve& curve)
{

  typedef typename TCurve::IncidentPointsRange Range; //range
  Range r = curve.getIncidentPointsRange(); //range
  
  typedef typename Range::ConstIterator ConstIterator; //iterator
  typedef GeometricalDCA<ConstIterator> SegmentComputer; //segment computer
  
  unsigned int nbok = 0;
  unsigned int nb = 0;  

  /*
  trace.beginBlock ( "Greedy segmentation" );
  {
    typedef GreedySegmentation<SegmentComputer> Segmentation;
    Segmentation theSegmentation( r.begin(), r.end(), SegmentComputer() );
    
    Board2D board; 
    board << r; 
      
    typename Segmentation::SegmentComputerIterator it = theSegmentation.begin();
    typename Segmentation::SegmentComputerIterator itEnd = theSegmentation.end();
    unsigned int n = 0; 
    unsigned int suml = 0; 
    for ( ; it != itEnd; ++it, ++n) {
      board << (*it); 
      for (ConstIterator i = it->begin(); i != it->end(); ++i)
        suml += 1; 
    }
    
    board.saveEPS("GeometricalDCAGreedySegmentationTest.eps", Board2D::BoundingBox, 5000 ); 

    trace.info() << r.size() << ";" << n << ";" << suml << endl;
    //comparison with the results gave by another program
    nbok += ((r.size()==85)&&(n==10)&&(suml==94)) ? 1 : 0; 
    nb++;
  }
  trace.endBlock();

  trace.beginBlock ( "Saturated segmentation" );
  {
    typedef SaturatedSegmentation<SegmentComputer> Segmentation;
    Segmentation theSegmentation( r.begin(), r.end(), SegmentComputer() );
    
    Board2D board; 
    board << r; 
    
    typename Segmentation::SegmentComputerIterator it = theSegmentation.begin();
    typename Segmentation::SegmentComputerIterator itEnd = theSegmentation.end();
    unsigned int n = 0; 
    unsigned int suml = 0; 
    for ( ; it != itEnd; ++it, ++n) {
      board << (*it); 
      for (ConstIterator i = it->begin(); i != it->end(); ++i)
        suml += 1; 
    }
    
    board.saveEPS("GeometricalDCASaturatedSegmentationTest.eps", Board2D::BoundingBox, 5000 ); 

    trace.info() << r.size() << ";" << n << ";" << suml << endl;
    //comparison with the results gave by another program
    nbok += ((r.size()==85)&&(n==25)&&(suml==255)) ? 1 : 0; 
    nb++;
  }
  trace.endBlock();
  */
  
  trace.info() << "(" << nbok << "/" << nb << ") " << endl;
  return (nbok == nb);
}
///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class GeometricalDCA" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res; 
  
  {//concept checking
    testGeometricalDCAConceptChecking();
  }
  
  {//basic operations
    std::string filename = testPath + "samples/DCA.dat";
    ifstream instream; // input stream
    instream.open (filename.c_str(), ifstream::in);
    
    typedef KhalimskySpaceND<2,int> KSpace; 
    GridCurve<KSpace> c; //grid curve
    c.initFromVectorStream(instream);

    res = testGeometricalDCA(c)
  && drawingTestGeometricalDCA(c); 
  }
  
  {//segmentations
    std::string filename = testPath + "samples/sinus2D4.dat";
    ifstream instream; // input stream
    instream.open (filename.c_str(), ifstream::in);
    
    typedef KhalimskySpaceND<2,int> KSpace; 
    GridCurve<KSpace> c; //grid curve
    c.initFromVectorStream(instream);
    
    res = res && testSegmentation(c); 
  }
  
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
