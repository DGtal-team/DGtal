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
 * @file testStabbingLineComputer.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/09/26
 *
 * @brief Functions for testing class StabbingLineComputer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/geometry/curves/GridCurve.h"

#include "DGtal/geometry/curves/CBidirectionalSegmentComputer.h"

#include "DGtal/geometry/curves/StabbingLineComputer.h"

#include "DGtal/geometry/curves/GreedySegmentation.h"
#include "DGtal/geometry/curves/SaturatedSegmentation.h"

#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/boards/CDrawableWithBoard2D.h"

#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/curves/FreemanChain.h"
#include <fstream>
#include <type_traits>
#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class StabbingLineComputer.
///////////////////////////////////////////////////////////////////////////////
/**
 * Basic methods
 */
template <typename TCurve>
bool testStabbingLineComputer(const TCurve& curve)
{

  typedef typename TCurve::IncidentPointsRange Range; //range
  typedef typename Range::ConstIterator ConstIterator; //iterator
  typedef typename Range::ConstReverseIterator ConstReverseIterator; //reverse iterator

  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Constructors, copy, assignment" );
  {
    Range r = curve.getIncidentPointsRange(); //range

    StabbingLineComputer<ConstIterator> s1, s2, s3;
    s2.init(r.begin());
    s3.init(++r.begin());
    StabbingLineComputer<ConstIterator> s4(s2);
    StabbingLineComputer<ConstIterator> s5(s3);
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

  trace.beginBlock ( "Extension operations" );
  {
    Range r = curve.getIncidentPointsRange(); //range

    StabbingLineComputer<ConstIterator> s, t;

    trace.info() << "forward extension " << endl;
    ConstIterator itBegin (r.begin());
    ConstIterator itEnd (r.end());
    s.init( itBegin+1 );
    while ( (s.end() != itEnd) && (s.isExtendableFront()) && (s.extendFront()) ) {}
    trace.info() << s << endl;
    double a, b, c;
    s.getParameters(a,b,c);
    trace.info() << a << " " << b << " " << c << endl;

    t.init( (itBegin + (itEnd - itBegin)/2) );
    while ( (t.end() != itEnd) && (t.extendFront())
         && (t.begin() != itBegin) && (t.extendBack()) ) {}
    trace.info() << t << endl;

    trace.info() << "backward extension " << endl;
    typename StabbingLineComputer<ConstIterator>::Reverse rs = s.getReverse();
    ConstReverseIterator ritBegin (t.end());
    ConstReverseIterator ritEnd (r.rend());
    rs.init( ritBegin );
    while ( (rs.end() != ritEnd) && (rs.isExtendableFront()) && (rs.extendFront()) ) {}
    trace.info() << rs << endl;
    double ap, bp, cp;
    rs.getParameters(ap,bp,cp);
    trace.info() << ap << " " << bp << " " << cp << endl;

    typename StabbingLineComputer<ConstIterator>::Reverse rt = t.getReverse();
    rt.init( (ritBegin + (ritEnd - ritBegin)/2) );
    while ( (rt.begin() != ritBegin) && (rt.extendBack())
         && (rt.end() != ritEnd) && (rt.extendFront()) ) {}
    trace.info() << rt << endl;

    trace.info() << "comparison... " << endl;
    bool myFlag = ( (s == t)&&(rs == rt) )
    && ( s.Uf() == rs.Uf() )
    && ( s.Ul() == rs.Ul() )
    && ( s.Lf() == rs.Lf() )
    && ( s.Ll() == rs.Ll() )
    && (a == ap)
    && (b == bp)
    && (c == cp)
    ;

    nbok += myFlag ? 1 : 0;
    nb++;
  }
  trace.endBlock();

  trace.info() << "(" << nbok << "/" << nb << ") " << endl;
  return nbok == nb;
}

/**
 * Extension operations with extendBack, on an iterator or a circulator.
 * For each element i of a range of n elements:
 * - the longest segment s starting at i, computed by forward extensions,
 *   must be equal to the segment computed by backward extensions from the
 *   last element of s, bounded by the length of s;
 * - the longest segment u ending at i, computed by backward extensions,
 *   must be equal to the segment computed by forward extensions from the
 *   first element of u, bounded by the length of u.
 * The length of the segments is bounded by n (whole range for a circulator).
 *
 * @param itb begin iterator (or circulator) of the range
 * @param ite end iterator (or circulator) of the range
 * @param n number of elements of the range
 * @tparam SC type of segment computer
 * @tparam I type of iterator or circulator
 */
template <typename SC, typename I>
bool testBackwardExtension(const I& itb, const I& ite, unsigned int n)
{
  const bool isCirculator =
    std::is_same_v<typename IteratorCirculatorTraits<I>::Type, CirculatorType>;
  (void) ite;

  unsigned int nbok = 0;
  unsigned int nb = 0;

  I i( itb );
  for (unsigned int k = 0; k < n; ++k, ++i)
  {
    //forward extensions, then backward extensions
    unsigned int limit = isCirculator ? n : n-k;
    SC s; s.init( i );
    unsigned int ms = 1;
    while ( (ms < limit) && (s.extendFront()) ) ++ms;
    I last( s.end() ); --last;
    SC t; t.init( last );
    unsigned int mt = 1;
    while ( (mt < ms) && (t.extendBack()) ) ++mt;
    bool flag1 = (mt == ms) && (t.begin() == s.begin()) && (t.end() == s.end()) && (s == t);

    //backward extensions, then forward extensions
    limit = isCirculator ? n : k+1;
    SC u; u.init( i );
    unsigned int mu = 1;
    while ( (mu < limit) && (u.extendBack()) ) ++mu;
    SC v; v.init( u.begin() );
    unsigned int mv = 1;
    while ( (mv < mu) && (v.extendFront()) ) ++mv;
    bool flag2 = (mv == mu) && (v.begin() == u.begin()) && (v.end() == u.end()) && (u == v);

    if ( !flag1 )
      trace.info() << "element " << k << ": " << ms << " forward extensions, but "
                   << mt << " backward extensions" << std::endl;
    if ( !flag2 )
      trace.info() << "element " << k << ": " << mu << " backward extensions, but "
                   << mv << " forward extensions" << std::endl;
    nbok += (flag1 && flag2) ? 1 : 0;
    nb++;
  }

  trace.info() << "(" << nbok << "/" << nb << ") " << endl;
  return nbok == nb;
}

/**
 * Extension operations with extendBack on a curve,
 * with iterators and, if the curve is closed, with circulators.
 */
template <typename TCurve>
bool testBackwardExtension(const TCurve& curve, const std::string& name, bool isClosed)
{
  typedef typename TCurve::IncidentPointsRange Range; //range
  typedef typename Range::ConstIterator ConstIterator; //iterator
  typedef typename Range::ConstCirculator ConstCirculator; //circulator

  Range r = curve.getIncidentPointsRange();
  unsigned int n = r.size();
  bool res = true;

  trace.beginBlock ( "Extension operations with extendBack (" + name + ", iterators)" );
  res = res && testBackwardExtension<StabbingLineComputer<ConstIterator> >( r.begin(), r.end(), n );
  trace.endBlock();

  if ( ! isClosed ) return res;

  trace.beginBlock ( "Extension operations with extendBack (" + name + ", circulators)" );
  res = res && testBackwardExtension<StabbingLineComputer<ConstCirculator> >( r.c(), r.c(), n );
  trace.endBlock();

  return res;
}

/*
* simple drawing
*/
template <typename TCurve>
bool drawingTestStabbingLineComputer(const TCurve& curve)
{

  typedef typename TCurve::IncidentPointsRange Range; //range
  typedef typename Range::ConstIterator ConstIterator; //iterator

  Range r = curve.getIncidentPointsRange(); //range

  StabbingLineComputer<ConstIterator> s;
  ConstIterator itEnd (r.end());
  s.init( r.begin() );
  while ( (s.end() != itEnd) && (s.extendFront()) ) {}

  double a, b, c;
  s.getParameters(a,b,c);

  Board2D board;
  board << r << s;
  board.saveEPS("StabbingLineComputerdrawingTest.eps");
  return true;
}

void testStabbingLineComputerConceptChecking()
{
   typedef std::pair<PointVector<2,int>, PointVector<2,int> > Pair;
   typedef std::vector<Pair>::const_iterator ConstIterator;
   typedef StabbingLineComputer<ConstIterator> GeomDSS;
   BOOST_CONCEPT_ASSERT(( concepts::CDrawableWithBoard2D<GeomDSS> ));
   BOOST_CONCEPT_ASSERT(( concepts::CBidirectionalSegmentComputer<GeomDSS> ));
}

template <typename TCurve>
bool testSegmentation(const TCurve& curve)
{

  typedef typename TCurve::IncidentPointsRange Range; //range
  Range r = curve.getIncidentPointsRange(); //range

  typedef typename Range::ConstIterator ConstIterator; //iterator
  typedef StabbingLineComputer<ConstIterator> SegmentComputer; //segment computer

  unsigned int nbok = 0;
  unsigned int nb = 0;


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

    board.saveEPS("StabbingLineComputerGreedySegmentationTest.eps", Board2D::BoundingBox, 5000 );

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

    board.saveEPS("StabbingLineComputerSaturatedSegmentationTest.eps", Board2D::BoundingBox, 5000 );

    trace.info() << r.size() << ";" << n << ";" << suml << endl;
    //comparison with the results gave by another program
    nbok += ((r.size()==85)&&(n==25)&&(suml==255)) ? 1 : 0;
    nb++;
  }
  trace.endBlock();


  trace.info() << "(" << nbok << "/" << nb << ") " << endl;
  return (nbok == nb);
}
///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class StabbingLineComputer" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res;

  {//concept checking
    testStabbingLineComputerConceptChecking();
  }

  {//basic operations
    std::string filename = testPath + "samples/DSS.dat";
    ifstream instream; // input stream
    instream.open (filename.c_str(), ifstream::in);

    typedef KhalimskySpaceND<2,int> KSpace;
    GridCurve<KSpace> c; //grid curve
    c.initFromVectorStream(instream);

    res = testStabbingLineComputer(c)
  && drawingTestStabbingLineComputer(c);
  }

  {//extension operations with extendBack on an open curve
    std::string filename = testPath + "samples/manche.fc";
    std::fstream fst( filename.c_str(), std::ios::in );
    FreemanChain<int> fc( fst );
    std::vector<Z2i::Point> points( fc.begin(), fc.end() );
    Z2i::Curve c;
    c.initFromPointsVector( points );

    res = res && testBackwardExtension( c, "manche.fc", false );
  }

  {//extension operations with extendBack on a closed curve
    std::string filename = testPath + "samples/contourS.fc";
    std::fstream fst( filename.c_str(), std::ios::in );
    FreemanChain<int> fc( fst );
    std::vector<Z2i::Point> points( fc.begin(), fc.end() );
    points.pop_back(); //the last point is equal to the first one
    Z2i::Curve c;
    c.initFromPointsVector( points );

    res = res && testBackwardExtension( c, "contourS.fc", true );
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
