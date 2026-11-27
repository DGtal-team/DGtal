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
 * @file testSegmentComputerEstimators.cpp
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 *
 * @date 2011/06/28

 * Functions for testing classes in SegmentComputerEstimators.h
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <cmath>
#include "DGtal/base/Common.h"


#include "DGtal/geometry/curves/ArithmeticalDSSComputer.h"
#include "DGtal/geometry/curves/StabbingLineComputer.h"
#include "DGtal/geometry/curves/StabbingCircleComputer.h"
#include "DGtal/geometry/curves/estimation/SegmentComputerEstimators.h"
#include "DGtal/geometry/curves/FreemanChain.h"
#include "DGtal/geometry/curves/GridCurve.h"
#include "DGtal/geometry/curves/SaturatedSegmentation.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include <fstream>

#include "DGtal/io/boards/Board2D.h"


#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace LibBoard;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing classes in SegmentComputerEstimators.h.
///////////////////////////////////////////////////////////////////////////////


/**
 * Tangent from DSS
 *
 */
template<typename DSSComputer>
bool testTangentFromDSS(
     const typename DSSComputer::ConstIterator& itb,
     const typename DSSComputer::ConstIterator& ite  )
{


  trace.info() << "feeding segment computer " << endl;

  DSSComputer dss;
  DGtal::longestSegment( dss, itb, ite );

  trace.info() << dss << endl;
  trace.info() << endl;

  trace.info() << "building and using the functor " << endl;
  const double epsilon = 0.00001;

  int nb = 0;
  int nbok = 0;

  { //angle
    TangentAngleFromDSSEstimator<DSSComputer> f;
    f.attach(dss);
    double q1 = f.eval(itb);
    double q2 = std::atan2((double)dss.a(),(double)dss.b());
    trace.info() << "Tangent orientation : " << q1 << " == " << q2 << endl;
    nbok += (std::abs(q1 - q2) < epsilon)?1:0;
    nb++;
    trace.info() << "(" << nbok << "/" << nb << ")" << std::endl;
  }

  { //vector
    TangentVectorFromDSSEstimator<DSSComputer> f;
    typedef typename TangentVectorFromDSSEstimator<DSSComputer>::Quantity Quantity;
    f.attach(dss);
    Quantity q1 = f.eval(itb);
    Quantity q2 = Quantity(dss.b(), dss.a());
    trace.info() << "Tangent vector : " << q1 << " == " << q2 << endl;
    nbok += (q1 == q2)?1:0;
    nb++;
    trace.info() << "(" << nbok << "/" << nb << ")" << std::endl;
  }

  { //unit vector
    TangentFromDSSEstimator<DSSComputer> f;
    typedef typename TangentFromDSSEstimator<DSSComputer>::Quantity Quantity;
    f.attach(dss);
    Quantity q1 = f.eval(itb);
    double n = std::sqrt( (double)dss.a()*dss.a() + (double)dss.b()*dss.b() );
    Quantity q2 = Quantity((double)dss.b()/n, (double)dss.a()/n);
    trace.info() << "Normalized tangent vector : " << q1 << " == " << q2 << endl;
    nbok += ((std::abs(q1[0] - q2[0]) < epsilon)&&(std::abs(q1[1] - q2[1]) < epsilon))?1:0;
    nb++;
    trace.info() << "(" << nbok << "/" << nb << ")" << std::endl;
  }

  return (nb == nbok);
}

/**
 * Estimations from DCA
 *
 */
template<typename DCAComputer>
bool testFromDCA(
     const typename DCAComputer::ConstIterator& itb,
     const typename DCAComputer::ConstIterator& ite  )
{


  trace.info() << "feeding segment computer " << endl;

  DCAComputer dca;
  DGtal::longestSegment( dca, itb, ite );

  trace.info() << dca << endl;
  trace.info() << endl;

  trace.info() << "building and using the functor " << endl;
  const double epsilon = 0.00001;

  int nb = 0;
  int nbok = 0;

  { //curvature: the curve of the test is a convex part scanned in a
    //clockwise orientation (the inner points are on the right),
    //so that its curvature is negative
    CurvatureFromDCAEstimator<DCAComputer,false> f;
    f.init(1.0, itb, ite);
    f.attach(dca);
    double q1 = f.eval(itb);
    f.init(0.1, itb, ite);
    double q2 = f.eval(itb);
    trace.info() << "Curvature (h=1): " << q1 << std::endl;
    trace.info() << "Curvature (h=0.1): " << q2 << std::endl;
    nbok += ((q1 <= 0)&&(q1 > -1)&&(q2 <= 0)&&(q2 > -10)
	     &&(std::abs(q2-(10*q1)) < epsilon))?1:0;
    nb++;
    trace.info() << "(" << nbok << "/" << nb << ")" << std::endl;
  }

  { //tangent
    TangentFromDCAEstimator<DCAComputer> f;
    f.attach(dca);
    typedef typename TangentFromDCAEstimator<DCAComputer>::Quantity Quantity;
    typename DCAComputer::ConstIterator it = DGtal::subRangeMiddle(itb, ite);
    Quantity q1 = f.eval( it );
    trace.info() << "Tangent: " << q1 << " == [PointVector] {1, 0} " << std::endl;
    nbok += (std::abs(q1[0]-1.0) < epsilon)
      && (std::abs(q1[1]) < epsilon) ? 1 : 0;
    nb++;
    trace.info() << "(" << nbok << "/" << nb << ")" << std::endl;
  }

  { //position
    DistanceFromDCAEstimator<DCAComputer> f;
    typedef typename DistanceFromDCAEstimator<DCAComputer>::Quantity Quantity;
    typename DCAComputer::ConstIterator it = DGtal::subRangeMiddle(itb, ite);
    f.init(1.0, itb, ite);
    f.attach(dca);
    Quantity q1 = f.eval( it );
    trace.info() << "Position (h=1): " << q1.first << " (<=0), " << q1.second << " (>0) " << std::endl;
    f.init(0.1, itb, ite);
    f.attach(dca);
    Quantity q2 = f.eval( it );
    trace.info() << "Position (h=0.1): " << q2.first << " (<=0), " << q2.second << " (>0) " << std::endl;
    nbok += ( (q1.first < epsilon)
	      && (q1.second > -epsilon)
	      && (std::abs((q1.second - q1.first) - 1) < epsilon)
	      && (q2.first < epsilon)
	      && (q2.second > -epsilon)
	      && (std::abs((q2.second - q2.first) - 0.1) < epsilon) )? 1 : 0;
    nb++;
    trace.info() << "(" << nbok << "/" << nb << ")" << std::endl;
  }

  return (nb == nbok);
}


///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

/**
 * Checks that the normal vectors estimated from the maximal DCAs of a
 * closed non-convex curve (contourS.fc) always point toward the outside,
 * i.e. from the inner point toward the outer point of each step.
 */
bool testNormalFromDCAOrientation()
{
  trace.beginBlock ( "Orientation of the normal and tangent vectors of DCAs" );

  typedef KhalimskySpaceND<2,int> KSpace;
  typedef KSpace::Point Point;
  std::fstream fst( (testPath + "samples/contourS.fc").c_str(), std::ios::in );
  FreemanChain<int> fc( fst );
  std::vector<Point> points( fc.begin(), fc.end() );
  points.pop_back(); //the last point is equal to the first one
  GridCurve<KSpace> curve;
  curve.initFromPointsVector( points );
  typedef GridCurve<KSpace>::IncidentPointsRange Range;
  Range r = curve.getIncidentPointsRange();
  typedef Range::ConstCirculator ConstCirculator;
  typedef StabbingCircleComputer<ConstCirculator> DCAComputer;
  typedef NormalFromDCAEstimator<DCAComputer> Normal;
  typedef TangentFromDCAEstimator<DCAComputer> Tangent;
  typedef CurvatureFromDCAEstimator<DCAComputer, true> CCWCurvature;
  typedef CurvatureFromDCAEstimator<DCAComputer, false> CWCurvature;

  //orientation of the curve (sign of its area)
  double area = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i)
    {
      Point p = points[i], q = points[(i+1) % points.size()];
      area += double( p[0] ) * q[1] - double( q[0] ) * p[1];
    }
  const bool isCCW = ( area > 0.0 );

  unsigned int nbok = 0;
  unsigned int nb = 0;
  ConstCirculator c = r.c();
  SaturatedSegmentation<DCAComputer> segmentation( c, c, DCAComputer() );
  bool hasConvexPart = false, hasConcavePart = false;
  for (auto seg = segmentation.begin(); seg != segmentation.end(); ++seg)
    {
      Normal f;
      f.attach( *seg );
      Tangent ft;
      ft.attach( *seg );
      ConstCirculator it = seg->begin();
      Normal::Quantity previous;
      do
        {
          auto pair = *it;
          Normal::Quantity n = f.eval( it );
          (void) previous;
          Normal::Quantity d( pair.second[0] - pair.first[0],
                              pair.second[1] - pair.first[1] );
          //the normal vector may be orthogonal to the pair of points
          //if the separating shape passes through both points
          bool ok = ( n.dot( d ) >= -1e-9 );
          if ( ! ok )
            trace.info() << "  wrong orientation: straight " << seg->isStraight()
                         << " convex " << seg->isConvex() << " n " << n << " d " << d << std::endl;
          nbok += ok ? 1 : 0;
          nb++;
          //orientation consistent with the previous normal vector of the segment
          if ( it != seg->begin() )
            {
              bool consistent = ( n.dot( previous ) > 0 );
              if ( ! consistent )
                trace.info() << "  inconsistent orientation: straight " << seg->isStraight()
                             << " n " << n << " previous " << previous << std::endl;
              nbok += consistent ? 1 : 0;
              nb++;
            }
          previous = n;
          //tangent vector: orthogonal to the normal vector and oriented
          //along the sequence of pairs (toward the next pair)
          Tangent::Quantity t = ft.eval( it );
          nbok += ( std::abs( t.dot( n ) ) < 1e-9 ) ? 1 : 0;
          nb++;
          ConstCirculator next( it ); ++next;
          if ( next != seg->end() )
            {
              auto pair2 = *next;
              Normal::Quantity e( (pair2.first[0]+pair2.second[0]) - (pair.first[0]+pair.second[0]),
                                  (pair2.first[1]+pair2.second[1]) - (pair.first[1]+pair.second[1]) );
              bool forward = ( t.dot( e ) > 0 );
              if ( ! forward )
                trace.info() << "  tangent not forward: t " << t << " e " << e << std::endl;
              nbok += forward ? 1 : 0;
              nb++;
            }
          ++it;
        } while ( it != seg->end() );
      if ( ! seg->isStraight() )
        {
          //the inner points are closer to the center of the separating
          //circle than the outer points for a convex part, farther for a
          //concave part
          double c0, c1, rad;
          seg->getSeparatingCircle().getParameters( c0, c1, rad );
          double s = 0.0;
          ConstCirculator it2 = seg->begin();
          do
            {
              auto pair = *it2;
              s += ( (pair.second[0]-c0)*(pair.second[0]-c0) + (pair.second[1]-c1)*(pair.second[1]-c1) )
                - ( (pair.first[0]-c0)*(pair.first[0]-c0) + (pair.first[1]-c1)*(pair.first[1]-c1) );
              ++it2;
            } while ( it2 != seg->end() );
          if ( s > 0 ) hasConvexPart = true; else hasConcavePart = true;
          //sign of the curvature: positive on convex parts and negative on
          //concave parts, given the orientation of the scanning
          CCWCurvature fccw; fccw.init( 1.0, seg->begin(), seg->end() ); fccw.attach( *seg );
          CWCurvature fcw; fcw.init( 1.0, seg->begin(), seg->end() ); fcw.attach( *seg );
          double k = isCCW ? fccw.eval( seg->begin() ) : fcw.eval( seg->begin() );
          bool goodSign = ( (k > 0) == seg->isConvex() ) && ( k != 0 );
          if ( ! goodSign )
            trace.info() << "  wrong curvature sign: " << k << " convex " << seg->isConvex() << std::endl;
          nbok += goodSign ? 1 : 0;
          nb++;
          //opposite sign for the opposite orientation
          double k2 = isCCW ? fcw.eval( seg->begin() ) : fccw.eval( seg->begin() );
          nbok += ( std::abs( k + k2 ) < 1e-12 ) ? 1 : 0;
          nb++;
          nbok += ( (s > 0) == seg->isConvex() ) ? 1 : 0;
          nb++;
        }
    }
  trace.info() << "(" << nbok << "/" << nb << ") normal vectors toward the outside, "
               << "convex parts: " << hasConvexPart << ", concave parts: " << hasConcavePart
               << std::endl;
  trace.endBlock();
  return ( nbok == nb ) && hasConvexPart && hasConcavePart;
}

/**
 * Normal vectors of the maximal segments of StabbingLineComputer along a
 * closed non-convex curve: they must be unit vectors orthogonal to the
 * separating straight line, oriented from the inner points toward the
 * outer points, and consistently oriented from a maximal segment to the
 * next one (which overlap).
 */
bool testNormalFromStabbingLine()
{
  trace.beginBlock ( "Normal vectors of the segments of StabbingLineComputer" );

  typedef KhalimskySpaceND<2,int> KSpace;
  typedef KSpace::Point Point;
  std::fstream fst( (testPath + "samples/contourS.fc").c_str(), std::ios::in );
  FreemanChain<int> fc( fst );
  std::vector<Point> points( fc.begin(), fc.end() );
  points.pop_back(); //the last point is equal to the first one
  GridCurve<KSpace> curve;
  curve.initFromPointsVector( points );
  typedef GridCurve<KSpace>::IncidentPointsRange Range;
  Range r = curve.getIncidentPointsRange();
  typedef Range::ConstCirculator ConstCirculator;
  typedef StabbingLineComputer<ConstCirculator> LineComputer;
  typedef NormalFromStabbingLineEstimator<LineComputer> Normal;

  unsigned int nbok = 0;
  unsigned int nb = 0;
  ConstCirculator c = r.c();
  SaturatedSegmentation<LineComputer> segmentation( c, c, LineComputer() );
  Normal::Quantity first, previous;
  bool isFirst = true;
  for (auto seg = segmentation.begin(); seg != segmentation.end(); ++seg)
    {
      Normal f;
      f.init( 1.0, seg->begin(), seg->end() );
      f.attach( *seg );
      Normal::Quantity n = f.eval( seg->begin() );
      //unit vector of direction (alpha, beta), up to its orientation
      double alpha, beta, gamma;
      seg->getParameters( alpha, beta, gamma );
      nbok += ( std::abs( n.norm() - 1.0 ) < 1e-9 ) ? 1 : 0;
      nb++;
      nbok += ( std::abs( n[0]*beta - n[1]*alpha ) < 1e-9 ) ? 1 : 0;
      nb++;
      //from the inner points toward the outer points
      bool outside = true, strictly = false;
      ConstCirculator it = seg->begin();
      do
        {
          auto pair = *it;
          double s = n[0] * ( pair.second[0] - pair.first[0] )
            + n[1] * ( pair.second[1] - pair.first[1] );
          outside = outside && ( s >= -1e-9 );
          strictly = strictly || ( s > 1e-9 );
          ++it;
        } while ( it != seg->end() );
      if ( ! ( outside && strictly ) )
        trace.info() << "  wrong orientation: n " << n << std::endl;
      nbok += ( outside && strictly ) ? 1 : 0;
      nb++;
      //consistent with the normal vector of the previous maximal segment
      if ( ! isFirst )
        {
          nbok += ( n.dot( previous ) > 0 ) ? 1 : 0;
          nb++;
        }
      else
        first = n;
      isFirst = false;
      previous = n;
    }
  nbok += ( previous.dot( first ) > 0 ) ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") normal vectors" << std::endl;
  trace.endBlock();
  return ( nbok == nb ) && ( nb > 10 );
}

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing segment computer functors" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = true;

  //types
  typedef PointVector<2,int> Point;


  //------------------------------------------ DSS
  {
    typedef std::vector<Point> Range;
    typedef Range::iterator ConstIterator;
    typedef ArithmeticalDSSComputer<ConstIterator,int,4> DSS4;
    typedef ArithmeticalDSSComputer<ConstIterator,int,8> DSS8;

    //input points
    Range curve4;
    curve4.push_back(Point(0,0));
    curve4.push_back(Point(1,0));
    curve4.push_back(Point(1,1));
    curve4.push_back(Point(2,1));
    curve4.push_back(Point(3,1));
    curve4.push_back(Point(3,2));
    curve4.push_back(Point(4,2));
    curve4.push_back(Point(5,2));
    curve4.push_back(Point(6,2));
    curve4.push_back(Point(6,3));
    curve4.push_back(Point(7,3));

    Range curve8;
    curve8.push_back(Point(0,0));
    curve8.push_back(Point(1,1));
    curve8.push_back(Point(2,1));
    curve8.push_back(Point(3,2));
    curve8.push_back(Point(4,2));
    curve8.push_back(Point(5,2));
    curve8.push_back(Point(6,3));
    curve8.push_back(Point(7,3));
    curve8.push_back(Point(8,4));
    curve8.push_back(Point(9,4));
    curve8.push_back(Point(10,5));

    //tests
    res = res && testTangentFromDSS<DSS4>(curve4.begin(), curve4.end())
      && testTangentFromDSS<DSS8>(curve8.begin(), curve8.end());
  }
  //------------------------------------------ DCA
  {
    typedef std::pair<Point,Point> Pair;
    typedef std::vector<Pair> Range;
    typedef Range::const_iterator ConstIterator;
    typedef StabbingCircleComputer<ConstIterator> DCA;

    Range curve;
    curve.push_back(std::make_pair(Point(0,0),Point(0,1)));
    curve.push_back(std::make_pair(Point(1,0),Point(1,1)));

    curve.push_back(std::make_pair(Point(2,1),Point(2,2)));
    curve.push_back(std::make_pair(Point(3,1),Point(3,2)));
    curve.push_back(std::make_pair(Point(4,1),Point(4,2)));

    curve.push_back(std::make_pair(Point(5,2),Point(5,3)));
    curve.push_back(std::make_pair(Point(6,2),Point(6,3)));
    curve.push_back(std::make_pair(Point(7,2),Point(7,3)));
    curve.push_back(std::make_pair(Point(8,2),Point(8,3)));
    curve.push_back(std::make_pair(Point(9,2),Point(9,3)));

    curve.push_back(std::make_pair(Point(10,1),Point(10,2)));
    curve.push_back(std::make_pair(Point(11,1),Point(11,2)));
    curve.push_back(std::make_pair(Point(12,1),Point(12,2)));

    curve.push_back(std::make_pair(Point(13,0),Point(13,1)));
    curve.push_back(std::make_pair(Point(14,0),Point(14,1)));

    res = res && testFromDCA<DCA>(curve.begin(), curve.end())
      && testFromDCA<DCA>(curve.begin()+2, curve.begin()+12);
    res = res && testNormalFromDCAOrientation();
    res = res && testNormalFromStabbingLine();
  }

  //----------------------------------------------------------
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();

  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
