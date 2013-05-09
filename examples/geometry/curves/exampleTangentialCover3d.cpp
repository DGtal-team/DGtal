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
 * @file exampleTangentialCover3d.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 *
 * @date 2013/05/09
 *
 * This file is part of the DGtal library
 */

/**
 * Description of exampleTangentialCover3d <p>
 */




#include <iostream>
#include <iterator>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <vector>

#include <QtGui/qapplication.h>
#include "DGtal/base/Common.h"
#include "DGtal/base/Exceptions.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/geometry/curves/GridCurve.h"
#include "DGtal/geometry/curves/ArithmeticalDSS3d.h"
#include "DGtal/geometry/curves/SaturatedSegmentation.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/readers/PointListReader.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"


using namespace DGtal;
using namespace std;

const Color  AXIS_COLOR( 0, 0, 0, 255 );
const double AXIS_LINESIZE = 2.0;
const Color  XY_COLOR( 0, 0, 255, 50 );
const Color  XZ_COLOR( 0, 255, 0, 50 );
const Color  YZ_COLOR( 255, 0, 0, 50 );
const Color  CURVE3D_COLOR( 100, 100, 140, 128 );
const Color  CURVE2D_COLOR( 200, 200, 200, 100 );
const double MS3D_LINESIZE = 3.0;
const int    MS3D_NBCOLORS = 3;

///////////////////////////////////////////////////////////////////////////////
// Functions for displaying the tangential cover of a 3D curve.
///////////////////////////////////////////////////////////////////////////////
template <typename Point, typename RealPoint>
void displayAxes( Viewer3D & viewer, 
                  const Point & lowerBound, const Point & upperBound )
{
  RealPoint p0( (double)lowerBound[ 0 ]-0.5,
                (double)lowerBound[ 1 ]-0.5,
                (double)lowerBound[ 2 ]-0.5 );
  RealPoint p1( (double)upperBound[ 0 ]-0.5,
                (double)upperBound[ 1 ]-0.5,
                (double)upperBound[ 2 ]-0.5 );
  viewer.addLine( p0[ 0 ], p0[ 1 ], p0[ 2 ], p1[ 0 ], p0[ 1 ], p0[ 2 ], AXIS_COLOR, AXIS_LINESIZE );
  viewer.addLine( p0[ 0 ], p0[ 1 ], p0[ 2 ], p0[ 0 ], p1[ 1 ], p0[ 2 ], AXIS_COLOR, AXIS_LINESIZE );
  viewer.addLine( p0[ 0 ], p0[ 1 ], p0[ 2 ], p0[ 0 ], p0[ 1 ], p1[ 2 ], AXIS_COLOR, AXIS_LINESIZE );
  viewer.addLine( p1[ 0 ], p0[ 1 ], p0[ 2 ], p1[ 0 ], p1[ 1 ], p0[ 2 ], AXIS_COLOR, AXIS_LINESIZE );
  viewer.addLine( p1[ 0 ], p0[ 1 ], p0[ 2 ], p1[ 0 ], p0[ 1 ], p1[ 2 ], AXIS_COLOR, AXIS_LINESIZE );
  viewer.addLine( p0[ 0 ], p1[ 1 ], p0[ 2 ], p1[ 0 ], p1[ 1 ], p0[ 2 ], AXIS_COLOR, AXIS_LINESIZE );
  viewer.addLine( p0[ 0 ], p1[ 1 ], p0[ 2 ], p0[ 0 ], p1[ 1 ], p1[ 2 ], AXIS_COLOR, AXIS_LINESIZE );
  viewer.addLine( p0[ 0 ], p0[ 1 ], p1[ 2 ], p1[ 0 ], p0[ 1 ], p1[ 2 ], AXIS_COLOR, AXIS_LINESIZE );
  viewer.addLine( p0[ 0 ], p0[ 1 ], p1[ 2 ], p0[ 0 ], p1[ 1 ], p1[ 2 ], AXIS_COLOR, AXIS_LINESIZE );
  viewer.addLine( p1[ 0 ], p1[ 1 ], p0[ 2 ], p1[ 0 ], p1[ 1 ], p1[ 2 ], AXIS_COLOR, AXIS_LINESIZE );
  viewer.addLine( p1[ 0 ], p0[ 1 ], p1[ 2 ], p1[ 0 ], p1[ 1 ], p1[ 2 ], AXIS_COLOR, AXIS_LINESIZE );
  viewer.addLine( p0[ 0 ], p1[ 1 ], p1[ 2 ], p1[ 0 ], p1[ 1 ], p1[ 2 ], AXIS_COLOR, AXIS_LINESIZE );
  viewer.addQuad( p1[ 0 ], p1[ 1 ], p1[ 2 ], p1[ 0 ], p0[ 1 ], p1[ 2 ],
                  p0[ 0 ], p0[ 1 ], p1[ 2 ], p0[ 0 ], p1[ 1 ], p1[ 2 ], XY_COLOR );
  viewer.addQuad( p1[ 0 ], p1[ 1 ], p1[ 2 ], p0[ 0 ], p1[ 1 ], p1[ 2 ],
                  p0[ 0 ], p1[ 1 ], p0[ 2 ], p1[ 0 ], p1[ 1 ], p0[ 2 ], XZ_COLOR );
  viewer.addQuad( p1[ 0 ], p1[ 1 ], p1[ 2 ], p1[ 0 ], p0[ 1 ], p1[ 2 ],
                  p1[ 0 ], p0[ 1 ], p0[ 2 ], p1[ 0 ], p1[ 1 ], p0[ 2 ], YZ_COLOR );
}

template <typename KSpace, typename ArithmeticalDSS3d>
void displayDSS3d( Viewer3D & viewer, 
		   const KSpace & ks, const ArithmeticalDSS3d & dss3d,
		   const DGtal::Color & color3d, 
		   const DGtal::Color & color2d )
{
  typedef typename ArithmeticalDSS3d::ConstIterator ConstIterator3d;
  typedef typename ArithmeticalDSS3d::ArithmeticalDSS2d ArithmeticalDSS2d;
  typedef typename ArithmeticalDSS2d::ConstIterator ConstIterator2d;
  typedef typename ArithmeticalDSS2d::Point Point2d;
  typedef typename KSpace::Cell Cell;
  typedef typename KSpace::Point Point3d;
  typedef DGtal::PointVector<2,double> PointD2d;
  typedef Display3D::pointD3D PointD3d;  
  Point3d b = ks.lowerBound();
  viewer << CustomColors3D( color3d, color3d ) << dss3d;
  for ( DGtal::Dimension i = 0; i < 3; ++i )
    {
      const ArithmeticalDSS2d & dss2d = dss3d.arithmeticalDSS2d( i );
      for ( ConstIterator2d itP = dss2d.begin(), itPEnd = dss2d.end(); itP != itPEnd; ++itP )
	{
	  Point2d p = *itP;
	  Point3d q;
	  switch (i) {
	  case 0: q = Point3d( 2*b[ i ]  , 2*p[ 0 ]+1, 2*p[ 1 ]+1 ); break;
	  case 1: q = Point3d( 2*p[ 0 ]+1, 2*b[ i ]  , 2*p[ 1 ]+1 ); break;
	  case 2: q = Point3d( 2*p[ 0 ]+1, 2*p[ 1 ]+1, 2*b[ i ]   ); break;
	  }
	  Cell c = ks.uCell( q );
	  viewer << CustomColors3D( CURVE2D_COLOR, CURVE2D_COLOR ) << c; 
	}

      // draw 2D bounding boxes for each arithmetical dss 2D.
      std::vector<PointD2d> pts2d;
      pts2d.push_back( dss2d.project(*dss2d.myF, dss2d.myUf) );
      pts2d.push_back( dss2d.project(*dss2d.myF, dss2d.myLf) );
      pts2d.push_back( dss2d.project(*dss2d.myL, dss2d.myLf) );
      pts2d.push_back( dss2d.project(*dss2d.myL, dss2d.myUf) );
      std::vector<PointD3d> bb;
      PointD3d p3;
      for ( unsigned int j = 0; j < pts2d.size(); ++j )
	{
	  switch (i) {
	  case 0: p3.x = (double) b[ i ]-0.5; p3.y = pts2d[ j ][ 0 ];     p3.z = pts2d[ j ][ 1 ];     break;
	  case 1: p3.x = pts2d[ j ][ 0 ];     p3.y = (double) b[ i ]-0.5; p3.z = pts2d[ j ][ 1 ];     break;
	  case 2: p3.x = pts2d[ j ][ 0 ];     p3.y = pts2d[ j ][ 1 ];     p3.z = (double) b[ i ]-0.5; break;
	  }
	  bb.push_back( p3 );
	}
      for ( unsigned int j = 0; j < pts2d.size(); ++j )
        viewer.addLine( bb[ j ].x, bb[ j ].y, bb[ j ].z,
                        bb[ (j+1)%4 ].x, bb[ (j+1)%4 ].y, bb[ (j+1)%4 ].z,
                        color2d, MS3D_LINESIZE );
    } // for ( DGtal::Dimension i = 0; i < 3; ++i )
}

/**
 * segmentation test
 *
 */
template <typename KSpace, typename PointIterator>
bool displayCover( Viewer3D & viewer, 
		   const KSpace & ks, PointIterator b, PointIterator e )
{
  typedef typename PointIterator::value_type Point;
  typedef ArithmeticalDSS3d<PointIterator,int,4> SegmentComputer;  
  typedef SaturatedSegmentation<SegmentComputer> Decomposition;
  typedef typename Decomposition::SegmentComputerIterator SegmentComputerIterator;
  typedef typename SegmentComputer::ArithmeticalDSS2d ArithmeticalDSS2d;
  trace.beginBlock( "Tangential cover 3D" );
  SegmentComputer algo;
  Decomposition theDecomposition(b, e, algo);

  viewer << SetMode3D( algo.className(), "BoundingBox" );
  HueShadeColorMap<int> cmap_hue( 0, MS3D_NBCOLORS, 1 );
           
  unsigned int c = 0;
  for ( SegmentComputerIterator i = theDecomposition.begin();
        i != theDecomposition.end(); ++i)
    {
      SegmentComputer ms3d(*i);
      const ArithmeticalDSS2d & dssXY = ms3d.arithmeticalDSS2dXY();
      const ArithmeticalDSS2d & dssXZ = ms3d.arithmeticalDSS2dXZ();
      const ArithmeticalDSS2d & dssYZ = ms3d.arithmeticalDSS2dYZ();
      Point f = *ms3d.begin();
      Point l = *(ms3d.end() - 1);
      trace.info() << "- " << c 
                   << " MS3D," 
                   << " [" << f[ 0 ] << "," << f[ 1 ] << ","<< f[ 2 ] << "]"
                   << "->[" << l[ 0 ] << "," << l[ 1 ] << ","<< l[ 2 ] << "]"
                   << ", XY(" 
                   << dssXY.getA() << "," << dssXY.getB() << "," << dssXY.getMu()
                   << "), XZ(" 
                   << dssXZ.getA() << "," << dssXZ.getB() << "," << dssXZ.getMu()
                   << "), YZ(" 
                   << dssYZ.getA() << "," << dssYZ.getB() << "," << dssYZ.getMu()
                   << ")" << std::endl;
      //trace.info() << ms3d << std::endl;  // information
      Color color = cmap_hue( c );
      displayDSS3d( viewer, ks, ms3d, color, color );
      c++;
    } 
  
  trace.endBlock();
  return true;
}

template <typename Point>
void makeExampleCurve3D( std::vector<Point> & sequence )
{
  sequence.push_back(Point(0,0,0));
  sequence.push_back(Point(1,0,0));
  sequence.push_back(Point(2,0,0));
  sequence.push_back(Point(2,1,0));
  sequence.push_back(Point(2,1,1));
  sequence.push_back(Point(3,1,1));
  sequence.push_back(Point(4,1,1));
  sequence.push_back(Point(4,2,1));
  sequence.push_back(Point(4,2,2));
  sequence.push_back(Point(5,2,2));
  sequence.push_back(Point(6,2,2));
  sequence.push_back(Point(6,3,2));
  sequence.push_back(Point(6,3,3));
  sequence.push_back(Point(6,4,3));
  sequence.push_back(Point(6,4,4));
  sequence.push_back(Point(6,5,4));
  sequence.push_back(Point(7,5,4));
  sequence.push_back(Point(7,6,4));
  sequence.push_back(Point(7,7,4));
  sequence.push_back(Point(7,7,5));
  sequence.push_back(Point(7,8,5));
  sequence.push_back(Point(7,8,6));
  sequence.push_back(Point(8,8,6));
  sequence.push_back(Point(9,8,6));
  sequence.push_back(Point(10,8,6));
  sequence.push_back(Point(11,8,6));
  sequence.push_back(Point(12,8,6));
  sequence.push_back(Point(12,8,5));
  sequence.push_back(Point(12,7,5));
  sequence.push_back(Point(12,6,5));
  sequence.push_back(Point(12,6,4));
}

int main(int argc, char **argv)
{
  typedef SpaceND<3,int> Z3;
  typedef KhalimskySpaceND<3,int> K3;
  typedef Z3::Point Point;
  typedef Z3::RealPoint RealPoint;
  QApplication application(argc,argv);
  Viewer3D viewer;
  trace.beginBlock ( "Example exampleTangentialCover3d" );

  // Create curve 3D.
  vector<Point> sequence;
  if ( argc <= 1 )
    // from scratch
    makeExampleCurve3D( sequence );
  else
    { // From a file, e.g. "DGtal/examples/samples/sinus.dat".
      fstream inputStream;
      inputStream.open ( argv[ 1 ], ios::in);
      try {
        sequence = PointListReader<Point>::getPointsFromInputStream( inputStream );
        if ( sequence.size() == 0) throw IOException(); 
      }
      catch (DGtal::IOException & ioe) {
        trace.error() << "Size is null." << std::endl;
      }
      inputStream.close();
    }

  // Create domain
  Point lowerBound = sequence[ 0 ];
  Point upperBound = sequence[ 0 ];
  for ( unsigned int j = 1; j < sequence.size(); ++j )
    {
      lowerBound = lowerBound.inf( sequence[ j ] );
      upperBound = upperBound.sup( sequence[ j ] );
    }
  lowerBound -= Point::diagonal( 1 );
  upperBound += Point::diagonal( 2 );
  K3 ks; ks.init( lowerBound, upperBound, true ); 
  GridCurve<K3> gc( ks ); 
  try {
    gc.initFromPointsVector( sequence );
  } catch (DGtal::ConnectivityException& /*ce*/) {
    throw ConnectivityException();
    return false;
  }

  // Displays everything.
  viewer.show();
  // Display axes.
  displayAxes<Point,RealPoint>( viewer, lowerBound, upperBound );
  // Display 3D tangential cover.
  bool res = displayCover( viewer, ks, sequence.begin(), sequence.end() );
  // Display 3D curve points.
  viewer << CustomColors3D( CURVE3D_COLOR, CURVE3D_COLOR )
         << gc.getPointsRange()
         << sequence.back(); // curiously, last point is not displayed.

  // User "interaction".
  viewer << Viewer3D::updateDisplay;
  application.exec();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();

  return res ? 0 : 1;
}
