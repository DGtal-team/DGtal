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
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 *
 * @date 2011/06/01
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
#include "DGtal/io/DrawWithDisplay3DModifier.h"


using namespace DGtal;
using namespace std;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ArithmeticalDSS3d.
///////////////////////////////////////////////////////////////////////////////

template <typename KSpace, typename ArithmeticalDSS3d>
void displayDSS3d( DGtal::Viewer3D & viewer, 
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
	  viewer << CustomColors3D( Color( 200, 200, 200, 100 ), Color( 200, 200, 200, 100 ) ) << c; 
	}

      //draw bounding box
      std::vector<PointD2d> pts2d;
      pts2d.push_back( dss2d.project(*dss2d.myF, dss2d.myUf) );
      pts2d.push_back( dss2d.project(*dss2d.myF, dss2d.myLf) );
      pts2d.push_back( dss2d.project(*dss2d.myL, dss2d.myLf) );
      pts2d.push_back( dss2d.project(*dss2d.myL, dss2d.myUf) );
      std::vector<PointD3d> bb;
      PointD3d p3;
      p3.R = color2d.red();
      p3.G = color2d.green();
      p3.B = color2d.blue();
      p3.T = color2d.alpha();
      p3.isSigned = false;
      p3.signPos = false;
      p3.size = 3;
      for ( unsigned int j = 0; j < pts2d.size(); ++j )
	{
	  switch (i) {
	  case 0: p3.x = (double) b[ i ]-0.5; p3.y = pts2d[ j ][ 0 ];     p3.z = pts2d[ j ][ 1 ];     break;
	  case 1: p3.x = pts2d[ j ][ 0 ];     p3.y = (double) b[ i ]-0.5; p3.z = pts2d[ j ][ 1 ];     break;
	  case 2: p3.x = pts2d[ j ][ 0 ];     p3.y = pts2d[ j ][ 1 ];     p3.z = (double) b[ i ]-0.5; break;
	  }
	  bb.push_back( p3 );
	}
      viewer << CustomColors3D( color2d, color2d );
      for ( unsigned int j = 0; j < pts2d.size(); ++j )
        viewer.addLine( bb[ j ].x, bb[ j ].y, bb[ j ].z,
                        bb[ (j+1)%4 ].x, bb[ (j+1)%4 ].y, bb[ (j+1)%4 ].z,
                        color2d, 2.0 );
      // viewer.addPolygon( bb, Color(255,255,255,0) );
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
  // typedef PointVector<3,int> Point;
  // typedef std::vector<Point>::iterator Iterator;
  typedef ArithmeticalDSS3d<PointIterator,int,4> SegmentComputer;  
  typedef SaturatedSegmentation<SegmentComputer> Decomposition;
  typedef typename Decomposition::SegmentComputerIterator SegmentComputerIterator;
  //Segmentation
  trace.beginBlock("Segmentation test");
  SegmentComputer algo;
  Decomposition theDecomposition(b, e, algo);

  viewer << SetMode3D( algo.className(), "BoundingBox" );
  HueShadeColorMap<int> cmap_hue( 0, 8, 1 );
           
  unsigned int c = 0;
  for ( SegmentComputerIterator i = theDecomposition.begin();
        i != theDecomposition.end(); ++i) {
    SegmentComputer currentSegmentComputer(*i);
    trace.info() << currentSegmentComputer << std::endl;  //standard output
    //Color color( random() % 256, random() % 256, random() % 256, 255 );
    Color color = cmap_hue( c );
    displayDSS3d( viewer, ks, currentSegmentComputer, color, color );
    // viewer << currentSegmentComputer;
    c++;
  } 
  
  trace.endBlock();
  return true;
}


int main(int argc, char **argv)
{
  typedef PointVector<3,int> Point;
  typedef KhalimskySpaceND<3, int> K3;
  QApplication application(argc,argv);
  Viewer3D viewer;
  trace.beginBlock ( "Example exampleTangentialCover3d" );
  vector<Point> sequence;
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

  // domain
  Point lowerBound = Point::diagonal( -1 );
  Point upperBound = Point::diagonal( 14 ); 

  //! [GridCurveDeclaration]
  K3 ks; ks.init( lowerBound, upperBound, true ); 
  GridCurve<K3> gc( ks ); 
  gc.initFromPointsVector( sequence );

  viewer.show();
  bool res = displayCover( viewer, ks, sequence.begin(), sequence.end() );
  Point p;
  viewer << CustomColors3D( Color( 100, 100, 140, 128 ), Color( 100, 100, 140, 128 ) );
  viewer << gc.getPointsRange();
  viewer << sequence.back();
  // viewer << SetMode3D( p.className(), "Paving" );
  // for ( vector<Point>::const_iterator it = sequence.begin(), itE = sequence.end();
  // 	it != itE; ++it )
  //   viewer << *it;
	 
  viewer << Viewer3D::updateDisplay;
  application.exec();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();

  return res ? 0 : 1;

}
