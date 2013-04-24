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
/**
 * segmentation test
 *
 */
template <typename PointIterator>
bool displayCover( Viewer3D & viewer, PointIterator b, PointIterator e )
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
           
  unsigned int c = 0;
  SegmentComputerIterator i = theDecomposition.begin();
  viewer << SetMode3D( algo.className(), "BoundingBox" );
  for ( ; i != theDecomposition.end(); ++i) {
    SegmentComputer currentSegmentComputer(*i);
    trace.info() << currentSegmentComputer << std::endl;  //standard output
    c++;
    viewer << currentSegmentComputer;
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
  Point lowerBound = Point::diagonal( 0 );
  Point upperBound = Point::diagonal( 15 ); 

  //! [GridCurveDeclaration]
  K3 ks; ks.init( lowerBound, upperBound, true ); 
  GridCurve<K3> gc( ks ); 
  gc.initFromPointsVector( sequence );

  viewer.show();
  bool res = displayCover( viewer, sequence.begin(), sequence.end() );
  Point p;
  viewer << gc.getPointsRange();
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
