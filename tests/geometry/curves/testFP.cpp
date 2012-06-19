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
 * @file testFP.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/01/26
 *
 * Functions for testing class FP.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/geometry/curves/FreemanChain.h"
#include "DGtal/geometry/curves/FP.h"
#include "DGtal/io/boards/Board2D.h"

#include "ConfigTest.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace LibBoard;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class FP.
///////////////////////////////////////////////////////////////////////////////

template<typename Coordinate>
void 
drawVectorOfPointsAsPolygon( const vector<PointVector<2,Coordinate> >& v, Board2D & aBoard) 
{
  //polyline to draw
  vector<LibBoard::Point> polyline;

  typename vector<PointVector<2,Coordinate> >::const_iterator i = v.begin();
  for ( ;i != v.end();++i) {
      PointVector<2,Coordinate> p = (*i);
      double xp = (double) p[0];
      double yp = (double) p[1];
      polyline.push_back(LibBoard::Point(xp,yp));
  }

  aBoard.drawPolyline(polyline);

}

/**
 * Example of a test. To be completed.
 *
 */
bool testDrawingFP()
{

  typedef int Coordinate;
  typedef HyperRectDomain<SpaceND<2,Coordinate> > Domain;
  typedef PointVector<2,Coordinate> Point;
  typedef PointVector<2,double> RealPoint;
  typedef FreemanChain<Coordinate> Contour; 
  typedef FP<Contour::ConstIterator,Coordinate,4> FP;

  std::string filename = testPath + "samples/france.fc";
  std::cout << filename << std::endl;

  std::fstream fst;
  fst.open (filename.c_str(), std::ios::in);
  Contour theContour(fst);

  trace.beginBlock ( "FP of a 4-connected digital curve..." );

  FP theFP( theContour.begin(),theContour.end(),true );
  //trace.info() << theFP << std::endl;

  // Draw the FP
  Board2D aBoard;
  aBoard << SetMode( "PointVector", "Grid" ) << theContour;
  aBoard << theFP;
  aBoard.saveEPS("FP.eps");

  //accessors: 
  Board2D newBoard;
  newBoard << SetMode( "PointVector", "Grid" ) << theContour;

  trace.info() << "FP" << endl;
  vector<Point> v( theFP.size() );
  theFP.copyFP( v.begin() );
//  copy( v.begin(),v.end(),ostream_iterator<Point>(cout, "\n") );
  drawVectorOfPointsAsPolygon<int>(v, newBoard); 

  trace.info() << "MLP" << endl;
  vector<RealPoint> v2( theFP.size() );
  theFP.copyMLP( v2.begin() );
//  copy( v2.begin(),v2.end(),ostream_iterator<RealPoint>(cout, "\n") );
  drawVectorOfPointsAsPolygon<double>(v2, newBoard); 

  newBoard.saveEPS("FP_MLP.eps");

  trace.endBlock();
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class FP" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testDrawingFP(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
