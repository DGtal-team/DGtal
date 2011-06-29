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
 * @file testGridCurve.cpp
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 *
 * @date 2011/06/27

 * Functions for testing class GridCurve
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/program_options.hpp>
#include "DGtal/base/Common.h"

#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/DigitalSetSelector.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/SurfelAdjacency.h"
#include "DGtal/topology/SurfelNeighborhood.h"


#include "DGtal/geometry/2d/GridCurve.h"

#include "DGtal/io/DGtalBoard.h"

#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace LibBoard;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class FreemanChain.
///////////////////////////////////////////////////////////////////////////////

/**
 * I Test
 *
 */
template <typename KSpace>
bool testReadGridCurve(const string& filename)
{
  trace.info() << "Reading GridCurve " << endl;
  
  ifstream instream; // input stream
  instream.open (filename.c_str(), ifstream::in);

  GridCurve<KSpace> c(instream); //grid curve

  vector<typename KSpace::Space::Point> aVectorOfPoints; 
  c.getData(c, aVectorOfPoints);


  return (aVectorOfPoints == c.myData);
}

/**
 * O Test
 *
 */
template <typename KSpace>
bool testWriteGridCurve(const string& filename)
{
  trace.info() << "Writing GridCurve " << endl;
  
  ifstream instream; // input stream
  instream.open (filename.c_str(), ifstream::in);

  GridCurve<KSpace> c(instream); //grid curve

  ofstream outstream("gridcurve.dat"); //output stream
  if (!outstream.is_open()) return false;
  else {
    GridCurve<KSpace>::write(outstream,c);
  }
  outstream.close();

  return true;
}





/**
 * Display
 *
 */
bool testDisplay(const string &filename)
{

  trace.info() << "Displaying GridCurve " << endl;
  
  //reading grid curve
  fstream inputStream;
  inputStream.open (filename.c_str(), ios::in);
  GridCurve<KhalimskySpaceND<2> > c(inputStream); 
  inputStream.close();

  //displaying it
  DGtalBoard aBoard;
  aBoard.setUnit(Board::UCentimeter);
  aBoard << SetMode(c.styleName(), "Edges") << c;
  
  aBoard.saveEPS( "GridCurve.eps", Board::BoundingBox, 5000 );

#ifdef WITH_CAIRO
  aBoard.saveCairo("GridCurve-cairo.pdf", DGtalBoard::CairoPDF, Board::BoundingBox, 5000);
#endif
  

  return true;
}


/**
 * PointsRange
 *
 */
bool testPointsRange(const string &filename)
{

  trace.info() << "Testing PointsRange " << endl;
  
  typedef GridCurve<KhalimskySpaceND<2> > GridCurve;

  //reading grid curve
  fstream inputStream;
  inputStream.open (filename.c_str(), ios::in);
  GridCurve c(inputStream); 
  inputStream.close();

  //points range
  GridCurve::PointsRange aRange = c.getPointsRange();
  GridCurve::PointsRange::ConstIterator i = aRange.begin();
  GridCurve::PointsRange::ConstIterator end = aRange.end();
  for ( ; i != end; ++i) {
    cout << "pouet" << endl;
  }
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class GridCurve" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;


  std::string sinus2D4 = testPath + "samples/sinus2D4.dat";
  std::string sinus3D = testPath + "samples/sinus2D4.dat";

  typedef KhalimskySpaceND<2> K2;
  typedef KhalimskySpaceND<3> K3;

  bool res = testReadGridCurve<K2>(sinus2D4)
    && testReadGridCurve<K3>(sinus3D)
    && testWriteGridCurve<K2>(sinus2D4)
    && testDisplay(sinus2D4)
    && testPointsRange(sinus2D4);
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
