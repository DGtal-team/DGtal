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
#include <exception>
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
 * IO Tests
 *
 */
template <typename KSpace>
bool testIOGridCurve(const string& filename)
{

  unsigned int d = KSpace::Point::dimension;
  GridCurve<KSpace> c; //grid curve

//////////////////////////////////////////
  trace.info() << endl;
  trace.info() << "Reading GridCurve d=" << d << endl;
  
  ifstream instream; // input stream
  instream.open (filename.c_str(), ifstream::in);

  c.initFromVectorStream(instream);

  cout << c << endl;

///////////////////////////////////////////
  std::stringstream s; 
  s << "gridcurve" << d << ".dat"; 

  trace.info() << "Writing GridCurve d=" << d << " in " << s.str() << endl;

  ofstream outstream(s.str()); //output stream
  if (!outstream.is_open()) return false;
  else {
    c.writeVectorToStream(outstream);
  }
  outstream.close();

  return true;
}


/**
 * Open/Closed
 *
 */
bool testIsOpen(const string &filename, const bool& aFlag)
{

  trace.info() << endl;
  trace.info() << "Open/Closed test" << endl;

  GridCurve<KhalimskySpaceND<2> > c; //grid curve

  ifstream instream; // input stream
  instream.open (filename.c_str(), ifstream::in);
  c.initFromVectorStream(instream);

  return (c.isOpen() == aFlag);
}

/**
 * Exceptions
 *
 */
bool testExceptions(const string &filename)
{

  GridCurve<KhalimskySpaceND<2> > c; //grid curve

  trace.info() << endl;
  trace.info() << "Trying to read bad file: " << filename << endl;
  
  ifstream instream; // input stream
  instream.open (filename.c_str(), ifstream::in);

  try {
    c.initFromVectorStream(instream);
    trace.info() << "no exception catched!?" << endl;
    return false;
  }  catch (DGtal::ConnectivityException& e) {
    trace.info() << e.what() << endl;
    return true;
  } catch (DGtal::InputException& e) {
    trace.info() << e.what() << endl;
    return true;
  } catch (exception& e) {
    trace.info() << e.what() << endl;
    return true;
  } 
}

/**
 * Display
 *
 */
bool testDisplay(const string &filename)
{

  GridCurve<KhalimskySpaceND<2> > c; //grid curve

  trace.info() << endl;
  trace.info() << "Displaying GridCurve " << endl;
  
  //reading grid curve
  fstream inputStream;
  inputStream.open (filename.c_str(), ios::in);
  c.initFromVectorStream(inputStream); 
  inputStream.close();

  //displaying it
  DGtalBoard aBoard;
  aBoard.setUnit(Board::UCentimeter);
  aBoard << SetMode(c.styleName(), "Edges") << c;
  
  aBoard.saveEPS( "GridCurveEdges.eps", Board::BoundingBox, 5000 );

  aBoard << SetMode(c.styleName(), "Points") << c;

  aBoard.saveEPS( "GridCurveBoth.eps", Board::BoundingBox, 5000 );

#ifdef WITH_CAIRO
  aBoard.saveCairo("GridCurveBoth-cairo.pdf", DGtalBoard::CairoPDF, Board::BoundingBox, 5000);
#endif
  

  return true;
}


/**
 * PointsRange
 *
 */
template <typename Range>
bool testRange(const Range &aRange)
{

  trace.info() << endl;
  trace.info() << "Testing Range (" << aRange.size() << " elts)" << endl;
  
{
  trace.info() << "Forward" << endl;
  typename Range::ConstIterator i = aRange.begin();
  typename Range::ConstIterator end = aRange.end();
  for ( ; i != end; ++i) {
    cout << *i << endl;
  }
}
{
  trace.info() << "Backward" << endl;
  typename Range::ConstReverseIterator i = aRange.rbegin();
  typename Range::ConstReverseIterator end = aRange.rend();
  for ( ; i != end; ++i) {
    cout << *i << endl;
  }
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
  std::string polyg2D = testPath + "samples/polyg2D.dat";
  std::string sinus3D = testPath + "samples/sinus3D.dat";
  std::string emptyFile = testPath + "samples/emptyFile.dat";
  std::string square = testPath + "samples/smallSquare.dat";

  typedef KhalimskySpaceND<2> K2;
  typedef KhalimskySpaceND<3> K3;

///////// general tests
  bool res = testIOGridCurve<K2>(sinus2D4)
    && testIOGridCurve<K3>(sinus3D)
    && testExceptions(sinus3D)
    && testExceptions(polyg2D)
    && testExceptions(emptyFile)
    && testDisplay(sinus2D4)
    && testIsOpen(sinus2D4,true)
    && testIsOpen(square,false); 

/////////// ranges test
  typedef GridCurve<K2> GridCurve;

  //reading grid curve
  GridCurve c; 
  fstream inputStream;
  inputStream.open (square.c_str(), ios::in);
  c.initFromVectorStream(inputStream);
  inputStream.close();

  res = res 
    && testRange<GridCurve::sCellsRange>(c.get0CellsRange())
    && testRange<GridCurve::sCellsRange>(c.get1CellsRange())
    && testRange<GridCurve::PointsRange>(c.getPointsRange())
    && testRange<GridCurve::MidPointsRange>(c.getMidPointsRange())
;

//////////////////////

  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
