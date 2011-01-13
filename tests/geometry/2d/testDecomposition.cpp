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
//LICENSE-END
/**
 * @file testDecomposition.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 *
 * @date 2010/07/02
 *
 * This file is part of the DGtal library
 */

/**
 * Description of testDecomposition <p>
 * Aim: simple test of \ref GreedyDecomposition
 */

#include <cstdio>
#include <cmath>
#include <fstream>
#include <vector>
#include <iostream>
#include <iterator>




#include "DGtal/base/Common.h"
#include "DGtal/base/Exceptions.h"
#include "DGtal/io/DGtalBoard.h"

#include "DGtal/geometry/2d/ArithmeticalDSS.h"
#include "DGtal/geometry/2d/FreemanChain.h"
#include "DGtal/geometry/2d/GreedyDecomposition.h"


#include "ConfigTest.h"


using namespace DGtal;
using namespace std;
using namespace LibBoard;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class GreedyDecomposition.
///////////////////////////////////////////////////////////////////////////////
/**
 * Test for the segmentation of 
 * 4-connected digital curves into DSS
 *
 */
bool testDec4()
{

  typedef int Coordinate;
  typedef PointVector<2,Coordinate> Point;
  typedef ArithmeticalDSS<Coordinate,4> PrimitiveType;
  
  typedef FreemanChain<Coordinate> ContourType; 

	typedef GreedyDecomposition<ContourType::ConstIterator,PrimitiveType> DecompositionType;

  std::string filename = testPath + "samples/manche.fc";
  std::cout << filename << std::endl;

  std::fstream fst;
  fst.open (filename.c_str(), std::ios::in);
  ContourType theContour(fst);

  //Segmentation
  trace.beginBlock("Segmentation of a chain code into DSS");
  DecompositionType theDecomposition(theContour.begin(), theContour.end());
  
	// Draw the grid
  DGtalBoard aBoard;
  aBoard.setUnit(Board::UCentimeter);
  
  aBoard << SetMode("PointVector", "Grid")
				 << theContour;
  
  //for each segment
	unsigned int compteur = 0;
  DecompositionType::ConstIterator i = theDecomposition.begin();
  for ( ; i != theDecomposition.end(); ++i) {
		
		compteur++;
    PrimitiveType segment(*i); 
    trace.info() << segment << std::endl;	//standard output
    aBoard << SetMode( "ArithmeticalDSS", "BoundingBox" )
					 << segment; // draw each segment  
  
  } 

  aBoard.saveSVG("segmentationDSS4.svg");

  trace.endBlock();
	return (compteur==91);
}

/**
 * Test for the segmentation of 
 * 8-connected digital curves into DSS
 *
 */
bool testDec8()
{

  typedef int Coordinate;
  typedef PointVector<2,Coordinate> Point;
  typedef ArithmeticalDSS<Coordinate,8> PrimitiveType;
  
  typedef std::vector<Point> ContourType; 

	typedef GreedyDecomposition<ContourType::iterator,PrimitiveType> DecompositionType;

	std::vector<Point> curve;
	curve.push_back(Point(0,0));
	curve.push_back(Point(1,1));
	curve.push_back(Point(2,1));
	curve.push_back(Point(3,2));
	curve.push_back(Point(4,2));
	curve.push_back(Point(5,2));
	curve.push_back(Point(6,3));
	curve.push_back(Point(6,4));
	curve.push_back(Point(7,4));
	curve.push_back(Point(8,4));
	curve.push_back(Point(9,3));
	curve.push_back(Point(10,2));
	curve.push_back(Point(11,2));

  //Segmentation
  trace.beginBlock("Segmentation of a 8-connected digital curve into DSS");
  DecompositionType theDecomposition(curve.begin(), curve.end());
  
	// Draw the pixels
  DGtalBoard aBoard;
  aBoard.setUnit(Board::UCentimeter);
  aBoard << SetMode("PointVector", "Both");
	for (ContourType::iterator it = curve.begin(); it != curve.end(); ++it) {
  	aBoard << (*it);
	}
				 

  //for each segment
	unsigned int compteur = 0;
  DecompositionType::ConstIterator i = theDecomposition.begin();
  for ( ; i != theDecomposition.end(); ++i) {

		compteur++;
    trace.info() << "Segment " << compteur << std::endl;
    PrimitiveType segment(*i); 		
		trace.info() << segment << std::endl;	//standard output
    aBoard << SetMode( "ArithmeticalDSS", "BoundingBox" )
					 << segment; // draw each segment    
  } 

  aBoard.saveSVG("segmentationDSS8.svg");

  trace.endBlock();

	return (compteur==4);
}

/**
 * Test for the segmentation of 
 * disconnected digital curves into DSS
 *
 */
bool testDisconnectedCurve()
{
  typedef int Coordinate;
  typedef PointVector<2,Coordinate> Point;
  typedef ArithmeticalDSS<Coordinate,4> PrimitiveType;
  
  typedef std::vector<Point> ContourType; 

	typedef GreedyDecomposition<ContourType::iterator,PrimitiveType> DecompositionType;

	std::vector<Point> curve;
	curve.push_back(Point(0,0));
	curve.push_back(Point(1,0));
	curve.push_back(Point(1,1));
	curve.push_back(Point(2,1));
	curve.push_back(Point(3,2));
	curve.push_back(Point(4,2));
	curve.push_back(Point(5,2));
	curve.push_back(Point(6,2));
	curve.push_back(Point(6,3));
	curve.push_back(Point(6,4));
	curve.push_back(Point(7,4));
	curve.push_back(Point(8,4));
	curve.push_back(Point(9,3));
	curve.push_back(Point(9,2));
	curve.push_back(Point(10,2));
	curve.push_back(Point(11,2));

  //Segmentation
  trace.beginBlock("Segmentation of a 8-connected digital curve into 4-connected DSS");
  DecompositionType theDecomposition(curve.begin(), curve.end());
  
	// Draw the pixels
  DGtalBoard aBoard;
  aBoard.setUnit(Board::UCentimeter);
  aBoard << SetMode("PointVector", "Both");
	for (ContourType::iterator it = curve.begin(); it != curve.end(); ++it) {
  	aBoard << (*it);
	}
				 

  //for each segment
	unsigned int compteur = 0;
  DecompositionType::ConstIterator i = theDecomposition.begin();
  for ( ; i != theDecomposition.end(); ++i) {

		compteur++;
    trace.info() << "Segment " << compteur << std::endl;
    PrimitiveType segment(*i); 		
		trace.info() << segment << std::endl;	//standard output
    aBoard << SetMode( "ArithmeticalDSS", "BoundingBox" )
					 << segment; // draw each segment    
  } 

  aBoard.saveSVG("specialCase.svg");

  trace.endBlock();

	return (compteur==4);

}

/**
 * Test for closed curves processed as closed
 *
 */
bool testClosedCurvesProcessedAsClosed()
{

  trace.beginBlock ( "Test for closed curves processed as closed" );

  typedef ArithmeticalDSS<int,4> DSS4;
  typedef FreemanChain<int> Contour4; 
  typedef GreedyDecomposition< Contour4::ConstIterator, DSS4 > Decomposition4;

  // A Freeman chain code is a string composed by the coordinates of the first pixel, and the list of elementary displacements. 
  std::stringstream ss(stringstream::in | stringstream::out);
  ss << "31 16 11121212121212212121212212122122222322323233323333333323333323303330330030300000100010010010001000101010101111" << endl;
  
  // Construct the Freeman chain
  Contour4 theContour( ss );

  //Segmentation
  Decomposition4 theDecomposition( theContour.begin(),theContour.end() );

  DGtalBoard aBoard;
  aBoard << SetMode( "PointVector", "Grid" )
	 			 << theContour;
  //for each segment
  aBoard << SetMode( "ArithmeticalDSS", "BoundingBox" );
  string styleName = "ArithmeticalDSS/BoundingBox";
  for ( Decomposition4::ConstIterator i = theDecomposition.begin();
	i != theDecomposition.end(); ++i ) 
    {

			DSS4 segment(*i);
			aBoard << CustomStyle( styleName, 
												     new CustomPenColor( DGtalBoard::Color::Blue ) )
						 << segment; // draw each segment

    } 
  aBoard.saveSVG("testClosedCurvesProcessedAsClosed.svg");

  trace.endBlock();

  return true;
}

/**
 * Test for closed curves processed as closed
 *
 */
bool testClosedCurvesProcessedAsOpen()
{

  trace.beginBlock ( "Test for closed curves processed as open" );

  typedef ArithmeticalDSS<int,4> DSS4;
  typedef FreemanChain<int> Contour4; 
  typedef GreedyDecomposition< Contour4::ConstIterator, DSS4 > Decomposition4;

  // A Freeman chain code is a string composed by the coordinates of the first pixel, and the list of elementary displacements. 
  std::stringstream ss(stringstream::in | stringstream::out);
  ss << "31 16 11121212121212212121212212122122222322323233323333333323333323303330330030300000100010010010001000101010101111" << endl;
  
  // Construct the Freeman chain
  Contour4 theContour( ss );

  //Segmentation
  Decomposition4 theDecomposition( theContour.begin(),theContour.end(), false );

  DGtalBoard aBoard;
  aBoard << SetMode( "PointVector", "Grid" )
	 			 << theContour;
  //for each segment
  aBoard << SetMode( "ArithmeticalDSS", "BoundingBox" );
  string styleName = "ArithmeticalDSS/BoundingBox";
  for ( Decomposition4::ConstIterator i = theDecomposition.begin();
	i != theDecomposition.end(); ++i ) 
    {

			DSS4 segment(*i);
			aBoard << CustomStyle( styleName, 
												     new CustomPenColor( DGtalBoard::Color::Blue ) )
						 << segment; // draw each segment

    } 
  aBoard.saveSVG("testClosedCurvesProcessedAsOpen.svg");

  trace.endBlock();

  return true;
}

/**
 * Test for open curves processed as closed
 *
 */
bool testOpenCurvesProcessedAsClosed()
{

  trace.beginBlock ( "Test for open curves processed as closed" );

  typedef ArithmeticalDSS<int,4> DSS4;
  typedef FreemanChain<int> Contour4; 
  typedef GreedyDecomposition< Contour4::ConstIterator, DSS4 > Decomposition4;

  // A Freeman chain code is a string composed by the coordinates of the first pixel, and the list of elementary displacements. 
  std::stringstream ss(stringstream::in | stringstream::out);
  ss << "31 16 11121212121212212121212212122122222322323233323333333323333" << endl;
  
  // Construct the Freeman chain
  Contour4 theContour( ss );

  //Segmentation
  Decomposition4 theDecomposition( theContour.begin(),theContour.end(), true );

  DGtalBoard aBoard;
  aBoard << SetMode( "PointVector", "Grid" )
	 			 << theContour;
  //for each segment
  aBoard << SetMode( "ArithmeticalDSS", "BoundingBox" );
  string styleName = "ArithmeticalDSS/BoundingBox";
  for ( Decomposition4::ConstIterator i = theDecomposition.begin();
	i != theDecomposition.end(); ++i ) 
    {

			DSS4 segment(*i);
			aBoard << CustomStyle( styleName, 
												     new CustomPenColor( DGtalBoard::Color::Blue ) )
						 << segment; // draw each segment

    } 
  aBoard.saveSVG("testOpenCurvesProcessedAsClosed.svg");

  trace.endBlock();

  return true;
}

/////////////////////////////////////////////////////////////////////////
//////////////// MAIN ///////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  
  trace.beginBlock ( "Testing class GreedyDecomposition" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testDec4()
					&& testDec8() 
					&& testDisconnectedCurve()
					&& testClosedCurvesProcessedAsClosed()
					&& testClosedCurvesProcessedAsOpen()
					&& testOpenCurvesProcessedAsClosed();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;

}
