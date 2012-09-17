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
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/Color.h"

#include "DGtal/geometry/curves/ArithmeticalDSS.h"
#include "DGtal/geometry/curves/FreemanChain.h"
#include "DGtal/geometry/curves/GreedyDecomposition.h"


#include "ConfigTest.h"


using namespace DGtal;
using namespace DGtal::deprecated;
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
  typedef FreemanChain<Coordinate> ContourType; 

  typedef ArithmeticalDSS<ContourType::ConstIterator,Coordinate,4> PrimitiveType;
  
  typedef GreedyDecomposition<PrimitiveType> DecompositionType;


  std::string filename = testPath + "samples/manche.fc";
  std::cout << filename << std::endl;

  std::fstream fst;
  fst.open (filename.c_str(), std::ios::in);
  ContourType theContour(fst);

  //Segmentation
  trace.beginBlock("Segmentation of a chain code into DSS");
  PrimitiveType computer;
  DecompositionType theDecomposition(theContour.begin(), theContour.end(), computer, false);
  
  // Draw the grid
  Board2D aBoard;
  aBoard.setUnit(Board::UCentimeter);
  
  aBoard << SetMode("PointVector", "Grid")
         << theContour;
  
  //for each segment
  unsigned int compteur = 0;
  DecompositionType::SegmentIterator i = theDecomposition.begin();
  for ( ; i != theDecomposition.end(); ++i) {
    
    compteur++;
    PrimitiveType segment(*i); 
    trace.info() << segment << std::endl;  //standard output
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
  typedef std::vector<Point> ContourType;
  typedef ArithmeticalDSS<ContourType::iterator,Coordinate,8> PrimitiveType;
  
  typedef GreedyDecomposition<PrimitiveType> DecompositionType;

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
  PrimitiveType computer;
  DecompositionType theDecomposition(curve.begin(), curve.end(), computer, false);
  
  // Draw the pixels
  Board2D aBoard;
  aBoard.setUnit(Board::UCentimeter);
  aBoard << SetMode("PointVector", "Both");
  for (ContourType::iterator it = curve.begin(); it != curve.end(); ++it) {
    aBoard << (*it);
  }
         

  //for each segment
  unsigned int compteur = 0;
  DecompositionType::SegmentIterator i = theDecomposition.begin();
  for ( ; i != theDecomposition.end(); ++i) {

    compteur++;
    trace.info() << "Segment " << compteur << std::endl;
    PrimitiveType segment(*i);     
    trace.info() << segment << std::endl;  //standard output
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
  typedef std::vector<Point> ContourType;
  typedef ArithmeticalDSS<ContourType::iterator,Coordinate,4> PrimitiveType;
  
  typedef GreedyDecomposition<PrimitiveType> DecompositionType;

  std::vector<Point> curve;  
  curve.push_back(Point(-1,-1));
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
  trace.beginBlock("Segmentation of disconnected digital curve");
  PrimitiveType computer;
  DecompositionType theDecomposition(curve.begin(), curve.end(), computer, false);
  
  // Draw the pixels
  Board2D aBoard;
  aBoard.setUnit(Board::UCentimeter);
  aBoard << SetMode("PointVector", "Both");
  for (ContourType::iterator it = curve.begin(); it != curve.end(); ++it) {
    aBoard << (*it);
  }
         

  //for each segment
  unsigned int compteur = 0;
  DecompositionType::SegmentIterator i = theDecomposition.begin();
  for ( ; i != theDecomposition.end(); ++i) {

    compteur++;
    trace.info() << "Segment " << compteur << std::endl;
    trace.info() << i.intersectPrevious() << " - " << i.intersectNext() << std::endl;
    PrimitiveType segment(*i);     
    trace.info() << segment << std::endl;  //standard output
    aBoard << SetMode( segment.className(), "BoundingBox" )
           << segment; // draw each segment    
  } 

  aBoard.saveSVG("specialCase.svg");

  trace.endBlock();

  return (compteur==5);

}

/**
 * Test for closed curves processed as closed
 *
 */
bool testClosedCurvesProcessedAsClosed()
{

  trace.beginBlock ( "Test for closed curves processed as closed" );


  typedef FreemanChain<int> Contour4; 
  typedef ArithmeticalDSS<Contour4::ConstIterator,int,4> DSS4;
  typedef GreedyDecomposition<DSS4> Decomposition4;

  // A Freeman chain code is a string composed by the coordinates of the first pixel, and the list of elementary displacements. 
  std::stringstream ss(stringstream::in | stringstream::out);
  ss << "31 16 1112121212121221212121221212212222232232323332333333332333332330333033003030000010001001001000100010101010111" << endl;
  
  // Construct the Freeman chain
  Contour4 theContour( ss );

  //Segmentation
  DSS4 computer;
  Decomposition4 theDecomposition( theContour.begin(),theContour.end(),computer,true );

  Board2D aBoard;
  aBoard << SetMode( "PointVector", "Grid" )
          << theContour;
  //for each segment
  aBoard << SetMode( "ArithmeticalDSS", "BoundingBox" );
  string className = "ArithmeticalDSS/BoundingBox";
  for ( Decomposition4::SegmentIterator i = theDecomposition.begin();
  i != theDecomposition.end(); ++i ) 
    {

      DSS4 segment(*i);
      aBoard << CustomStyle( className, 
                             new CustomPenColor( Color::Blue ) )
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

  typedef FreemanChain<int> Contour4; 
  typedef ArithmeticalDSS<Contour4::ConstIterator,int,4> DSS4;
  typedef GreedyDecomposition<DSS4> Decomposition4;

  // A Freeman chain code is a string composed by the coordinates of the first pixel, and the list of elementary displacements. 
  std::stringstream ss(stringstream::in | stringstream::out);
  ss << "31 16 1112121212121221212121221212212222232232323332333333332333332330333033003030000010001001001000100010101010111" << endl;
  
  // Construct the Freeman chain
  Contour4 theContour( ss );

  //Segmentation
  DSS4 computer;
  Decomposition4 theDecomposition( theContour.begin(),theContour.end(),computer,false );

  Board2D aBoard;
  aBoard << SetMode( "PointVector", "Grid" )
          << theContour;
  //for each segment
  aBoard << SetMode( "ArithmeticalDSS", "BoundingBox" );
  string className = "ArithmeticalDSS/BoundingBox";
  for ( Decomposition4::SegmentIterator i = theDecomposition.begin();
  i != theDecomposition.end(); ++i ) 
    {

      DSS4 segment(*i);
      aBoard << CustomStyle( className, 
                             new CustomPenColor( Color::Blue ) )
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

  typedef FreemanChain<int> Contour4; 
  typedef ArithmeticalDSS<Contour4::ConstIterator,int,4> DSS4;
  typedef GreedyDecomposition<DSS4> Decomposition4;

  // A Freeman chain code is a string composed by the coordinates of the first pixel, and the list of elementary displacements. 
  std::stringstream ss(stringstream::in | stringstream::out);
  ss << "31 16 1112121212121221212121221212212222232232323332333333332333" << endl;
  
  // Construct the Freeman chain
  Contour4 theContour( ss );

  //Segmentation
  DSS4 computer;
  Decomposition4 theDecomposition( theContour.begin(),theContour.end(),computer,true );

  Board2D aBoard;
  aBoard << SetMode( "PointVector", "Grid" )
          << theContour;
  //for each segment
  aBoard << SetMode( "ArithmeticalDSS", "BoundingBox" );
  string className = "ArithmeticalDSS/BoundingBox";
  for ( Decomposition4::SegmentIterator i = theDecomposition.begin();
  i != theDecomposition.end(); ++i ) 
    {

      DSS4 segment(*i);
      aBoard << CustomStyle( className, 
                             new CustomPenColor( Color::Blue ) )
             << segment; // draw each segment

    } 
  aBoard.saveSVG("testOpenCurvesProcessedAsClosed.svg");

  trace.endBlock();

  return true;
}

/**
 * Test with no point
 *
 */
bool testNoPoint()
{
  typedef int Coordinate;
  typedef PointVector<2,Coordinate> Point;
  typedef ArithmeticalDSS<std::vector<Point>::iterator,Coordinate,4> PrimitiveType;
  typedef GreedyDecomposition<PrimitiveType> DecompositionType;

  std::vector<Point> curve;
  try {

    trace.beginBlock("Digital curve having no point");
    PrimitiveType computer;
    DecompositionType theDecomposition(curve.begin(), curve.end(), computer, false);

    for ( DecompositionType::SegmentIterator i = theDecomposition.begin();
                                           i != theDecomposition.end(); ++i ) 
      {        } 
    trace.endBlock();

    return true;
  } catch (std::exception e) {
    return false;
  }


}

/**
 * Test with one point
 *
 */
bool testOnePoint()
{
  typedef int Coordinate;
  typedef PointVector<2,Coordinate> Point;
  typedef ArithmeticalDSS<std::vector<Point>::iterator,Coordinate,4> PrimitiveType;
  
  typedef GreedyDecomposition<PrimitiveType> DecompositionType;

  std::vector<Point> curve;
  curve.push_back(Point(5,5));
  try {

    trace.beginBlock("Digital curve having one point");
    PrimitiveType computer;
    DecompositionType theDecomposition(curve.begin(), curve.end(), computer, false);

    Board2D aBoard;
    aBoard << curve.at(0);
    //for each segment
    aBoard << SetMode( "ArithmeticalDSS", "BoundingBox" );
    for ( DecompositionType::SegmentIterator i = theDecomposition.begin();
                                        i != theDecomposition.end(); ++i ) 
      {
        PrimitiveType primitive(*i);
        trace.info() << primitive << endl;
        aBoard << primitive; 
        
      } 
    aBoard.saveSVG("testOnePoint.svg");
    trace.endBlock();

    return true;
  } catch (std::exception e) {
    return false;
  }


}
/**
 * Test when init with two end iterators
 *
 */
bool testTwoEndIterators()
{
  typedef int Coordinate;
  typedef PointVector<2,Coordinate> Point;
  typedef ArithmeticalDSS<std::vector<Point>::iterator,Coordinate,4> PrimitiveType;
  
  typedef GreedyDecomposition<PrimitiveType> DecompositionType;

  std::vector<Point> curve;
  curve.push_back(Point(5,5));

  try {

    trace.beginBlock("Two end iterators");
    PrimitiveType computer;
    DecompositionType theDecomposition(curve.begin(), curve.end(), computer, false);

    for ( DecompositionType::SegmentIterator i = theDecomposition.begin();
                                           i != theDecomposition.end(); ++i ) 
      {        } 

    trace.endBlock();

    return true;
  } catch (std::exception e) {
    return false;
  }


}

/**
 * Test for the segmentation of 
 * one DSS into DSSs
 *
 */
bool testOneDSS()
{

  typedef int Coordinate;
  typedef PointVector<2,Coordinate> Point;
  typedef ArithmeticalDSS<std::vector<Point>::iterator,Coordinate,8> PrimitiveType;
  typedef GreedyDecomposition<PrimitiveType> DecompositionType;

  std::vector<Point> curve;
  curve.push_back(Point(0,0));
  curve.push_back(Point(1,1));
  curve.push_back(Point(2,1));
  curve.push_back(Point(3,2));
  curve.push_back(Point(4,2));
  curve.push_back(Point(5,2));
  curve.push_back(Point(6,3));
  curve.push_back(Point(7,3));

  //Segmentation
  trace.beginBlock("Segmentation of one DSS");
  PrimitiveType computer;
  DecompositionType theDecomposition(curve.begin(), curve.end(), computer, false);

  // Draw the pixels
  Board2D aBoard;
  aBoard.setUnit(Board::UCentimeter);
  aBoard << SetMode("PointVector", "Both");
  for (std::vector<Point>::iterator it = curve.begin(); it != curve.end(); ++it) {
    aBoard << (*it);
  }
         
  //for each segment
  unsigned int compteur = 0;
  DecompositionType::SegmentIterator i = theDecomposition.begin();
  for ( ; i != theDecomposition.end(); ++i) {

    ++compteur;
    PrimitiveType segment(*i);     
    trace.info() << segment << std::endl;  //standard output
    aBoard << SetMode( "ArithmeticalDSS", "BoundingBox" )
           << segment; // draw each segment    
  } 

  aBoard.saveSVG("oneDSS.svg");

  trace.endBlock();

  return (compteur==1);
}

/**
 * Test for the segmentation of 
 * 8-connected digital curves into DSS
 *
 */
bool testDec8Reverse()
{

  typedef int Coordinate;
  typedef PointVector<2,Coordinate> Point;
  typedef std::vector<Point> ContourType;
  typedef ArithmeticalDSS<ContourType::reverse_iterator,Coordinate,8> PrimitiveType;
  typedef GreedyDecomposition<PrimitiveType> DecompositionType;

  std::vector<Point> curve;
curve.push_back(Point(1,1));                    //input elements
curve.push_back(Point(2,1));
curve.push_back(Point(3,2));
curve.push_back(Point(4,2));
curve.push_back(Point(5,2));
curve.push_back(Point(6,2));
curve.push_back(Point(7,2));
curve.push_back(Point(8,1));
curve.push_back(Point(9,1));

  //Segmentation
  trace.beginBlock("Segmentation of a 8-connected digital curve into DSS");
  PrimitiveType computer;
  DecompositionType theDecomposition(curve.rbegin(), curve.rend(), computer, false);
  
  // Draw the pixels
  Board2D aBoard;
  aBoard.setUnit(Board::UCentimeter);
  aBoard << SetMode("PointVector", "Both");
  for (ContourType::iterator it = curve.begin(); it != curve.end(); ++it) {
    aBoard << (*it);
  }
         

  //for each segment
  unsigned int compteur = 0;
  DecompositionType::SegmentIterator i = theDecomposition.begin();
  for ( ; i != theDecomposition.end(); ++i) {

    compteur++;
    trace.info() << "Segment " << compteur << std::endl;
    PrimitiveType segment(*i);     
    trace.info() << segment << std::endl;  //standard output
    aBoard << SetMode( "ArithmeticalDSS", "BoundingBox" )
           << segment; // draw each segment    
  } 

  aBoard.saveSVG("right_left.svg");

  trace.endBlock();

  return (compteur==2);
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
          && testOpenCurvesProcessedAsClosed()
          && testNoPoint()
          && testOnePoint()
          && testTwoEndIterators()
          && testOneDSS()
          && testDec8Reverse();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();

  return res ? 0 : 1;

}
