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
 * @file convex-and-concave-parts.cpp
 * @ingroup Examples
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2011/01/24
 *
 * An example file named convex-and-concave-parts.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include "DGtal/base/Common.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/Color.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/curves/ArithmeticalDSS.h"
#include "DGtal/geometry/curves/FreemanChain.h"
#include "DGtal/geometry/curves/SaturatedSegmentation.h"
///////////////////////////////////////////////////////////////////////////////
#include "ConfigExamples.h"

using namespace std;
using namespace DGtal;
using namespace Z2i;

///////////////////////////////////////////////////////////////////////////////

typedef FreemanChain<int> Range; 

///////////////////////////////////////////////////////////////////////////////

/**
 * Drawing a segmentation
 */
template <typename Iterator, typename Board>
void drawCCP(const Iterator& itb, const Iterator& ite, Board& aBoard)
{
  
  typedef typename Iterator::SegmentComputer::ConstIterator PointIterator; 

  aBoard << SetMode( "ArithmeticalDSS", "BoundingBox" );
  string aStyleName = "ArithmeticalDSS/BoundingBox";

  for (Iterator i(itb); i != ite; ++i) {
     
    //choose pen color
    CustomPenColor* aPenColor;

    if ( !(i.intersectNext() && i.intersectPrevious()) ) {

      aPenColor = new CustomPenColor( Color::Black );

    } else {

      //end points

      PointIterator begin = i->begin();  --begin; 
      PointIterator end = i->end();

      //parameters
      int mu = i->getMu();
      int omega = i->getOmega();

      //configurations
      if ( (i->getRemainder(begin)<=mu-1)&&
           (i->getRemainder(end)<=mu-1) ) {                //concave
        aPenColor = new CustomPenColor( Color::Green);
      } else if ( (i->getRemainder(begin)>=mu+omega)&&
            (i->getRemainder(end)>=mu+omega) ) {           //convex
        aPenColor = new CustomPenColor( Color::Blue );
      } else if ( (i->getRemainder(begin)>=mu+omega)&&
            (i->getRemainder(end)<=mu-1) ) {               //convex to concave
        aPenColor = new CustomPenColor( Color::Yellow );
      } else if ( (i->getRemainder(begin)<=mu-1)&&
            (i->getRemainder(end)>=mu+omega) ) {           //concave to convex
        aPenColor = new CustomPenColor( Color::Yellow );
      } else {                                                    //pb
        aPenColor = new CustomPenColor( Color::Red );
      }

    }

    // draw each segment
    aBoard << CustomStyle( aStyleName, aPenColor )
           << *i; 
  
  } 

}

/**
 * saturated segmentation of a range
 */
template <typename Iterator, typename Board>
void segmentationIntoMaximalDSSs(const Iterator& itb, const Iterator& ite, 
                                 Board& aBoard)
{
  typedef typename IteratorCirculatorTraits<Iterator>::Value::Coordinate Coordinate; 
  typedef ArithmeticalDSS<Iterator,Coordinate,4> RecognitionAlgorithm;
  typedef SaturatedSegmentation<RecognitionAlgorithm> Segmentation;

  RecognitionAlgorithm algo;
  Segmentation s(itb,ite,algo);
  
  typename Segmentation::SegmentComputerIterator i = s.begin();
  typename Segmentation::SegmentComputerIterator end = s.end();

  drawCCP<typename Segmentation::SegmentComputerIterator, Board>
  (i,end,aBoard); 

}


int main( int argc, char** argv )
{

  trace.beginBlock ( "Example convex-and-concave-parts" );

  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;


  string codes; 
  if (argc >= 2) codes = argv[1];
  else codes = "030030330303303030300001010101101011010000030330303303030300001010110101011010000033"; 

  stringstream ss(stringstream::in | stringstream::out);
  ss << "0 0 " << codes << endl;
  Range theContour( ss );
  
  trace.info() << "Processing of " << ss.str() << endl;

  //Maximal Segments
  Board2D aBoard;
  aBoard
   << SetMode( "PointVector", "Grid" )
   << theContour;

  segmentationIntoMaximalDSSs(theContour.begin(), theContour.end(), aBoard);

  aBoard.saveSVG("convex-and-concave-parts.svg");

  trace.endBlock();

  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
