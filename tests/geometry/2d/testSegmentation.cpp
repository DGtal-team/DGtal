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
 * @file testSegmentation.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 *
 * @date 2011/07/22
 *
 * This file is part of the DGtal library
 */

/**
 * Description of testSegmentation <p>
 * Aim: simple test of \ref GreedySegmentation
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

#include "DGtal/base/Circulator.h"

#include "DGtal/geometry/2d/ArithmeticalDSS.h"
#include "DGtal/geometry/2d/FreemanChain.h"
#include "DGtal/geometry/2d/GreedySegmentation.h"


#include "ConfigTest.h"


using namespace DGtal;
using namespace std;
using namespace LibBoard;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class GreedySegmentation
///////////////////////////////////////////////////////////////////////////////



/**
 * Drawing a segmentation
 */
template <typename Iterator, typename Board>
void draw(const Iterator& itb, const Iterator& ite, Board& aBoard)
{

  for (Iterator i(itb); i != ite; ++i) {
		
    typename Iterator::SegmentComputer segment(*i); 

    trace.info() << segment << std::endl;	//standard output

    aBoard << SetMode(segment.styleName(), "BoundingBox" )
					 << segment; // draw bounding box
  
  } 

}

/**
 * Segmenting a (sub)range
 */
template <typename Iterator, typename Board>
void segmentationIntoDSSs(const Iterator& itb, const Iterator& ite, 
                          const Iterator& sitb, const Iterator& site,
                          const string& aMode, Board& aBoard)
{
  typedef typename IteratorCirculatorTraits<Iterator>::Value::Coordinate Coordinate; 
  typedef ArithmeticalDSS<Iterator,Coordinate,4> RecognitionAlgorithm;
	typedef GreedySegmentation<RecognitionAlgorithm> Segmentation;

  RecognitionAlgorithm algo;
  Segmentation s(itb,ite,algo);
  s.init(sitb,site);
  s.setMode(aMode);
  
  typename Segmentation::SegmentComputerIterator i = s.begin();
  typename Segmentation::SegmentComputerIterator end = s.end();

  draw<typename Segmentation::SegmentComputerIterator, Board>
  (i,end,aBoard); 

}

/**
 * Simple visual test
 */
bool visualTest()
{

  typedef int Coordinate;
  typedef FreemanChain<Coordinate> FC; 
  typedef FreemanChain<Coordinate>::ConstIterator ConstIterator; 


  std::string filename = testPath + "samples/manche.fc";
  std::cout << filename << std::endl;

  std::fstream fst;
  fst.open (filename.c_str(), std::ios::in);
  FC fc(fst);



  trace.beginBlock("Segmentation of a whole range");
{
  Board2D aBoard;
  aBoard << SetMode("PointVector", "Grid") << fc; 

  segmentationIntoDSSs<ConstIterator,Board2D>
    (fc.begin(),fc.end(),
     fc.begin(),fc.end(),
     "Truncate",aBoard);   

  aBoard.saveEPS("SimpleOpenSeg.eps");
}
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

  bool res = visualTest()
;

  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();

  return res ? 0 : 1;

}
