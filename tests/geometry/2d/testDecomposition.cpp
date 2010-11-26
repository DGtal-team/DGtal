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

using namespace LibBoard;




int main(int , char **)
{

	typedef int Coordinate;
	typedef PointVector<2,Coordinate> Point;
	typedef ArithmeticalDSS<StandardBase<Coordinate> > PrimitiveType;

  typedef FreemanChain<Coordinate> ContourType; 

  std::string filename = testPath + "samples/manche.fc";
  std::cout << filename << std::endl;

  std::fstream fst;
  fst.open (filename.c_str(), std::ios::in);
  ContourType theContour(fst);

  //Segmentation
  trace.beginBlock("Segmentation of a chain code into DSS");
  GreedyDecomposition<ContourType,PrimitiveType> theDecomposition(theContour);
  
  //Board aBoard;
  //aBoard.setUnit(Board::UCentimeter);
  //theContour.selfDraw(aBoard);

  DGtalBoard aBoard;
  aBoard.setUnit(Board::UCentimeter);
  
  
  //for each segment
  GreedyDecomposition<ContourType,PrimitiveType>::ConstIterator i = 
    theDecomposition.begin();
  for ( ; i != theDecomposition.end(); ++i) {
    trace.info() << "segment number " << i.getPosition() << std::endl;
    PrimitiveType segment(*i); 
    trace.info() << segment << std::endl;	//standard output
    aBoard << segment; // draw each segment
    //segment.selfDraw(aBoard); //drawing
  } 
  aBoard.saveSVG("segmentation.svg");

  trace.endBlock();


  return 0;
}
