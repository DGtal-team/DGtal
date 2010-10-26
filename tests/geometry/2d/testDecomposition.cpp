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
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/DomainPredicate.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/io/DGtalBoard.h"

#include "DGtal/geometry/2d/ArithDSS4.h"
#include "DGtal/geometry/2d/FreemanChain.h"
#include "DGtal/geometry/2d/GreedyDecomposition.h"


#include "ConfigTest.h"


using namespace DGtal;

using namespace LibBoard;




int main(int argc, char **argv)
{


  typedef SpaceND<2> Space2Type;
  typedef HyperRectDomain<Space2Type> Domain2D;
	typedef ArithDSS4<Domain2D> PrimitiveType; 
	typedef FreemanChain ContourType; 
  typedef Domain2D::Point Point; 

  std::string filename = testPath + "samples/france.fc";
	std::cout << filename << std::endl;

  std::fstream fst;
  fst.open (filename.c_str(), std::ios::in);
  ContourType theContour(fst);

	//Segmentation
  trace.beginBlock("Segmentation of a chain code into DSS");
	GreedyDecomposition<ContourType,PrimitiveType> theDecomposition(theContour);

  Board aBoard;
  aBoard.setUnit(Board::UCentimeter);

/*
  int minX, maxX, minY, maxY;
  theContour.computeBoundingBox(minX, minY, maxX, maxY);  
  Domain2D theDomain( Point(minX,minY), Point(maxX,maxY) );  
  theDomain.selfDrawAsGrid(aBoard);
*/
  theContour.selfDraw(aBoard);

	//pour tous les segments
	GreedyDecomposition<ContourType,PrimitiveType>::ConstIterator i = 
		theDecomposition.begin();
	for ( ; i != theDecomposition.end(); ++i) {
		PrimitiveType segment(*i); 
  	std::cout << "segment number " << i.getPosition() << std::endl;
  	std::cout << segment << std::endl;	//standard output
  	i.get().selfDraw(aBoard); //drawing
	} 
  aBoard.saveSVG("segmentation.svg");

  trace.endBlock();


  return 0;
}
