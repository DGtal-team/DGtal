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
 * @file testHalfPlane.cpp
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
 * Description of testPreimage <p>
 * Aim: simple test of \ref Preimage2D
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

#include "DGtal/geometry/2d/StraightLine.h"
#include "DGtal/geometry/2d/Preimage2D.h"
	

using namespace DGtal;

using namespace LibBoard;


/**** small test, to be completed *******/

int main(int argc, char **argv)
{

	typedef int Coordinate;
	typedef PointVector<2,Coordinate> Point;
	typedef StraightLine<Coordinate> StraightLine;
  typedef Preimage2D<StraightLine> Preimage2D; 

	//data
	std::vector<Point> bInf, bSup;
	bInf.push_back(Point(0,10));
	bInf.push_back(Point(1,11));
	bInf.push_back(Point(2,11));
	bInf.push_back(Point(3,12));
	bInf.push_back(Point(4,10));
	bInf.push_back(Point(5,10));
	bInf.push_back(Point(6,11));
	bInf.push_back(Point(7,12));
	bInf.push_back(Point(8,12));
	bInf.push_back(Point(9,13));

	bSup.push_back(Point(0,12));
	bSup.push_back(Point(1,13));
	bSup.push_back(Point(2,13));
	bSup.push_back(Point(3,14));
	bSup.push_back(Point(4,13));
	bSup.push_back(Point(5,12));
	bSup.push_back(Point(6,13));
	bSup.push_back(Point(7,14));
	bSup.push_back(Point(8,14));
	bSup.push_back(Point(9,15));


  //Location
  trace.beginBlock("Computation of the preimage");

	Preimage2D thePreimage(bInf.at(0),bSup.at(0));
	for (int i = 1; i<(bInf.size()-1); ++i) {
		thePreimage.addFront(bInf.at(i),bSup.at(i));
  	trace.info() << "--- Constraint number " << i << " ---" << std::endl;
  	trace.info() << thePreimage << std::endl;
	}





  trace.endBlock();


  return 0;
}
