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
 * @file testArithDSS.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 *
 * @date 2011/06/01
 *
 * This file is part of the DGtal library
 */

/**
 * Description of testArithDSS <p>
 * Aim: simple test of \ref ArithmeticalDSS3d
 */




#include <iostream>
#include <iterator>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <vector>

#include "DGtal/base/Common.h"
#include "DGtal/base/Exceptions.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/geometry/3d/ArithmeticalDSS3d.h"

#ifdef WITH_GMP
#include <gmpxx.h>
#endif

using namespace DGtal;
using namespace std;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ArithmeticalDSS.
///////////////////////////////////////////////////////////////////////////////
/**
 * Test
 *
 */
bool testDSSreco()
{

	typedef PointVector<3,int> Point;
	typedef std::vector<Point>::iterator Iterator;
	typedef ArithmeticalDSS<Iterator,int,4> SegmentComputer;  

	std::vector<Point> sequence;
	sequence.push_back(Point(0,0,0));
	sequence.push_back(Point(1,0,0));
	sequence.push_back(Point(2,0,0));
	sequence.push_back(Point(2,1,0));
	sequence.push_back(Point(2,1,1));
	sequence.push_back(Point(3,1,1));
	sequence.push_back(Point(4,1,1));
	sequence.push_back(Point(4,2,1));
	sequence.push_back(Point(4,2,2));
	sequence.push_back(Point(5,2,2));
  
  // Adding step
  trace.beginBlock("Add points while it is possible and display the result");

		SegmentComputer algo;	
		Iterator i = sequence.begin();	
		algo.init(i);
		i++;
		trace.info() << algo << " " << algo.isValid() << std::endl;

		while ( (i!=sequence.end())
					&&(algo.extend(i)) ) {
			i++;
		}
	  trace.info() << algo << " " << algo.isValid() << std::endl;

  trace.endBlock();

	return true;  
}


int main(int argc, char **argv)
{

  trace.beginBlock ( "Testing class ArithmeticalDSS" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testDSSreco;
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();

  return res ? 0 : 1;

}
