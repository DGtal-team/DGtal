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
 * @file gridcurve.cpp
 * @ingroup Examples
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/09/09
 *
 * @brief An example file for GridCurve.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/example/ConfigExamples.h"

#include "DGtal/geometry/2d/GridCurve.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z2i; 

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  trace.beginBlock ( "Example for 2d gridcurves" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  std::string square = examplesPath + "samples/smallSquare.dat";
  Curve c; 
  
  trace.emphase() << "Input" << endl;
  trace.info() << "\t from a data file " << endl;
  {
    fstream inputStream;
    inputStream.open (square.c_str(), ios::in);
    c.initFromVectorStream(inputStream);
    inputStream.close();
  }
  trace.info() << "\t from a points vector " << endl;
// @TODO trace.info() << "\t from a FreemanChain (from a file) " << endl; 


  trace.emphase() << "Output" << endl;
  trace.info() << "\t standard output " << endl;
  trace.info() << "\t into a data file " << endl;
  trace.info() << "\t into a vector graphics file " << endl;
  // @TODO trace.info() << "\t from a FreemanChain (from a file) " << endl; 
  
  trace.emphase() << "Ranges Ouput" << endl;
  
  trace.emphase() << "Ranges Iterators" << endl;
  
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
