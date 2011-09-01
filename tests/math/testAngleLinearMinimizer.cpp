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
 * @file testAngleLinearMinimizer.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/09/01
 *
 * Functions for testing class AngleLinearMinimizer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/math/AngleLinearMinimizer.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class AngleLinearMinimizer.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testAngleLinearMinimizer()
{
  unsigned int nbok = 1;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing AngleLinearMinimizer ." );
  nbok += true ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "true == true" << std::endl;
   
  AngleLinearMinimizer alm;
  alm.setSize(10);
  double valDec [10] = {0.2, 0.3, -0.2, -0.2, -0.1, 0.0, 0.0, 0.0,  0.0, 1.0};
  double valDecMin [10] = {-1, -2, -1, -0.2, -0.1, -1.0, -1.2, -0.5,  -0.3, -0.4};
  double valDecMax [10] = {0.2, 0.3, 0.2, 1.2, 1.1, 2.0, 0.5, 0.2,  0.1, 0.8};
  for(unsigned int i=0; i<10; i++){
    AngleLinearMinimizer::ValueInfo vi;
    double val = i + valDec[i];
    vi.value = val;
    vi.oldValue = val;
    vi.min = val + valDecMin[i];
    vi.max = val + valDecMax[i];
    vi.distToNext = 4.0;    
  }
  
  while(alm.optimize()>0.00001){
  }
  
  trace.endBlock();
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class AngleLinearMinimizer" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testAngleLinearMinimizer(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
