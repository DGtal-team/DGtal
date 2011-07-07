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
 * @file testL1LengthEstimator.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/06/27
 *
 * Functions for testing class L1LengthEstimator.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/DigitalSetSelector.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/SurfelAdjacency.h"
#include "DGtal/topology/SurfelNeighborhood.h"


#include "DGtal/geometry/2d/GridCurve.h"
#include "DGtal/geometry/2d/L1LengthEstimator.h"


#include "ConfigTest.h"


///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class L1LengthEstimator.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testL1LengthEstimator(std::string &filename)
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.info() << "Reading GridCurve " << endl;
  
  ifstream instream; // input stream
  instream.open (filename.c_str(), ifstream::in);

  GridCurve<KhalimskySpaceND<2> > c; //grid curve
  c.initFromVectorStream(instream);

  GridCurve<KhalimskySpaceND<2> >::PointsRange r = c.getPointsRange(); 
  L1LengthEstimator<  GridCurve<KhalimskySpaceND<2> >::PointsRange::ConstIterator > l1length;
    
  l1length.init(1, r.begin(), r.end(), false);
  trace.info() << "L1 length (h=1) = "<< l1length.eval()<<std::endl;

  l1length.init(10, r.begin(), r.end(), false);
  trace.info() << "L1 length (h=10) = "<< l1length.eval()<<std::endl;
    
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class L1LengthEstimator" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  std::string sinus2D4 = testPath + "samples/sinus2D4.dat";



  bool res = testL1LengthEstimator(sinus2D4); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
