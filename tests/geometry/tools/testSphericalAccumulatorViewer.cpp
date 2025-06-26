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
 * @file testSphericalAccumulator.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/09/18
 *
 * Functions for testing class SphericalAccumulator.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
///#include "DGtal/io/viewers/PolyscopeViewer.h"
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/viewers/PolyscopeViewer.h"
#include "DGtal/geometry/tools/SphericalAccumulator.h"
/////////////////////ddzad//////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class SphericalAccumulator.
///////////////////////////////////////////////////////////////////////////////
void testSphericalViewer()
{
  trace.beginBlock ( "Testing Spherical Accumulator Viewer..." );

  typedef Z3i::RealVector Vector;

  SphericalAccumulator<Vector> accumulator(15);
  trace.info()<< accumulator << std::endl;

  for(unsigned int i=0; i< 10000; i++)
    accumulator.addDirection( Vector (1+10.0*(rand()-RAND_MAX/2)/(double)RAND_MAX,
              (1+10.0*(rand()-RAND_MAX/2))/(double)RAND_MAX,
              (1+10.0*(rand()-RAND_MAX/2))/(double)RAND_MAX));

  PolyscopeViewer<> viewer;
  Vector a,b,c,d;
  viewer << accumulator;
  
  trace.info() << "Bin values: ";
  for(SphericalAccumulator<Vector>::ConstIterator it=accumulator.begin(), itend=accumulator.end();
      it != itend;
      ++it)
    trace.info() << *it<<" ";
  trace.info() << std::endl;
  trace.info() << accumulator<<std::endl;

  trace.emphase() << "Passed." << endl;
  trace.endBlock();
  viewer.show();
}

void testSphericalViewerInteger()
{
  trace.beginBlock ( "Testing Spherical Accumulator Viewer  with Integer numbers..." );

  typedef Z3i::Vector Vector;

  SphericalAccumulator<Vector> accumulator(15);
  trace.info()<< accumulator << std::endl;

  for(unsigned int i=0; i< 10000; i++)
    accumulator.addDirection( Vector (1+(rand()-RAND_MAX/2),
                                      (1+(rand()-RAND_MAX/2)),
                                      (1+(rand()-RAND_MAX/2))));

  PolyscopeViewer<> viewer;
  viewer << accumulator;
  // Display3DFactory<Space,KSpace>::draw(viewer,accumulator, Z3i::RealVector(1.0,1.0,1.0), 3.0);

    trace.info() << "Bin values: ";
  for(SphericalAccumulator<Vector>::ConstIterator it=accumulator.begin(), itend=accumulator.end();
      it != itend;
      ++it)
    trace.info() << *it<<" ";
  trace.info() << std::endl;
  trace.info() << accumulator<<std::endl;

  trace.emphase() << "Passed." << endl;
  trace.endBlock();
  viewer.show();
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class SphericalAccumulator" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  testSphericalViewer();
  testSphericalViewerInteger();
  trace.emphase() << "Passed." << endl;
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
