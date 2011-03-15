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
 * @file testDT2Dperf.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/03/03
 *
 * Functions for testing class DT2Dperf.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/images/ImageSelector.h"
#include "DGtal/geometry/nd/volumetric/DistanceTransformation.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DT2Dperf.
///////////////////////////////////////////////////////////////////////////////


template<typename Image>
void randomSeeds(Image &input, const unsigned int nb, const int value)
{
  typename Image::Point p, low = input.lowerBound();
  typename Image::Vector ext;

  ext = input.extent();

  for (Dimension k = 0 ; k < nb; k++)
  {
    for (Dimension dim = 0; dim < Image::dimension; dim++)
    {
      p[dim] = rand() % (ext[dim]) +  low[dim];
    }
    input.setValue(p, value);
  }
}




/**
 * Example of a test. To be completed.
 *
 */
bool testDT2Dperf()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  Z2i::Point a (0, 0 );
  Z2i::Point b ( 500, 500 );

  typedef ImageSelector< Z2i::Domain, unsigned int>::Type Image;
  Image image ( a, b );

  for ( Image::Iterator it = image.begin(), itend = image.end();it != itend; ++it)
    image.setValue ( it, 128);

  randomSeeds(image, 50, 0);

  typedef ImageSelector<Z2i::Domain, long int>::Type ImageLong;


  trace.beginBlock ( "Testing DT2D ..." );
 
  DistanceTransformation<Image, ImageLong, Z2i::L2Metric > dt;

  dt.checkTypesValidity ( image );

  ImageLong result = dt.compute ( image );

  nbok += true ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class DT2Dperf" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testDT2Dperf(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
