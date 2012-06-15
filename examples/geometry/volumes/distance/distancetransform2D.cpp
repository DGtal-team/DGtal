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
 * @file distancetransform2D.cpp
 * @ingroup Examples
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/11/28
 *
 * An example file named distancetransform2D. The aim of this example
 * is to demonstrate the distance transformation package of the DGtal
 * library. Using a simple random seed example, we compute the
 * distance transformation (linear in time algorithm) using the L1,
 * Linfinity and L2 metrics.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <iomanip>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/colormaps/GrayscaleColorMap.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/geometry/volumes/distance/DistanceTransformation.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

/** 
 * Set to a given value a random set of @param nb points.
 * 
 * @param image the  image
 * @param nb the number of random points to insert
 * @param value the value to add at each random point
 */
template<typename Image>
void randomSeeds(Image &image, const unsigned int nb, const int value)
{
  typename Image::Point p, low = image.domain().lowerBound();
  typename Image::Vector ext;

  ext = image.extent();

  for (unsigned int k = 0 ; k < nb; k++)
  {
    for (unsigned int dim = 0; dim < Image::dimension; dim++)
      p[dim] = rand() % (ext[dim]) +  low[dim];
    
    image.setValue(p, value);
  }
}

int main()
{
  trace.beginBlock ( "Example distancetransform2D" );

  Z2i::Point a ( 0, 0 );
  Z2i::Point b ( 127, 127);
  
  //Input image with unsigned char values
  typedef ImageSelector<Z2i::Domain, unsigned int>::Type Image;
  Image image ( Z2i::Domain(a, b ));

  //We fill the image with the 128 value
  for ( Image::Iterator it = image.begin(), itend = image.end();it != itend; ++it)
    (*it)=128;
  //We generate 16 seeds with 0 values.
  randomSeeds(image,50,0);

  //Colormap used for the SVG output
  typedef HueShadeColorMap<long int, 2> HueTwice;
  typedef GrayscaleColorMap<unsigned char> Gray;
  
  //Input shape output
  Board2D board;
  board.setUnit ( LibBoard::Board::UCentimeter );
  Display2DFactory::drawImage<Gray>(board, image, (unsigned int)0, (unsigned int)129);
  board.saveSVG("inputShape.svg");

  typedef  DistanceTransformation<Image, 2> DTL2;
  typedef  DistanceTransformation<Image, 0> DTLInf;
  typedef  DistanceTransformation<Image, 1> DTL1;
 
  DTL2 dtL2;
  DTLInf dtLinf;
  DTL1 dtL1;
  
  DTL2::OutputImage resultL2 = dtL2.compute ( image );
  DTLInf::OutputImage resultLinf = dtLinf.compute ( image );
  DTL1::OutputImage resultL1 = dtL1.compute ( image );
  
  unsigned int maxv=0;
  //We compute the maximum DT value on the Linf map
  for ( DTLInf::OutputImage::ConstIterator it = resultLinf.begin(), itend = resultLinf.end();it != itend; ++it)
    if ( (*it) > maxv)  maxv = (unsigned int)(*it);
  unsigned int maxv2=0;
  //We compute the maximum DT value on the L2 map
  for ( DTL2::OutputImage::ConstIterator it = resultL2.begin(), itend = resultL2.end();it != itend; ++it)
    if ( (*it) > maxv2)  maxv2 = (unsigned int)(*it);
  unsigned int maxv1=0;
  //We compute the maximum DT value on the L1 map
  for ( DTL1::OutputImage::ConstIterator it = resultL1.begin(), itend = resultL1.end();it != itend; ++it)
    if ( (*it) > maxv1)  maxv1 = (unsigned int)(*it);
  
  
  trace.warning() << resultL2 << " maxValue= "<<maxv2<< endl;
  board.clear();
  Display2DFactory::drawImage<HueTwice>(board, resultL2, (DGtal::int64_t)0, (DGtal::int64_t)maxv2 + 1);
  board.saveSVG ( "example-DT-L2.svg" );

  trace.warning() << resultL1 << " maxValue= "<<maxv1<< endl;
  board.clear();
  Display2DFactory::drawImage<HueTwice>(board, resultL1, (DGtal::int64_t)0, (DGtal::int64_t)maxv1 + 1);
  board.saveSVG ( "example-DT-L1.svg" );

  trace.warning() << resultLinf << " maxValue= "<<maxv<< endl;
  board.clear();
  Display2DFactory::drawImage<HueTwice>(board, resultLinf, (DGtal::int64_t)0, (DGtal::int64_t)maxv + 1);
  board.saveSVG ( "example-DT-Linf.svg" );

  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
