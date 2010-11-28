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
 * An example file named distancetransform2D.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/colormaps/HueShadeColormap.h"
#include "DGtal/io/DGtalBoard.h"
#include "DGtal/kernel/images/ImageSelector.h"
#include "DGtal/geometry/nd/volumetric/DistanceTransformation.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace DGtal::Z2i;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  trace.beginBlock ( "Example distancetransform2D" );

  Point a ( 0, 0 );
  Point b ( 15, 15 );
  typedef ImageSelector<Domain, unsigned int>::Type Image;
  typedef HueShadeColorMap<unsigned char, 2> HueTwice;
  Image image ( a, b );
  Domain domain(a,b);

  //We construct a ball
  for ( Domain::ConstIterator it = domain.begin(), itend = domain.end() ;
	it != itend;
	++it)
    {
      int x = (*it)[0];
      int y = (*it)[1];
      if  (  (x - 7)*(x - 7) + (y - 7)*(y - 7) < 25)
	image.setValue ( *it, 128 );
    }

  typedef ImageSelector<Domain, long int>::Type ImageLong;

  DistanceTransformation<Image, ImageLong, L2Metric> dt;
  DGtalBoard board;

  board.setUnit ( LibBoard::Board::UCentimeter );

  ImageLong result = dt.compute ( image );

  trace.warning() << result << endl;

  result.selfDraw<HueTwice> ( board, 0, 16 );
  board.saveSVG ( "image-postDT.svg" );

  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
