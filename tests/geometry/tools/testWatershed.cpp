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
 * @file testWatershed.cpp
 * @ingroup Tests
 * @author Jérémy Gaillard (\c jeremy.gaillard@insa.lyon.fr )
 * Institut National des Sciences Appliquées - INSA, France
 *
 *
 * @date 2012/07/05
 *
 * This file is part of the DGtal library
 */

/**
 * @brief Aim: simple test of Watershed
 */

#include <cstdio>
#include <cmath>
#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>
#include <iterator>




#include "DGtal/base/Common.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/Color.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/images/imagesSetsUtils/SetFromImage.h"

#include "DGtal/geometry/tools/Watershed.h"


#include "ConfigTest.h"

using namespace std;
using namespace DGtal;
using namespace Z2i;

typedef ImageContainerBySTLVector < Domain, int > Image;

int main()
{
  //! [Initialization]
  Point p1(0,0);
  Point p2(4,4);
  Domain domain(p1, p2);
  
  
  Image image(domain);
  image.setValue(Point(0,0), 1); image.setValue(Point(1,0), 1); image.setValue(Point(2,0), 2); image.setValue(Point(3,0), 1); image.setValue(Point(4,0), 1);
  image.setValue(Point(0,1), 1); image.setValue(Point(1,1), 1); image.setValue(Point(2,1), 2); image.setValue(Point(3,1), 1); image.setValue(Point(4,1), 1);
  image.setValue(Point(0,2), 4); image.setValue(Point(1,2), 1); image.setValue(Point(2,2), 2); image.setValue(Point(3,2), 1); image.setValue(Point(4,2), 1);
  image.setValue(Point(0,3), 1); image.setValue(Point(1,3), 1); image.setValue(Point(2,3), 3); image.setValue(Point(3,3), 2); image.setValue(Point(4,3), 1);
  image.setValue(Point(0,4), 1); image.setValue(Point(1,4), 1); image.setValue(Point(2,4), 4); image.setValue(Point(3,4), 1); image.setValue(Point(4,4), 3);

  DigitalSet set(domain);
  
  SetFromImage<DigitalSet>::append<Image>(set, image, 0, 5);
  
  Object4_8 object(dt4_8, set);
  //! [Initialization]
  
  trace.beginBlock("Simple Watershed test");
  Watershed<Object4_8, Image> ws(object, image);
  Image result = ws.segmentation();
  trace.endBlock();
  
  Board2D board;
  board << SetMode( domain.className(), "Paving" )
  << domain
  << SetMode( p1.className(), "Paving" );
  string specificStyle = p1.className() + "/Paving";
  
  GradientColorMap<int> cmap_grad( 1, 3 );
  cmap_grad.addColor( Color( 50, 50, 255 ) );
  cmap_grad.addColor( Color( 255, 0, 0 ) );
  cmap_grad.addColor( Color( 255, 255, 10 ) );
  
  for ( typename Object4_8::DigitalSet::ConstIterator it = object.pointSet().begin();
  it != object.pointSet().end();
  ++it )
  {
    if( result(*it) == Watershed<Object4_8, Image>::WSHED )
    {
      board << CustomStyle( specificStyle,
	  new CustomColors( Color::Black,
	  Color::Black ) )
	<< *it;
    }
    else
    {
      cout << result(*it) << " " << image(*it) << endl;
      board << CustomStyle( specificStyle,
	  new CustomColors( Color::Black,
	  cmap_grad( result(*it) ) ) )
	<< *it;
    }
    
  }
  
  board.saveEPS("testWatershed.eps");

  return 0;
}
