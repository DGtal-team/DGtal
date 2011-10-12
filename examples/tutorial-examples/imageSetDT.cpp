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
 * @file imageSetDT.cpp
 * @ingroup Examples
 * @author David Coeurjolly (david.coeurjolly@liris.cnrs.fr)
 *
 *
 * @date 2010/12/01
 * 
 * An example of DT computation from a digital set extracted from an image.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include "DGtal/base/Common.h"
///////////////////////////////////////////////////////////////////////////////

#include "DGtal/helpers/StdDefs.h"

#include "DGtal/images/imagesSetsUtils/SetFromImage.h"
#include "DGtal/images/ImageContainerBySTLVector.h"

#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/readers/PNMReader.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/colormaps/GrayscaleColorMap.h"

#include "ConfigExamples.h"


using namespace std;
using namespace DGtal;
using namespace Z2i;



///////////////////////////////////////////////////////////////////////////////

int main()
{
  typedef GrayscaleColorMap<unsigned char> Gray;
 
  typedef ImageContainerBySTLVector< Z2i::Domain, int> Image;

  Image image = PNMReader<Image>::importPGMImage( examplesPath + "samples/church.pgm" ); 

  trace.info() << "PGM Import ok: "<<image<<endl;

  Board2D aBoard;
  aBoard << image.domain();  
  aBoard.saveSVG("imageDomainTuto.svg");
  
  aBoard.clear();
  image.selfDraw<Gray> ( aBoard, 0, 255 );
  aBoard.saveEPS("imageDomainTuto2.eps");
 

  return 0;

}

///////////////////////////////////////////////////////////////////////////////
