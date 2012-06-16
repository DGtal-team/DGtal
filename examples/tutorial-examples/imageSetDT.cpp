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
#include <algorithm>
///////////////////////////////////////////////////////////////////////////////

//! [ImageSetDT-includes]
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

#include "DGtal/base/BasicFunctors.h"
#include "DGtal/kernel/BasicPointPredicates.h"
#include "DGtal/kernel/sets/DigitalSetInserter.h"

#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/ImageHelper.h"
#include "DGtal/geometry/volumes/distance/DistanceTransformation.h"

#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/readers/PNMReader.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/io/colormaps/GrayscaleColorMap.h"

#include "ConfigExamples.h"
//! [ImageSetDT-includes]


///////////////////////////////////////////////////////////////////////////////

int main()
{
  //! [ImageSetDT-types]
  typedef DGtal::ImageContainerBySTLVector< Z2i::Domain, unsigned char> Image;
  typedef DGtal::GrayscaleColorMap<unsigned char> Gray;
  //! [ImageSetDT-types]


  //! [ImageSetDT-image]
  std::string filename =  examplesPath + "samples/contourS.pgm";
  Image image = DGtal::PNMReader<Image>::importPGM(filename); 
  DGtal::trace.info() << "Imported image: "<<image<<endl;
  //! [ImageSetDT-image]


  //! [ImageSetDT-board1]
  DGtal::Board2D aBoard;
  aBoard << image.domain();  
  aBoard.saveSVG("imageDomainTuto.svg");
  aBoard.clear();
  Display2DFactory::drawImage<Gray>(aBoard, image, (unsigned char)0, (unsigned char)255);
  aBoard.saveEPS("imageDomainTuto2.eps");
  //! [ImageSetDT-board1]


  Z2i::DigitalSet mySet(image.domain());
  DigitalSetInserter<Z2i::DigitalSet> inserter(mySet); 
  setFromImage(image, inserter, 1, 135);
  aBoard.clear();
  aBoard << mySet.domain()
	 << mySet;
  aBoard.saveEPS("imageDomainTuto2bis.eps");


  //! [ImageSetDT-DT]
  typedef DGtal::DistanceTransformation<Image, 2> DTL2;
  typedef DTL2::OutputImage OutputImage;
  DTL2 dt;

  typedef IntervalThresholder<Image::Value> Binarizer; 
  Binarizer b(1, 135); 
  OutputImage result = dt.compute(image, 
            PointFunctorPredicate<Image,Binarizer>(image, b)); 
  //! [ImageSetDT-DT]
 

  //! [ImageSetDT-DTvis]
  OutputImage::Value maxDT = (*std::max_element(result.begin(), 
						result.end()));
  typedef DGtal::HueShadeColorMap<OutputImage::Value,2> HueTwice;

  aBoard.clear();
  Display2DFactory::drawImage<HueTwice>(aBoard, result, (OutputImage::Value)0, (OutputImage::Value)maxDT);
  aBoard.saveEPS("imageDomainTuto3.eps");
  //! [ImageSetDT-DTvis]

  return 0;

}

///////////////////////////////////////////////////////////////////////////////
