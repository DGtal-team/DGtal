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
 * @file exampleConstImageAdapter.cpp
 * @ingroup Examples
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/12/4
 *
 * @brief An example file for ConstImageAdapter.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"

#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/io/colormaps/GrayscaleColorMap.h"

#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/ConstImageAdapter.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
  using namespace Z2i;
  
  Board2D aBoard;
  typedef HueShadeColorMap<unsigned char> HueShade;
  typedef GrayscaleColorMap<unsigned char> Gray;

  trace.beginBlock("image");

    typedef ImageContainerBySTLVector<Domain, unsigned char> Image;
    Domain domain(Point(1,1), Point(16,16));
    Image image(domain);
    
    unsigned int i = 0;
    for (Image::Iterator it = image.begin(); it != image.end(); ++it)
        *it = (unsigned char)i++;
    
    aBoard.clear();
    Display2DFactory::drawImage<HueShade>(aBoard, image, (unsigned char)0, (unsigned char)255);
    aBoard.saveSVG("image.svg");
  
  trace.endBlock();
  
  trace.beginBlock("subImage");
  
    typedef ConstImageAdapter<Image, Domain, DefaultFunctor, Image::Value, DefaultFunctor > ConstImageAdapter1;
    Domain subDomain(Point(1,1), Point(8,8));
    DefaultFunctor g1, f1;
    ConstImageAdapter1 subImage(image, subDomain, g1, g1);
    
    aBoard.clear();
    Display2DFactory::drawImage<HueShade>(aBoard, subImage, (unsigned char)0, (unsigned char)255);
    aBoard.saveSVG("subImage.svg");
    
  trace.endBlock();
  
  trace.beginBlock("specificImage");
  
    DigitalSet set(domain);
    unsigned int j = 0;
    for (Domain::ConstIterator it = domain.begin(); it != domain.end(); ++it, ++j )
      if (j%2) set.insertNew(*it);   
  
    typedef ConstImageAdapter<Image, DigitalSetDomain<DigitalSet>, DefaultFunctor, Image::Value, DefaultFunctor > ConstImageAdapter2;
    DigitalSetDomain<DigitalSet> specificDomain(set);
    DefaultFunctor g2, f2;
    ConstImageAdapter2 specificImage(image, specificDomain, g2, f2);
    
    aBoard.clear();
    Display2DFactory::drawImage<HueShade>(aBoard, specificImage, (unsigned char)0, (unsigned char)255);
    aBoard.saveSVG("specificImage.svg");
    
  trace.endBlock();
  
  trace.beginBlock("thresholderImage");
  
    typedef ConstImageAdapter<Image, Domain, DefaultFunctor, bool, Thresholder<Image::Value> > ConstImageAdapter3;
    DefaultFunctor g3; Thresholder<Image::Value> t(127);
    ConstImageAdapter3 thresholderImage(image, domain, g3, t);
    
    aBoard.clear();
    Display2DFactory::drawImage<Gray>(aBoard, thresholderImage, (unsigned char)0, (unsigned char)1);
    aBoard.saveSVG("thresholderImage.svg");
    
  trace.endBlock();
  
  trace.beginBlock("newDomainImage");
  
    typedef ConstImageAdapter<Image, Domain, DefaultFunctor, float, CastFunctor<float> > ConstImageAdapter4;
    DefaultFunctor g4; CastFunctor<float> f4;
    ConstImageAdapter4 newDomainImage(image, domain, g4, f4);
    
    aBoard.clear();
    Display2DFactory::drawImage<HueShade>(aBoard, newDomainImage, (float)0, (float)255);
    aBoard.saveSVG("newDomainImage.svg");
    
  trace.endBlock();
  
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
