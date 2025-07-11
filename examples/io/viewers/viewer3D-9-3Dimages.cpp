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
 * @file io/viewers/viewer3D-9-3Dimages.cpp
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2013/05/19
 *
 * An example file named viewer3D-9-3Dimages.
 *
 * This file is part of the DGtal library.
 */



/**
 * \image html  viewer3D-9.png  "Illustration of the 3D image visualization."
 *  \example io/viewers/viewer3D-9-3Dimages.cpp
 *
 */



///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/readers/GenericReader.h"
#include "DGtal/images/ImageHelper.h"
#include "DGtal/images/Image.h"
#include "ConfigExamples.h"

#include "DGtal/helpers/StdDefs.h"

#include "DGtal/io/viewers/PolyscopeViewer.h"
#include "DGtal/io/Color.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

int main( int argc, char** argv )
{
  PolyscopeViewer viewer;

  typedef ImageContainerBySTLVector<Z3i::Domain,  unsigned char > Image3D;

  std::string inputFilename = examplesPath + "samples/lobster.vol";
  Image3D imageVol = GenericReader<Image3D>::import(inputFilename);

  Z3i::Point ptLow (100, 100, 20);
  Z3i::Point ptUpp (200, 200, 40);
  Z3i::Domain subDomain(ptLow, ptUpp);

  Z3i::Point ptLow2 (220, 50, 10);
  Z3i::Point ptUpp2 (260, 100, 20);
  Z3i::Domain subDomain2(ptLow2, ptUpp2);

  Image3D imageCrop(subDomain);
  Image3D imageCrop2(subDomain2);
  
  // Adding too many single point is slow and will 
  // makke the view laggy
  // This parameter allows to reuse some list if 
  // possible (but it won't work if multiples types are 
  // inserted)
  viewer.allowReuseList = true;

  for(Z3i::Domain::ConstIterator it= imageVol.domain().begin(), itend = imageVol.domain().end(); it != itend; ++it){
    if(imageVol(*it)>140)
      viewer << *it;
    Z3i::Point pt = *it;
    if(pt[0]>=ptLow[0] && pt[1] >= ptLow[1] && pt[2] >= ptLow[2] &&
       pt[0]<=ptUpp[0] && pt[1] <= ptUpp[1] && pt[2] <= ptUpp[2]){
      imageCrop.setValue(*it, imageVol(*it));
    }

    if(pt[0]>=ptLow2[0] && pt[1] >= ptLow2[1] && pt[2] >= ptLow2[2] &&
       pt[0]<=ptUpp2[0] && pt[1] <= ptUpp2[1] && pt[2] <= ptUpp2[2]){
      imageCrop2.setValue(*it, imageVol(*it));
    }
  }
  viewer << imageCrop;

  viewer.show();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
