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
 * @file testCompressedVolWriter.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2016/12/26
 *
 * Functions for testing class CompressedVolWriter.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/writers/VolWriter.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class CompressedVolWriter.
///////////////////////////////////////////////////////////////////////////////


TEST_CASE( "Testing CompressedVolWriter" )
{
  Domain domain(Point(0,0,0), Point(10,10,10));
  typedef ImageContainerBySTLVector<Domain, unsigned char> Image;
  Image image(domain);
  
  
  SECTION("Testing API of CompressedVolWriter")
    {
      VolWriter< ImageContainerBySTLVector<Domain, unsigned char> >::exportVol("test.vol", image);
      
      VolWriter< ImageContainerBySTLVector<Domain, unsigned char> >::exportVol("testz.vol", image,functors::Identity(), true);
      
      REQUIRE( image.isValid() );
    }
  
  SECTION("Testing write/read of CompressedVolWriter")
    {
      
      
    }

}

/** @ingroup Tests **/
