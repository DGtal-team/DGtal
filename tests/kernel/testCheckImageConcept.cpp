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
 * @file testCheckImageConcept.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/11/28
 *
 * Functions for testing the models of the ImageContainer concept.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/images/ImageContainerBySTLVector.h"
#include "DGtal/kernel/images/ImageContainerBySTLMap.h"
#include "DGtal/kernel/images/ImageContainerByITKImage.h"
#include "DGtal/kernel/images/ImageContainerByHashTree.h"
#include "DGtal/kernel/images/CImageContainer.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z2i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class CheckImageConcept.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testCheckImageConcept()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing block ..." );
  
  typedef ImageContainerBySTLVector<Domain, int> ImageVector;
  typedef ImageContainerBySTLVector<Domain, int> ImageMap;
  typedef experimental::ImageContainerByITKImage<Domain, int> ImageITK;

  //HashTree is not (yet) a model of CImageContainer
  //typedef experimental::ImageContainerByHashTree<Domain, int>  ImageHash;

  BOOST_CONCEPT_ASSERT ((CImageContainer< ImageVector >));
  BOOST_CONCEPT_ASSERT ((CImageContainer< ImageMap >));
  BOOST_CONCEPT_ASSERT ((CImageContainer< ImageITK >));
  
  //BOOST_CONCEPT_ASSERT ((CImageContainer< ImageHash >));

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
  trace.beginBlock ( "Testing class CheckImageConcept" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testCheckImageConcept(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
