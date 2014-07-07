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
 * @file rigidtransformation2d.cpp
 * @ingroup Examples
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 * @date 2014/06/28
 *
 * An example file named rigidtransformation2d.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <cmath>
#include "DGtal/images/ImageSelector.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/ConstImageAdapter.h"
#include "ConfigExamples.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"
#include "DGtal/io/readers/PGMReader.h"
#include "DGtal/io/writers/GenericWriter.h"
#include "DGtal/images/RigidTransformation2D.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace functors;
using namespace Z2i;

///////////////////////////////////////////////////////////////////////////////

int main( int , char** )
{
  typedef ImageSelector<Domain, unsigned char >::Type Image;
  typedef ForwardRigidTransformation2D < Point, RealVector > ForwardTrans;
  typedef BackwardRigidTransformation2D < Point, RealVector > BackwardTrans;
  typedef ConstImageAdapter<Image, Domain, BackwardTrans, Image::Value, Identity > MyImageBackwardAdapter;
  typedef DomainRigidTransformation2D < Domain, ForwardTrans > MyTransformedDomain;
  typedef MyTransformedDomain::Bounds Bounds;
  
  trace.beginBlock ( "Example rigidtransformation2d" );

    ForwardTrans forwardTrans( Point ( 5, 5 ), M_PI_4, RealVector( 3, -3 ) );
    BackwardTrans backwardTrans( Point ( 5, 5 ), M_PI_4, RealVector( 3, -3 ) );
    MyTransformedDomain domainForwardTrans ( forwardTrans );
    Identity idD;
    
    Image image = PGMReader<Image>::importPGM ( examplesPath + "samples/church.pgm" ); 
    Bounds bounds = domainForwardTrans ( image.domain() );
    Domain transformedDomain ( bounds.first, bounds.second );
    
    trace.beginBlock ( "Backward - Eulerian model" );
      MyImageBackwardAdapter backwardImageAdapter ( image, transformedDomain , backwardTrans, idD );
      backwardImageAdapter >> "backward_transform.pgm";
    trace.endBlock();
    
    trace.beginBlock( "Forward - Lagrangian model" );
      Image forwardTransformedImage ( transformedDomain );
      for ( Domain::ConstIterator it = image.domain().begin(); it != image.domain().end(); ++it )
      {
	forwardTransformedImage.setValue ( forwardTrans ( *it ), image ( *it ) );
      }
      forwardTransformedImage >> "forward_transform.pgm";
    trace.endBlock();
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
