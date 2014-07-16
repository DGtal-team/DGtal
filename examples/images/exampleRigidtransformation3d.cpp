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
 * @file rigidtransformation3d.cpp
 * @ingroup Examples
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 * @date 2014/06/28
 *
 * An example file named rigidtransformation3d.
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
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/io/writers/GenericWriter.h"
#include "DGtal/images/RigidTransformation3D.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace functors;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////

int main( int , char** )
{
  typedef ImageSelector<Domain, unsigned char >::Type Image;

  typedef ForwardRigidTransformation3D < Space > ForwardTrans;
  typedef BackwardRigidTransformation3D < Space > BackwardTrans;
  typedef ConstImageAdapter<Image, Domain, BackwardTrans, Image::Value, Identity > MyImageBackwardAdapter;
  typedef DomainRigidTransformation3D < Domain, ForwardTrans > MyTransformedDomain;
  typedef MyTransformedDomain::Bounds Bounds;
  
  trace.beginBlock ( "Example rigidtransformation3d" );
  
    ForwardTrans forwardTrans( Point ( 5, 5, 5 ), RealVector ( 1, 0, 1 ), M_PI_4, RealVector( 3, -3, 3 ) );
    BackwardTrans backwardTrans( Point ( 5, 5, 5 ), RealVector ( 1, 0, 1 ), M_PI_4, RealVector( 3, -3, 3 ) );
    MyTransformedDomain domainForwardTrans ( forwardTrans );
    Identity idD;
    
    Image image = VolReader<Image>::importVol ( examplesPath + "samples/cat10.vol" );
    Bounds bounds = domainForwardTrans ( image.domain() );
    Domain transformedDomain ( bounds.first, bounds.second );
  
    trace.beginBlock ( "Backward - Eulerian model" );
      MyImageBackwardAdapter adapter ( image, transformedDomain, backwardTrans, idD );
      adapter >> "backward_transform.pgm3d";
    trace.endBlock();
  
    trace.beginBlock( "Forward - Lagrangian model" );
      Image transformed ( transformedDomain );
      for ( Domain::ConstIterator it = image.domain().begin(); it != image.domain().end(); ++it )
      {
	transformed.setValue ( forwardTrans ( *it ), image ( *it ) );
      }
      transformed >> "forward_transform.pgm3d";
    trace.endBlock();
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
