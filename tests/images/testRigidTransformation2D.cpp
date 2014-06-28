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
 * @file testtestRigidTransformation2D.cpp
 * @ingroup Tests
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 * @date 2014/06/26
 *
 * Functions for testing class testRigidTransformation2D.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <cmath>
#include <DGtal/images/ImageSelector.h>
#include <DGtal/images/ImageContainerBySTLVector.h>
#include "DGtal/images/ConstImageAdapter.h"
#include "DGtal/base/Common.h"
#include "DGtal/io/boards/Board2D.h"
#include "ConfigTest.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/images/RigidTransformation2D.h"
#include "DGtal/io/colormaps/GrayscaleColorMap.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z2i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class testRigidTransformation2D.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */

class testRigidTransformation2D
{
  typedef ImageSelector<Domain, int >::Type Image;
  typedef ForwardRigidTransformation2D < Point, RealVector > ForwardTrans;
  typedef BackwardRigidTransformation2D < Point, RealVector > BackwardTrans;
  typedef ConstImageAdapter<Image, Domain, ForwardTrans, Image::Value, DefaultFunctor > MyImageForwardAdapter;
  typedef ConstImageAdapter<Image, Domain, BackwardTrans, Image::Value, DefaultFunctor > MyImageBackwardAdapter;
private:
  Image binary;
  ForwardTrans forwardTrans;
  BackwardTrans backwardTrans;
  typedef GrayscaleColorMap<unsigned char> Gray;
  DefaultFunctor idD;
  Board2D aBoard;
  DomainRigidTransformation2D < Domain, ForwardTrans > domainTrans;
public:
  // Setup part
  testRigidTransformation2D() : 
  binary ( Domain ( Point ( 0,0 ), Point ( 10, 10 ) ) ),
  forwardTrans ( Point ( 5, 5 ), M_PI_2/2, RealVector() ),
    backwardTrans ( Point ( 5, 5 ), M_PI_2/2, RealVector() ),
    domainTrans ( forwardTrans )
    {
      binary.setValue ( Point ( 3,3 ), 255 );
      binary.setValue ( Point ( 3,4 ), 255 );
      binary.setValue ( Point ( 4,3 ), 255 );
      binary.setValue ( Point ( 4,4 ), 255 );
    }
    
    bool forwardTransformationBinary ()
    {
      Display2DFactory::drawImage<Gray>(aBoard, binary, (unsigned char)0, (unsigned char)255);
      aBoard.saveSVG( "forward_before.svg" );
      
      Image transformed ( domainTrans ( binary.domain() ) );
      for ( Domain::ConstIterator it = binary.domain().begin(); it != binary.domain().end(); ++it )
      {
	if ( binary(*it) == 255 )
	{
	  transformed.setValue ( forwardTrans ( *it ), 255 );
	}
	else
	{
	  transformed.setValue ( forwardTrans ( *it ), 0 );
	}
      }
      Display2DFactory::drawImage<Gray>(aBoard, transformed, (unsigned char)0, (unsigned char)255);
      aBoard.saveSVG( "forward_after.svg" );
      return true;
    }
    
    bool backwardTransformationBinary ()
    {
      MyImageBackwardAdapter adapter ( binary, domainTrans ( binary.domain() ), backwardTrans, idD );
      Display2DFactory::drawImage<Gray>(aBoard, binary, (unsigned char)0, (unsigned char)255);
      aBoard.saveSVG( "backward_before.svg" );
      Display2DFactory::drawImage<Gray>(aBoard, adapter, (unsigned char)0, (unsigned char)255);
      aBoard.saveSVG( "backward_after.svg" );
      return true;
    }
};

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int, char** )
{
  trace.beginBlock ( "Testing RigidTransformation2D" );
  bool res = true;
  testRigidTransformation2D testRigid;
  res &= testRigid.forwardTransformationBinary();
  res &= testRigid.backwardTransformationBinary();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
