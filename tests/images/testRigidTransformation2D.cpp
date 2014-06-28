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
#include "ConfigTest.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/images/RigidTransformation2D.h"
#include "DGtal/io/readers/PGMReader.h"
#include "DGtal/io/writers/GenericWriter.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace functors;
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
  typedef ImageSelector<Domain, unsigned char >::Type Image;
  typedef ForwardRigidTransformation2D < Point, RealVector > ForwardTrans;
  typedef BackwardRigidTransformation2D < Point, RealVector > BackwardTrans;
  typedef ConstImageAdapter<Image, Domain, BackwardTrans, Image::Value, DefaultFunctor > MyImageBackwardAdapter;
private:
  Image binary;
  Image gray;
  ForwardTrans forwardTrans;
  BackwardTrans backwardTrans;
  DefaultFunctor idD;
  DomainRigidTransformation2D < Domain, ForwardTrans > domainForwardTrans;
public:
  // Setup part
  testRigidTransformation2D() : 
  binary ( Domain ( Point ( 0,0 ), Point ( 10, 10 ) ) ),
  gray ( PGMReader<Image>::importPGM ( testPath + "samples/church-small.pgm" ) ),
  forwardTrans ( Point ( 5, 5 ), M_PI_4, RealVector( 3, -3 ) ),
  backwardTrans ( Point ( 5, 5 ), M_PI_4, RealVector( 3, -3 ) ),
  domainForwardTrans ( forwardTrans )
    {
      binary.setValue ( Point ( 3,3 ), 255 );
      binary.setValue ( Point ( 3,4 ), 255 );
      binary.setValue ( Point ( 4,3 ), 255 );
      binary.setValue ( Point ( 4,4 ), 255 );
      
      binary >> "binary.pgm";
      gray >> "gray.pgm";
    }
    
    bool forwardTransformationBinary ()
    {
      Image transformed ( domainForwardTrans ( binary.domain() ) );
      for ( Domain::ConstIterator it = binary.domain().begin(); it != binary.domain().end(); ++it )
      {
	transformed.setValue ( forwardTrans ( *it ), binary ( *it ) );
      }
      transformed >> "binary_after_forward.pgm";
      return true;
    }
    
    bool backwardTransformationBinary ()
    {
      MyImageBackwardAdapter adapter ( binary, domainForwardTrans ( binary.domain() ), backwardTrans, idD );
      adapter >> "binary_after_backward.pgm";
      return true;
    }
    
    bool backwardTransformationGray ()
    {
      MyImageBackwardAdapter adapter ( gray, domainForwardTrans ( gray.domain() ), backwardTrans, idD );
      adapter >> "gray_after_backward.pgm";
      return true;
    }
    
    bool forwardTransformationGray ()
    {
      Image transformed ( domainForwardTrans ( gray.domain() ) );
      for ( Domain::ConstIterator it = gray.domain().begin(); it != gray.domain().end(); ++it )
      {
	transformed.setValue ( forwardTrans ( *it ), gray ( *it ) );
      }
      transformed >> "gray_after_forward.pgm";
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
  res &= testRigid.forwardTransformationGray();
  res &= testRigid.backwardTransformationGray();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
