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
 * @file images/exampleAffinetransformation2d.cpp
 * @ingroup Examples
 * @author Phuc Ngo (\c hoai-diem-phuc.ngo@loria.fr )
 * Laboratoire Lorrain de Recherche en Informatique et ses Applications (LORIA), France
 *
 * @date 12/05/2021
 *
 * An example file named affinetransformation2d.
 *
 * This file is part of the DGtal library.
 */

/**
*  Example of 2D affine transformation using forward and backward models.
   @see @ref moduleGeometricTransform
   \image html church_AffineBackward.png "Result for backward model" 
*  \example images/exampleAffinetransformation2d.cpp
**/

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
//! [include]
#include "DGtal/images/AffineTransformation2D.h"
//! [include]
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace functors;
using namespace Z2i;

int main( int , char** )
{
    typedef ImageSelector<Domain, unsigned char >::Type Image;
    //! [def]
    typedef ForwardAffineTransformation2D < Space > ForwardTrans;
    typedef BackwardAffineTransformation2D < Space > BackwardTrans;
    typedef DomainGeometricTransformation2D < Domain, ForwardTrans > MyDomainTransformer;
    typedef MyDomainTransformer::Bounds Bounds;
    //! [def]
    trace.beginBlock ( "Example AffineTransformation2d" );
    //! [trans]
    ForwardTrans forwardTrans(1.2, -1, 1.6, 2, RealPoint ( 5, 5 ));
    BackwardTrans backwardTrans(1.2, -1, 1.6, 2, RealPoint ( 5, 5 ));
    //! [trans]
    //![init_domain_helper]
    MyDomainTransformer domainTransformer ( forwardTrans );
    
    Image image = PGMReader<Image>::importPGM ( "../church-small.pgm" );
    //! [domain]
    Bounds bounds = domainTransformer ( image.domain() );
    Domain transformedDomain ( bounds.first, bounds.second );
    //! [domain]
    
    trace.beginBlock ( "Backward - Eulerian model" );
    //! [backward]
    //![init_domain_helper]
    Image backwardTransformedImage ( transformedDomain );
    for ( Domain::ConstIterator it = backwardTransformedImage.domain().begin(); it != backwardTransformedImage.domain().end(); ++it )
    {
        Point p = backwardTrans ( *it );
        if(image.domain().isInside(p))
            backwardTransformedImage.setValue ( *it, image(backwardTrans ( *it )) );
    }
    backwardTransformedImage >> "backward_transform.pgm";
    //! [backward]
    trace.endBlock();
    
    trace.beginBlock( "Forward - Lagrangian model" );
    Image forwardTransformedImage ( transformedDomain );
    //! [forward]
    for ( Domain::ConstIterator it = image.domain().begin(); it != image.domain().end(); ++it )
    {
        forwardTransformedImage.setValue ( forwardTrans ( *it ),  image (*it)) ;
    }
    forwardTransformedImage >> "forward_transform.pgm";
    //! [forward]
    trace.endBlock();
    trace.endBlock();
    return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
