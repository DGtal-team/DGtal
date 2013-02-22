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
 * @file exampleDigitalShapesDecorator.cpp
 * @ingroup Examples
 * @author Jérémy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2012/12/17
 *
 * An example file named exampleDigitalShapesDecorator.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"

// Shape construction
#include "DGtal/shapes/parametric/Ball2D.h"
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/topology/LightImplicitDigitalSurface.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/graph/DepthFirstVisitor.h"
#include "DGtal/graph/GraphVisitorRange.h"

#include "DGtal/shapes/DigitalShapesDecorator.h"

// Drawing
#include "DGtal/io/boards/Board2D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
    trace.beginBlock ( "Example DigitalShapesDecorator" );
    trace.info() << "Args:";
    for ( int i = 0; i < argc; ++i )
        trace.info() << " " << argv[ i ];
    trace.info() << endl;

    /// Construction of the shape + digitalization
    double h = 1.0;

    typedef Ball2D< Z2i::Space > ShapeA;
    ShapeA shapeA(0.0, 0.0, 5.0);
    ShapeA shapeB(1.0, 0.0, 5.0);

    /*typedef DigitalShapesMinus< ShapeA, ShapeA > Minus;
    Minus s_minus ( shapeA, shapeB );

    typedef ShapeA::RealPoint RealPoint;
    typedef GaussDigitizer< Z2i::Space, ShapeA > MyGaussDigitizer;
    MyGaussDigitizer digShape;
    digShape.attach( shapeA );
    digShape.init( shapeA.getLowerBound() + RealPoint(-1,-1), shapeA.getUpperBound(), h );

    Z2i::Domain domainShape = digShape.getDomain();
    Z2i::KSpace KSpaceShape;
    bool space_ok = KSpaceShape.init( domainShape.lowerBound(), domainShape.upperBound(), true );
    if ( !space_ok )
    {
        trace.error() << "Error in the Khamisky space construction." << std::endl;
        return 2;
    }

    typedef Z2i::KSpace::Surfel Surfel;
    typedef LightImplicitDigitalSurface< Z2i::KSpace, MyGaussDigitizer > LightImplicitDigSurface;
    typedef DigitalSurface< LightImplicitDigSurface > MyDigitalSurface;
    SurfelAdjacency< Z2i::KSpace::dimension > SAdj( true );
    Surfel bel = Surfaces< Z2i::KSpace >::findABel( KSpaceShape, digShape, 100000 );
    LightImplicitDigSurface LightImplDigSurf( KSpaceShape, digShape, SAdj, bel );
    MyDigitalSurface digSurf( LightImplDigSurf );

    typedef DepthFirstVisitor< MyDigitalSurface > Visitor;
    typedef GraphVisitorRange< Visitor > VisitorRange;
    typedef VisitorRange::ConstIterator SurfelConstIterator;

    VisitorRange range( new Visitor( digSurf, *digSurf.begin() ) );
    SurfelConstIterator abegin = range.begin();
    SurfelConstIterator aend = range.end();*/

    Board2D board;
    // export here
    board.saveSVG ( "example-DigitalShapesDecorator.svg" );

    trace.endBlock();
    return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
