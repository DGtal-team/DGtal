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
 * @file exampleEuclideanShapesDecorator.cpp
 * @ingroup Examples
 * @author Jérémy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2012/12/17
 *
 * An example file named exampleEuclideanShapesDecorator.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"

// Shape construction
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/topology/LightImplicitDigitalSurface.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/images/ImageHelper.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/shapes/implicit/ImplicitBall.h"
#include "DGtal/shapes/EuclideanShapesDecorator.h"

// Drawing
#include "DGtal/io/boards/Board2D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
    trace.beginBlock ( "Example EuclideanShapesDecorator" );
    trace.info() << "Args:";
    for ( int i = 0; i < argc; ++i )
        trace.info() << " " << argv[ i ];
    trace.info() << std::endl;

    //! [EuclideanShapesDecoratorUsageFull1]
    /// Construction of the shape + digitalization
    double h = 1.0;

    //! [EuclideanShapesDecoratorUsage]
    //! [EuclideanShapesDecoratorUsageFull1]
    typedef ImplicitBall< Z2i::Space > MyEuclideanShapeA;
    typedef ImplicitBall< Z2i::Space > MyEuclideanShapeB;
    MyEuclideanShapeA shapeA( Z2i::RealPoint( 0.0, 0.0 ), 14 );
    MyEuclideanShapeB shapeB( Z2i::RealPoint( 1.0, 0.0 ), 14 );

    typedef EuclideanShapesMinus< MyEuclideanShapeA, MyEuclideanShapeB > Minus;
    Minus s_minus ( shapeA, shapeB );
    //! [EuclideanShapesDecoratorUsage]

    typedef Z2i::KSpace::Surfel Surfel;

    typedef GaussDigitizer< Z2i::Space, MyEuclideanShapeA > MyGaussDigitizerShapeA;
    typedef LightImplicitDigitalSurface< Z2i::KSpace, MyGaussDigitizerShapeA > LightImplDigSurfaceA;
    typedef DigitalSurface< LightImplDigSurfaceA > MyDigitalSurfaceA;
    MyGaussDigitizerShapeA digShapeA;
    digShapeA.attach( shapeA );
    digShapeA.init( shapeA.getLowerBound() + Z2i::RealPoint( -1.0, -1.0 ),
                    shapeA.getUpperBound() + Z2i::RealPoint( 1.0, 1.0 ),
                    h );
    Z2i::Domain domainShapeA = digShapeA.getDomain();

    Z2i::DigitalSet aSetA( domainShapeA );
    Shapes<Z2i::Domain>::digitalShaper( aSetA, digShapeA );



    typedef GaussDigitizer< Z2i::Space, MyEuclideanShapeB > MyGaussDigitizerShapeB;
    typedef LightImplicitDigitalSurface< Z2i::KSpace, MyGaussDigitizerShapeB > LightImplDigSurfaceB;
    typedef DigitalSurface< LightImplDigSurfaceB > MyDigitalSurfaceB;
    MyGaussDigitizerShapeB digShapeB;
    digShapeB.attach( shapeB );
    digShapeB.init( shapeB.getLowerBound() + Z2i::RealPoint( -1.0, -1.0 ),
                    shapeB.getUpperBound() + Z2i::RealPoint( 1.0, 1.0 ),
                    h );
    Z2i::Domain domainShapeB = digShapeB.getDomain();

    Z2i::DigitalSet aSetB( domainShapeB );
    Shapes<Z2i::Domain>::digitalShaper( aSetB, digShapeB );



    //! [EuclideanShapesDecoratorUsageFull2]
    typedef GaussDigitizer< Z2i::Space, Minus > MyGaussDigitizer;
    typedef LightImplicitDigitalSurface< Z2i::KSpace, MyGaussDigitizer > LightImplicitDigSurface;
    typedef DigitalSurface< LightImplicitDigSurface > MyDigitalSurface;
    MyGaussDigitizer digShape;
    digShape.attach( s_minus );
    digShape.init( s_minus.getLowerBound() + Z2i::RealPoint( -1.0, -1.0 ),
                   s_minus.getUpperBound() + Z2i::RealPoint( 1.0, 1.0 ),
                   h );
    Z2i::Domain domainShape = digShape.getDomain();
    Z2i::DigitalSet aSet( domainShape );
    Shapes<Z2i::Domain>::digitalShaper( aSet, digShape );
    //! [EuclideanShapesDecoratorUsageFull2]


    Board2D board;
    board << SetMode( domainShape.className(), "Paving" )
          << domainShape;

    Color dblack  ( 0,    0,    0,  50  );
    Color dgreen  ( 0,    192,  0,  50  );
    Color dred    ( 192,  0,    0,  50  );
    //! [EuclideanShapesDecoratorUsageFull3]
    Color dorange ( 255,  136,  0,  220 );
    //! [EuclideanShapesDecoratorUsageFull3]

    board << CustomStyle( aSetA.className(), new CustomFillColor( dgreen ));
    board << aSetA;

    board << CustomStyle( aSetB.className(), new CustomFillColor( dred ));
    board << aSetB;

    //! [EuclideanShapesDecoratorUsageFull4]
    board << CustomStyle( aSet.className(), new CustomFillColor( dorange ));
    board << aSet;
    //! [EuclideanShapesDecoratorUsageFull4]

    board << CustomStyle( aSetA.className(), new CustomFillColor( dblack ) );
    board << Z2i::Domain::Point( 0.0, 0.0 );

    board.saveSVG ( "example-EuclideanShapesDecorator.svg" );

    trace.endBlock();
    return 0;
}

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
