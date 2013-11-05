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
 * @file exampleIntegralInvariantCurvature3D.cpp
 * @ingroup Examples
 * @author Jérémy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2012/12/17
 *
 * An example file named exampleIntegralInvariantCurvature3D.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"

// Shape constructors
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/images/imagesSetsUtils/SetFromImage.h"
#include "DGtal/images/imagesSetsUtils/SimpleThresholdForegroundPredicate.h"
#include "DGtal/topology/SurfelAdjacency.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/topology/LightImplicitDigitalSurface.h"
#include "DGtal/images/ImageHelper.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/graph/DepthFirstVisitor.h"
#include "DGtal/graph/GraphVisitorRange.h"

// Integral Invariant includes
#include "DGtal/geometry/surfaces/FunctorOnCells.h"
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantGaussianCurvatureEstimator.h"

// Drawing
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include <QtGui/QApplication>

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
    if ( argc != 4 )
    {
        trace.error() << "Usage: " << argv[0]
                               << " <fileName.vol> <threshold> <re_convolution_kernel>" << std::endl;
        trace.error() << "Example : "<< argv[0] << " Al.150.vol 0 7" << std::endl;
        return 0;
    }

    trace.beginBlock ( "Example IntegralInvariantCurvature3D" );
    trace.info() << "Args:";
    for ( int i = 0; i < argc; ++i )
        trace.info() << " " << argv[ i ];
    trace.info() << endl;

    double h = 1.0;
    unsigned int threshold = std::atoi( argv[ 2 ] );

    /// Construction of the shape from vol file
    typedef ImageSelector< Z3i::Domain, unsigned int >::Type Image;
    typedef SimpleThresholdForegroundPredicate< Image > ImagePredicate;
    typedef Z3i::KSpace::Surfel Surfel;
    typedef LightImplicitDigitalSurface< Z3i::KSpace, ImagePredicate > MyLightImplicitDigitalSurface;
    typedef DigitalSurface< MyLightImplicitDigitalSurface > MyDigitalSurface;

    std::string filename = argv[1];
    Image image = VolReader<Image>::importVol( filename );
    ImagePredicate predicate = ImagePredicate( image, threshold );

    Z3i::Domain domain = image.domain();

    Z3i::KSpace KSpaceShape;

    bool space_ok = KSpaceShape.init( domain.lowerBound(), domain.upperBound(), true );
    if (!space_ok)
    {
      trace.error() << "Error in the Khalimsky space construction."<<std::endl;
      return 2;
    }

    SurfelAdjacency< Z3i::KSpace::dimension > SAdj( true );
    Surfel bel = Surfaces< Z3i::KSpace >::findABel( KSpaceShape, predicate, 100000 );
    MyLightImplicitDigitalSurface LightImplDigSurf( KSpaceShape, predicate, SAdj, bel );
    MyDigitalSurface digSurf( LightImplDigSurf );

    typedef DepthFirstVisitor< MyDigitalSurface > Visitor;
    typedef GraphVisitorRange< Visitor > VisitorRange;
    typedef VisitorRange::ConstIterator SurfelConstIterator;

    VisitorRange range( new Visitor( digSurf, *digSurf.begin() ) );
    SurfelConstIterator abegin = range.begin();
    SurfelConstIterator aend = range.end();

    typedef ImageToConstantFunctor< Image, ImagePredicate > MyPointFunctor;
    MyPointFunctor pointFunctor( &image, &predicate, 1 );

    /// Integral Invariant stuff
    //! [IntegralInvariantUsage]
    double re_convolution_kernel = std::atof(argv[3]);

    typedef FunctorOnCells< MyPointFunctor, Z3i::KSpace > MyCellFunctor;
    typedef IntegralInvariantGaussianCurvatureEstimator< Z3i::KSpace, MyCellFunctor > MyCurvatureEstimator; // Gaussian curvature estimator

    MyCellFunctor functor ( pointFunctor, KSpaceShape ); // Creation of a functor on Cells, returning true if the cell is inside the shape
    MyCurvatureEstimator estimator ( KSpaceShape, functor );
    estimator.init( h, re_convolution_kernel ); // Initialisation for a given Euclidean radius of convolution kernel
    std::vector< double > results;
    back_insert_iterator< std::vector< double > > resultsIterator( results ); // output iterator for results of Integral Invariante curvature computation
    estimator.eval ( abegin, aend, resultsIterator ); // Computation
    //! [IntegralInvariantUsage]

    /// Drawing results
    typedef MyCurvatureEstimator::Quantity Quantity;
    Quantity min = numeric_limits < Quantity >::max();
    Quantity max = numeric_limits < Quantity >::min();
    for ( unsigned int i = 0; i < results.size(); ++i )
    {
        if ( results[ i ] < min )
        {
            min = results[ i ];
        }
        else if ( results[ i ] > max )
        {
            max = results[ i ];
        }
    }

    QApplication application( argc, argv );
    typedef Viewer3D<Z3i::Space, Z3i::KSpace> Viewer;
    Viewer viewer( KSpaceShape );
    viewer.setWindowTitle("example Integral Invariant 3D");
    viewer.show();

    typedef GradientColorMap< Quantity > Gradient;
    Gradient cmap_grad( min, max );
    cmap_grad.addColor( Color( 50, 50, 255 ) );
    cmap_grad.addColor( Color( 255, 0, 0 ) );
    cmap_grad.addColor( Color( 255, 255, 10 ) );

    VisitorRange range2( new Visitor( digSurf, *digSurf.begin() ) );
    abegin = range2.begin();

    for ( unsigned int i = 0; i < results.size(); ++i )
    {
        viewer << CustomColors3D( Color::Black, cmap_grad( results[ i ] ))
               << *abegin;
        ++abegin;
    }

    viewer << Viewer3D<>::updateDisplay;

    trace.endBlock();
    return application.exec();
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
