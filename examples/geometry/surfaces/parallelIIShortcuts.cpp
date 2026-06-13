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
 * @file
 * @ingroup Examples
 * @author David Coeurjolly (david.coeurjolly@cnrs.fr)
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 *
 * @date 2026/06/08
 *
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "ConfigExamples.h"

// Helpers
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"


#ifdef DGTAL_WITH_POLYSCOPE_VIEWER
// Visualization
#include "DGtal/io/viewers/PolyscopeViewer.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#endif

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;

// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace> SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
///////////////////////////////////////////////////////////////////////////////

int main()
{

    //! [Parallel-instantiation]
    auto params = SH3::defaultParameters() | SHG3::defaultParameters();
    params( "polynomial", "goursat" )( "gridstep", 0.125 );
    auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
    auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
    auto K         = SH3::getKSpace( params );
    auto surface   = SH3::makeDigitalSurface( digitized_shape, K, params );
    auto surfels   = SH3::getSurfelRange( surface, params );
    //! [Parallel-instantiation]

    //! [Parallel-run]
    trace.info()<< "Input vol domain: "<< digitized_shape->getDomain() << std::endl;
    //Sequential
    trace.beginBlock("Single thread");
    auto curv      = SHG3::getIIMeanCurvatures( digitized_shape, surfels, params );
    trace.endBlock();

    //Parallel
    trace.beginBlock("4 threads on default axis");
    auto curv_par4  = SHG3::getIIMeanCurvatures( digitized_shape, surfels,
                                                params( "ii-thread-number", 4 ));
    trace.endBlock();

    //Parallel
    trace.beginBlock("8 threads on axis 0");
    auto curv_par8_0 = SHG3::getIIMeanCurvatures( digitized_shape, surfels,
                                                params( "ii-thread-number", 8 )
                                                ( "ii-split-axis", 0 ) );
    trace.endBlock();

    //Parallel
    trace.beginBlock("8 threads on axis 1");
    auto curv_par8_1  = SHG3::getIIMeanCurvatures( digitized_shape, surfels,
                                                params( "ii-thread-number", 8 )
                                                ( "ii-split-axis", 1 ) );
    trace.endBlock();

    //Parallel
    trace.beginBlock("8 threads on axis 2");
    auto curv_par8_2  = SHG3::getIIMeanCurvatures( digitized_shape, surfels,
                                                params( "ii-thread-number", 8 )
                                                ( "ii-split-axis", 2 ) );
    trace.endBlock();

    //Parallel
    trace.beginBlock("16 threads on axis 2");
    auto curv_par16  = SHG3::getIIMeanCurvatures( digitized_shape, surfels,
                                                 params( "ii-thread-number", 16 )
                                                 ( "ii-split-axis", 2 ) );
    trace.endBlock();
    //! [Parallel-run]

#ifdef DGTAL_WITH_POLYSCOPE_VIEWER
    PolyscopeViewer viewer;

    std::string objectName = "Surfels";
    viewer.draw(surfels, objectName); // Draws the object independently
    viewer.addQuantity(objectName, "Mean curvature", curv_par8_0);

    AxisDomainSplitter<Z3i::Domain> splitter(0);
    AxisDomainSplitter<Z3i::Domain>::SplitDomainsInfo splits = splitter(digitized_shape->getDomain(), 8);
    HueShadeColorMap<unsigned int> cmap(0,(unsigned int)splits.size());
    for(auto i=0; i< splits.size(); ++i)
    {
        viewer << cmap(i);
        viewer << splits[i].domain;
    }
    viewer.show();
#endif

    return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
