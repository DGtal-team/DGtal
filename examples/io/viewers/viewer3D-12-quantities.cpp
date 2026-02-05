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
 * @file io/viewers/viewer3D-12-quantities.cpp
 * @ingroup Examples
 * @author Batien Doignies
 * LIRIS, Université Claude Bernard Lyon 1
 *
 * @date 2025/06/02
 *
 * Simple example of class PolyscopeViewer.
 *
 * This file is part of the DGtal library.
 */

/**
 * Example of adding quantities to drawn element with PolyscopeViewer
 */
#include "ConfigExamples.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"

#include "DGtal/io/viewers/PolyscopeViewer.h"

using namespace std;
using namespace DGtal;
using namespace Z3i;

int main() {
    typedef Shortcuts<KSpace> SH3;
    typedef ShortcutsGeometry<KSpace> SHG3;

    auto params = SH3::defaultParameters() | SHG3::defaultParameters();

    params( "polynomial", "3*x^2+2*y^2+z^2-90" )( "gridstep", 0.25 );
    auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
    auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
    auto binary_image    = SH3::makeBinaryImage      ( digitized_shape, params );
    auto K               = SH3::getKSpace( params );
    auto surface         = SH3::makeLightDigitalSurface( binary_image, K, params );
    auto surfels         = SH3::getSurfelRange( surface, params );
    auto positions       = SHG3::getPositions( implicit_shape, K, surfels, params );
    auto normals         = SHG3::getNormalVectors( implicit_shape, K, surfels, params );
    auto mean_curvs      = SHG3::getMeanCurvatures( implicit_shape, K, surfels, params );
    auto gauss_curvs     = SHG3::getGaussianCurvatures( implicit_shape, K, surfels, params );

    PolyscopeViewer viewer;
    //! [ExampleViewer3DQuantitiesStream]
    // Option 1: stream operator (Scell_1_2d in viewer)
    //   Most usefull when adding single quantity, otherwise
    //   it should be nested.
    viewer << WithQuantity(
        WithQuantity(
          WithQuantity(
            surfels, "Mean Curv", mean_curvs
            ),
          "Gauss Curve", gauss_curvs
          ),
        "Normal", normals
        );
    //! [ExampleViewer3DQuantitiesStream]

    //! [ExampleViewer3DQuantitiesAddQuantity]
    // Option 2: draw first then add quantities (Surfels 1 in viewer)
    //  This requires to obtain or set object name
    std::string objectName = "Surfels 1";
    viewer.draw(surfels, objectName); // Draws the object independantly
    viewer.addQuantity(objectName, "Mean Curv", mean_curvs);
    viewer.addQuantity(objectName, "Gauss Curv", gauss_curvs);
    viewer.addQuantity(objectName, "Normal", normals);
    //! [ExampleViewer3DQuantitiesAddQuantity]

    // Option 3: Within loops (Surfels 3 within the viewer)
    //  Here, a group must be created for the colormaps to work
    //  Note: here Option 1 and 2 are superior, this is just an example

    //! [ExampleViewer3DQuantitiesLoop]
    std::string objectName2 = "Surfels 3";
    viewer.newVolumetricList(objectName2); // Signed cells are drawn as volumetric meshes
    viewer.allowReuseList = true; // Allows for automatic groupping

    auto surfIt = surfels.begin();
    auto mcurveIt = mean_curvs.begin();
    auto gcurveIt = gauss_curvs.begin();
    auto ncurveIt = normals.begin();

    for (; surfIt != surfels.end(); ++surfIt, ++mcurveIt, ++gcurveIt, ++ncurveIt) {
      // Both options 1 and 2 can be used together, as long as we know the name for sure
      viewer << WithQuantity(*surfIt, "Mean Curv", *mcurveIt);
      viewer.addQuantity(objectName2, "Gauss Curve", *gcurveIt);
      viewer.addQuantity(objectName2, "Normals", *ncurveIt);
    }
    //! [ExampleViewer3DQuantitiesLoop]

    viewer.show();
    return 0;
 }
