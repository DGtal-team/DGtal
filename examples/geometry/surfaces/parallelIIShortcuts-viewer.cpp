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

// Visualization
#include "DGtal/io/viewers/PolyscopeViewer.h"
#include "DGtal/io/colormaps/GradientColorMap.h"

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
  std::string filename = examplesPath + std::string("/samples/bunny-128.vol");
  auto binary_image    = SH3::makeBinaryImage(filename, params );
  auto K               = SH3::getKSpace( binary_image, params );
  auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
  auto surfels   = SH3::getSurfelRange( surface, params );
  //! [Parallel-instantiation]

  //Parallel
  const auto axis = 2;
  const auto nbThreads = 4;
  trace.beginBlock(std::to_string(nbThreads) + "threads on axis "+std::to_string(axis));
  auto curv = SHG3::getIIMeanCurvatures( binary_image, surfels,
                                        params( "ii-thread-number", nbThreads )
                                        ( "ii-split-axis", axis )
                                        ( "r-radius", 8 ));
  trace.endBlock();

  PolyscopeViewer viewer;
  std::string objectName = "Surfels";
  viewer.draw(surfels, objectName); // Draws the object independently
  viewer.addQuantity(objectName, "Mean curvature", curv);

  AxisDomainSplitter<Z3i::Domain> splitter(axis);
  AxisDomainSplitter<Z3i::Domain>::SplitDomainsInfo splits = splitter(binary_image->domain(), nbThreads);
  HueShadeColorMap<unsigned int> cmap(0,(unsigned int)splits.size());
  for(auto i=0; i< splits.size(); ++i)
  {
    viewer << cmap(i);
    viewer << splits[i].domain;
  }
  viewer.show();

  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
