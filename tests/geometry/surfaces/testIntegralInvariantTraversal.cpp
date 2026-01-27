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
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2020/03/14
 *
 * Functions for testing class IntegralInvariantShortcuts.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>

#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"
#include "DGtal/io/viewers/PolyscopeViewer.h"
#include "polyscope/curve_network.h"

///////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace DGtal;
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class IntegralInvariantShortcuts.
///////////////////////////////////////////////////////////////////////////////

int main()
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  params( "polynomial", "goursat" )( "gridstep", 2. )( "surfaceTraversal", "BreadthFirst" );
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto binary_image    = SH3::makeBinaryImage( digitized_shape, params );
  auto K               = SH3::getKSpace( params );
  auto embedder        = SH3::getSCellEmbedder(K);
  
  trace.info()<<"Binary image "<< *binary_image<< std::endl;

  auto surface         = SH3::makeLightDigitalSurface( binary_image, K, params );
  auto surfels         = SH3::getSurfelRange( surface, params );
  trace.info() << "Nb surfels= " << surfels.size() << std::endl;

  //Computing some differential quantities
  params("r-radius", 3.0);
  
  trace.beginBlock("II SHG3 (BreadthFirst)");
  auto normals   = SHG3::getIINormalVectors(binary_image,surfels,params);
  trace.endBlock();
  
  //Duplicating params and surfel range
  auto paramsdepth= SH3::defaultParameters()  | SHG3::defaultParameters();
  paramsdepth( "surfaceTraversal", "DepthFirst" )("r-radius",3.0);
  auto surfelsdepth = SH3::getSurfelRange( surface, paramsdepth  );
  
  trace.beginBlock("II (DepthFirst)");
  auto normalsdepth  = SHG3::getIINormalVectors(binary_image,surfelsdepth,paramsdepth);
  trace.endBlock();
  
  std::vector<unsigned int> id(surfels.size());
  auto cpt=0;
  for(auto &v: id)
    v=cpt++;
  
  PolyscopeViewer viewer;
  std::string objectName = "Surfels BF";
  viewer.draw(surfels, objectName);
  viewer.addQuantity(objectName, "Normals (BF)", normals);
  viewer.addQuantity(objectName, "Id (BF)", id);

  std::string objectName2 = "Surfels DF";
  viewer.draw(surfelsdepth, objectName2);
  viewer.addQuantity(objectName2, "Normals (DF)", normalsdepth);
  viewer.addQuantity(objectName2, "Id (BF)", id);

  //Nodes
  std::vector<Z3i::RealPoint> bfnodes;
  for(auto s: surfels)
    bfnodes.push_back(embedder(s));
  std::vector<Z3i::RealPoint> dfnodes;
  for(auto s: surfelsdepth)
    dfnodes.push_back(embedder(s));
  
  polyscope::registerCurveNetworkLine("BF", bfnodes)-> addNodeScalarQuantity("id", id);
  polyscope::registerCurveNetworkLine("DF", dfnodes)-> addNodeScalarQuantity("id", id);
  
  
  viewer.show();
}

/** @ingroup Tests **/
