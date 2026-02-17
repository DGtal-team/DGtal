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
#include <algorithm>
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
  params( "polynomial", "goursat" )( "gridstep", 1. )( "surfaceTraversal", "BreadthFirst" );
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

  std::vector<double> distbf, distdf;
  for(auto i = 1 ; i < surfels.size(); ++i)
  {
    distbf.push_back( ( embedder(surfels[i]) - embedder(surfels[i-1])).norm());
    distdf.push_back( ( embedder(surfelsdepth[i]) - embedder(surfelsdepth[i-1])).norm());
  }

  auto ps=  polyscope::registerCurveNetworkLine("BF", bfnodes);
  ps->addNodeScalarQuantity("id", id);
  ps->addEdgeScalarQuantity("dist", distbf);
  auto ps2=polyscope::registerCurveNetworkLine("DF", dfnodes);
  ps2->addNodeScalarQuantity("id", id);
  ps2->addEdgeScalarQuantity("dist", distdf);

  trace.info()<<"Counting the number of consecutive pairs with distance <=  1.0"<< std::endl;
  trace.info()<<"BF strategy = "<< std::count_if(distbf.begin(), distbf.end(), [](double a){ return a-0.000001 <= 1.0;})<< std::endl;
  trace.info()<<"DF strategy = "<< std::count_if(distdf.begin(), distdf.end(), [](double a){ return a-0.000001 <= 1.0;})<< std::endl;

  trace.info()<<"Counting the number of consecutive pairs with distance <=  sqrt{2}/2"<< std::endl;
  trace.info()<<"BF strategy = "<< std::count_if(distbf.begin(), distbf.end(), [](double a){ return a-0.000001 <= sqrt(2.0)/2.0;})<< std::endl;
  trace.info()<<"DF strategy = "<< std::count_if(distdf.begin(), distdf.end(), [](double a){ return a-0.000001 <= sqrt(2.0)/2.0;})<< std::endl;


  viewer.show();
}

/** @ingroup Tests **/
