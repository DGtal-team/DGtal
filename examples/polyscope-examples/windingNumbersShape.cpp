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
 */
/**
 * @file
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2023/06/15
 *
 * This file is part of the DGtal library.
 */
#include <iostream>
#include <string>
#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/point_cloud.h>

#include <DGtal/shapes/WindingNumbersShape.h>

#include "ConfigExamples.h"

using namespace DGtal;
using namespace Z3i;

// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
// The following typedefs are useful
typedef SurfaceMesh< RealPoint, RealVector >  SurfMesh;


void myCallback()
{
  
}

int main()
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  params("surfaceComponents", "All")( "gridstep", 1. )("r-radius" , 2.0);

  std::string filename = examplesPath + std::string("/samples/bunny-32.vol");
  auto binary_image    = SH3::makeBinaryImage(filename, params );
  auto K               = SH3::getKSpace( binary_image, params );
  auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
  SH3::Cell2Index c2i;
  auto primalSurface   = SH3::makePrimalSurfaceMesh(surface);
  auto surfels         = SH3::getSurfelRange( surface, params);
  auto embedder        = SH3::getSCellEmbedder( K );


  //Need to convert the faces
  std::vector<std::vector<SH3::SurfaceMesh::Vertex>> faces;
  std::vector<RealPoint> positions;
  
  for(auto face= 0 ; face < primalSurface->nbFaces(); ++face)
    faces.push_back(primalSurface->incidentVertices( face ));
  
  //Recasting to vector of vertices
  positions = primalSurface->positions();
   
  auto surfmesh = SurfMesh(positions.begin(),
                      positions.end(),
                      faces.begin(),
                      faces.end());

  // Initialize polyscope
  polyscope::init();
  
  auto iinormals = SHG3::getIINormalVectors(binary_image, surfels, params);
  auto psMesh = polyscope::registerSurfaceMesh("input digital surface", positions, faces);
  psMesh->addFaceVectorQuantity("normals", iinormals);
  
  //
  Eigen::MatrixXd points(surfels.size(),3);
  Eigen::MatrixXd normals(surfels.size(),3);
  
  for(auto i=0; i< surfels.size(); ++i)
  {
    auto p = embedder(surfels[i]);
    auto n = iinormals[i];
    points(i,0) = p(0);
    points(i,1) = p(1);
    points(i,2) = p(2);
    normals(i,0) = n(0);
    normals(i,1) = n(1);
    normals(i,2) = n(2);
    
  }
  auto pc= polyscope::registerPointCloud("input boundary points", points);
  pc->addVectorQuantity("normals", normals);
  WindingNumbersShape<Z3i::Space> wnshape(points,normals);
  
  auto lower = binary_image->domain().lowerBound();
  auto upper = binary_image->domain().upperBound();
  auto extend = (upper-lower);

  {
    double step = 1.0;
    size_t size = (size_t)std::floor(extend[0]*step * extend[1]*step * extend[2] * step);
    Eigen::MatrixXd queries(size,3);
    auto cpt=0;
    for(double x= lower[0] ; x < upper[0] ; x+= step)
      for(double y= lower[1] ; y < upper[1] ; y+= step)
        for(double z= lower[2] ; z < upper[2] ; z+= step)
        {
          Eigen::RowVector3<double> p(x,y,z);
          queries.row(cpt) = p;
          ++cpt;
        }
    trace.info()<<"Cpt= "<<cpt<<" size= "<<size<<std::endl;
    auto orientations = wnshape.orientationBatch(queries);
    polyscope::registerPointCloud("probes",queries)->addScalarQuantity("orientation",orientations);
  }
  
  {
    double step = 2;
    size_t size = (size_t)std::floor(extend[0]*step * extend[1]*step * extend[2] * step);
    Eigen::MatrixXd queries(size,3);
    auto cpt=0;
    for(double x= lower[0] ; x < upper[0] ; x+= step)
      for(double y= lower[1] ; y < upper[1] ; y+= step)
        for(double z= lower[2] ; z < upper[2] ; z+= step)
        {
          Eigen::RowVector3<double> p(x,y,z);
          queries.row(cpt) = p;
          ++cpt;
        }
    trace.info()<<"Cpt= "<<cpt<<" size= "<<size<<std::endl;
    auto orientations = wnshape.orientationBatch(queries);
    polyscope::registerPointCloud("probeslow",queries)->addScalarQuantity("orientation",orientations);
  }
  
  // Set the callback function
  polyscope::state::userCallback = myCallback;
  polyscope::show();
  return EXIT_SUCCESS;
}
