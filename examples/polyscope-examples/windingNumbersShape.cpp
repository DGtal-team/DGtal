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
#include <algorithm>
#include <fstream>      // std::ofstream

#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/point_cloud.h>

#include <DGtal/shapes/WindingNumbersShape.h>
#include <DGtal/shapes/GaussDigitizer.h>

#include "ConfigExamples.h"

using namespace DGtal;
using namespace Z3i;

// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
// The following typedefs are useful
typedef SurfaceMesh< RealPoint, RealVector >  SurfMesh;

int main()
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  params("surfaceComponents", "All")( "gridstep", 1. )("r-radius" , 4.0);
  
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
  
  
  trace.info()<<"Got "<<surfels.size()<<" surfels."<<std::endl;
  
  // Initialize polyscope
  polyscope::init();
  
  auto iinormals = SHG3::getIINormalVectors(binary_image, surfels, params);
  auto psMesh = polyscope::registerSurfaceMesh("input digital surface", positions, faces);
  psMesh->addFaceVectorQuantity("normals", iinormals);
  
  Eigen::MatrixXd points(surfels.size(),3);
  Eigen::MatrixXd normals(surfels.size(),3);
  std::ofstream ofs ("bunny.pts", std::ofstream::out);
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
    
    ofs<<p(0)<<" "<<p(1)<<" "<<p(2)<<" "<<n(0)<<" "<< n(1)<<" "<<n(2)<<std::endl;
  }
  ofs.close();
  auto pc= polyscope::registerPointCloud("input boundary points", points);
  pc->addVectorQuantity("normals", normals);
  
  //Winding number shape
  WindingNumbersShape<Z3i::Space> wnshape(points,normals);
 
  
  auto lower = binary_image->domain().lowerBound();
  auto upper = binary_image->domain().upperBound();
  
  auto resample_h = [&](double h){
    
    RegularPointEmbedder<Z3i::Space> pointEmbedder;
    pointEmbedder.init( h );
    Z3i::Point lowerPoint = pointEmbedder.floor( lower );
    Z3i::Point upperPoint = pointEmbedder.ceil( upper );
    Z3i::Domain domain(lowerPoint,upperPoint);
    trace.info() <<"Digital domain = "<<domain.size()<<" " <<domain<<std::endl;
    
    //Winding (batched)
    size_t size = domain.size();
    Eigen::MatrixXd queries(size,3);
    auto cpt=0;
    for(const auto &vox: domain)
    {
      Eigen::RowVector3<double> p(vox[0],vox[1],vox[2]);
      p *= h;
      queries.row(cpt) = p;
      ++cpt;
    }
    trace.info()<<"Cpt= "<<cpt<<" size= "<<size<<std::endl;
    auto orientations = wnshape.orientationBatch(queries);
    
    //Binary Predicate
    Z3i::DigitalSet voxels(domain);
    cpt=0;
    for(const auto &voxel: domain)
    {
      if (orientations[cpt]==INSIDE)
        voxels.insertNew(voxel);
      ++cpt;
    }
    trace.info() <<"Number of voxels = "<<voxels.size()<<std::endl;
    
    //Digital surface
    Z3i::KSpace kspace;
    kspace.init(lowerPoint, upperPoint, true);
    typedef KSpace::SurfelSet SurfelSet;
    typedef SetOfSurfels< KSpace, SurfelSet > MySetOfSurfels;
    typedef DigitalSurface< MySetOfSurfels > MyDigitalSurface;
    typedef SurfelAdjacency<KSpace::dimension> MySurfelAdjacency;
    
    MySurfelAdjacency surfAdj( true ); // interior in all directions.
    MySetOfSurfels theSetOfSurfels( kspace, surfAdj );
    Surfaces<KSpace>::sMakeBoundary(theSetOfSurfels.surfelSet(),
                                    kspace,
                                    voxels,
                                    lowerPoint,
                                    upperPoint);
    trace.info()<<"Surfel set size= "<<theSetOfSurfels.surfelSet().size()<<std::endl;
    
    //Polyscope visualization
    auto surfPtr = CountedPtr<DigitalSurface< MySetOfSurfels >>(new MyDigitalSurface(theSetOfSurfels));
    auto primalSurfaceReco   = SH3::makePrimalSurfaceMesh(surfPtr);
    
    std::vector<RealPoint> positionsReco = primalSurfaceReco->positions();
    //Fixing the embedding
    std::for_each(std::begin(positionsReco), std::end(positionsReco), [&](RealPoint &p){p=p*h;});
    
    std::vector<std::vector<SH3::SurfaceMesh::Vertex>> facesReco;
    for(auto face= 0 ; face < primalSurfaceReco->nbFaces(); ++face)
      facesReco.push_back(primalSurfaceReco->incidentVertices( face ));
    auto psMesh = polyscope::registerSurfaceMesh("Reconstruction "+std::to_string(h), positionsReco, facesReco);
  };
  
  resample_h(1.0);
  resample_h(2.0); //downscaling
  resample_h(0.5); //upscaling
#if defined(NDEBUG)  
  resample_h(0.2); //upscaling  
  resample_h(0.2); //upscaling
#else  
  trace.warning() << "CMake Debug mode detected, limiting upscaling to 0.5.";  
#endif  
  //resample_h(0.07); //extreme upscaling, 2M vertices
  
  // Set the callback function
  polyscope::show();
  return EXIT_SUCCESS;
}
