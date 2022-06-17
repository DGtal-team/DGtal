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
 * @file dec/VectorHeatMethod.cpp
 * @author Baptiste GENEST (\c baptistegenest@gmail.com )
 * intership at Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2022/06/15
 *
 * This file is part of the DGtal library.
 */

//////// INCLUDES ////////

#include <iostream>

#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>
#include <DGtal/shapes/SurfaceMesh.h>
#include <DGtal/geometry/surfaces/DigitalSurfaceRegularization.h>
#include <DGtal/dec/PolygonalCalculus.h>
#include <DGtal/dec/VectorsInHeat.h>

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

////////// NAMESPACES /////////

using namespace DGtal;

// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
// The following typedefs are useful
typedef SurfaceMesh< Z3i::RealPoint, Z3i::RealVector > 			  SurfMesh;
typedef SurfMesh::Vertices                   			  Vertices;
typedef SurfMesh::RealPoint                  			  RealPoint;
typedef SurfMesh::Face                       			  Face;
typedef SurfMesh::Vertex                     			  Vertex;
typedef PolygonalCalculus<SH3::RealPoint,SH3::RealVector> PC;
typedef PC::Vector Vector;
typedef PC::SparseMatrix SparseMatrix;

//Polyscope global
polyscope::SurfaceMesh *psMesh;
polyscope::PointCloud *psVertices;
polyscope::CurveNetwork *psBoundary;

SurfMesh surfmesh;

//Polygonal Calculus and VectorsInHeat solvers
PC *calculus;
VectorsInHeat<PC> *VHM;

//sources
std::vector<Vector> X_0;

bool noSources = true;

/**
 * @brief addRandomSource add a random vector in the tangent space
 * of a vertex
 */
void addRandomSource(){
    size_t id = rand()%surfmesh.nbVertices();
    VHM->addSource(id,Eigen::Vector3d::Random(3).normalized());

    X_0[id] = VHM->extrinsicVectorSourceAtVertex(id);
    psMesh->addVertexVectorQuantity("X_0",X_0);
    noSources = false;
}

/**
 * @brief diffuse solves systems and add the solution to the
 * display, if no source is given, adds a random one
 */
void diffuse(){
    if (noSources)
        addRandomSource();
    psMesh->addVertexVectorQuantity("VHM field",VHM->compute());
}

/**
 * @brief precompute initialize VHM solvers, and source container
 */
void precompute(){
    auto nv = surfmesh.nbVertices();
    auto ael = surfmesh.averageEdgeLength();
    VHM->init(ael*ael);//init vector heat method solvers

    X_0.resize(nv,Vector::Zero(3));//extrinsic Source vectors

    psMesh->addVertexVectorQuantity("X_0",X_0);
}

void myCallback()
{
    if(ImGui::Button("Compute Vector Field"))
    {
        diffuse();
    }
    if(ImGui::Button("Add random source"))
    {
        addRandomSource();
    }
}

int main(int, char **argv)
{
    std::vector<std::vector<SH3::SurfaceMesh::Vertex>> faces;
    std::vector<RealPoint> positions;

    //load voxel model
    auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
    params("surfaceComponents", "All");
    auto binary_image    = SH3::makeBinaryImage(argv[1], params );
    auto K               = SH3::getKSpace( binary_image, params );
    auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
    auto primalSurface   = SH3::makePrimalSurfaceMesh(surface);

    //Need to convert the faces
    for(size_t face= 0 ; face < primalSurface->nbFaces(); ++face)
        faces.push_back(primalSurface->incidentVertices( face ));

    //Recasting to vector of vertices
    positions = primalSurface->positions();

    surfmesh = SurfMesh(positions.begin(),
                        positions.end(),
                        faces.begin(),
                        faces.end());

    //instantiate PolyDEC
    calculus = new PC(surfmesh);

    //instantiate VHM
    VHM = new VectorsInHeat<PC>(calculus);

    //Initialize polyscope
    polyscope::init();

    psMesh = polyscope::registerSurfaceMesh("Digital Surface", positions, faces);

    //Initialize solvers
    precompute();

    polyscope::view::upDir = polyscope::view::UpDir::XUp;

    polyscope::state::userCallback = myCallback;

    polyscope::show();
    return EXIT_SUCCESS;

}
