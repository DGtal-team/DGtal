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
 * @file exampleHarmonicParametrization.cpp
 * @author Baptiste GENEST (\c baptistegenest@gmail.com )
 * internship at Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
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
#include <DGtal/dec/PolygonalCalculus.h>
#include "DGtal/math/linalg/DirichletConditions.h"

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>

#include "ConfigExamples.h"

////////// NAMESPACES /////////

using namespace DGtal;

////////// TYPEDEFS /////////
// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
// The following typedefs are useful
typedef SurfaceMesh< Z3i::RealPoint, Z3i::RealVector >  SurfMesh;
typedef SurfMesh::Vertices                    Vertices;
typedef SurfMesh::RealPoint                   RealPoint;
typedef SurfMesh::Face                        Face;
typedef SurfMesh::Vertex                      Vertex;
typedef PolygonalCalculus<SH3::RealPoint,SH3::RealVector> PC;
typedef PC::SparseMatrix SparseMatrix;
typedef PC::DenseMatrix DenseMatrix;
typedef PC::Solver Solver;
typedef PC::Vector Vector;
typedef PC::Triplet Triplet;
typedef std::vector<Vertex> chain;
typedef DirichletConditions< PC::LinAlg > Conditions;
typedef Conditions::IntegerVector IntegerVector;


//Polyscope global
polyscope::SurfaceMesh *psMesh;
polyscope::SurfaceMesh *psParam;

//DEC
PC *calculus;

//surface mesh global
SurfMesh surfmesh;
std::vector<std::vector<size_t>> faces;
std::vector<RealPoint> positions;


/**
 * @brief FixBoundaryParametrization maps the give boundary chain to uv
 *  coordinates (forms a circle, with arc-length parametrization)
 * @param boundary
 * @return the pair of uv parametrization as two vectors
 */
std::pair<Vector,Vector> FixBoundaryParametrization(const std::vector<Vertex>& boundary)
{
    auto nb = boundary.size();
    auto n = surfmesh.nbVertices();
    Vector u = Vector::Zero(n),v = Vector::Zero(n);
    double totalBoundaryLength = 0;
    for (Vertex i = 0;i<nb;i++)
        totalBoundaryLength += surfmesh.distance(boundary[(i+1)%nb],boundary[i]);

    double partialSum = 0;
    for (Vertex i = 0;i<nb;i++)
    {
        double th = 2*M_PI*partialSum/totalBoundaryLength;
        auto vi = boundary[i];
        auto vj = boundary[(i+1)%nb];
        u(vi) = std::cos(th);
        v(vj) = std::sin(th);
        partialSum += surfmesh.distance(vi,vj);
    }
    return {u,v};
}

/**
 * @brief VisualizeParametrizationOnCircle creates a polyscope mesh in 2D
 * with UV coordinates
 * @param UV input parametrization
 */
void VisualizeParametrizationOnCircle(const DenseMatrix& UV)
{
    auto n= surfmesh.nbVertices();
    std::vector<RealPoint> pos(n);
    double scale = 0;
    RealPoint avg = {0.,0.,0.};
    for (size_t v = 0;v<n;v++)
    {
        auto p = surfmesh.position(v);
        avg += p;
        if (p.norm() > scale)
            scale = p.norm();
    }
    avg /= surfmesh.nbVertices();
    scale /= 2;
    for (size_t v = 0;v<n;v++)
        pos[v] = RealPoint{scale*UV(v,0),scale*UV(v,1),0.} + avg;
    polyscope::registerSurfaceMesh("On circle parametrization", pos, faces)->setEnabled(false);
}

/**
 * @brief HarmonicParametrization computes the harmonic parametrization of the
 * loaded mesh, the fixed boundary is the largest one,
 * all holes must be homeomorphic to circles
 * @return (n,2) matrix with uv coordinates in columns
 */
DenseMatrix HarmonicParametrization()
{
    auto n = surfmesh.nbVertices();
    std::cout<<"Nb boundary edges = "<< surfmesh.computeManifoldBoundaryEdges().size()<<std::endl;
    std::vector<chain> chains = surfmesh.computeManifoldBoundaryChains();
    //choose longest chain as boundary of the parametrization
    std::cout<<"Nb boundaries  = "<< chains.size() << std::endl;

    //choose longest chain as boundary of the parametrization
    auto B = *std::max_element(chains.begin(),chains.end(),[] (const chain& A,const chain& B) {return A.size() < B.size();});

    IntegerVector boundary = IntegerVector::Zero(n);
    for (Vertex v : B)
        boundary(v) = 1;

    std::pair<Vector,Vector> uv_b = FixBoundaryParametrization(B);//maps boundary to circle

    calculus = new PC(surfmesh);

    //Impose dirichlet boundary condition to laplace problem
    Vector Z = Vector::Zero(n);
    SparseMatrix L = calculus->globalLaplaceBeltrami();
    SparseMatrix L_d = Conditions::dirichletOperator( L, boundary );

    PC::Solver solver;
    solver.compute(L_d);

    Vector b_u = Conditions::dirichletVector( L, Z,boundary, uv_b.first );
    Vector b_v = Conditions::dirichletVector( L, Z,boundary, uv_b.second );

    Vector rslt_u_d = solver.solve(b_u);
    Vector rslt_v_d = solver.solve(b_v);

    Vector rslt_u = Conditions::dirichletSolution(rslt_u_d,boundary,uv_b.first);
    Vector rslt_v = Conditions::dirichletSolution(rslt_v_d,boundary,uv_b.second);

    DenseMatrix uv(n,2);
    uv.col(0) = rslt_u;
    uv.col(1) = rslt_v;

    return uv;
}

int main(int argc, char **argv)
{
    //Import Voxel Model
    auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
    params("surfaceComponents", "All");

    //load .vol
    std::string inputFilename(examplesPath + "samples/bunny-64.vol" );

    auto binary_image    = SH3::makeBinaryImage(inputFilename, params );

    //offset K space to create boundary
    auto K               = SH3::getKSpace( binary_image, params );
    K.init(K.lowerBound()+SH3::Point(5,0,0),K.upperBound(),true);

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


    // Initialize polyscope
    polyscope::init();

    //compute parametrization
    DenseMatrix UV = HarmonicParametrization();

    //visualize parametrization on 2D circle
    VisualizeParametrizationOnCircle(UV);

    psMesh = polyscope::registerSurfaceMesh("Digital Surface", positions, faces);
    psMesh->addVertexParameterizationQuantity("Harmonic parametrization",UV)->setEnabled(true);

    //set correct view for voxel mesh
    polyscope::view::upDir = polyscope::view::UpDir::XUp;

    polyscope::show();
    return EXIT_SUCCESS;
}
