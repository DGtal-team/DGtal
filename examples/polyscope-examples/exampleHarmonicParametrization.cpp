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
 * @file polyscope-examples/exampleHarmonicParametrization.cpp
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
#include <DGtal/dec/VectorsInHeat.h>

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>

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
typedef Eigen::Triplet<double> Triplet;
typedef std::vector<Vertex> chain;


//Polyscope global
polyscope::SurfaceMesh *psMesh;
polyscope::SurfaceMesh *psParam;
polyscope::PointCloud *psVertices;
polyscope::CurveNetwork *psBoundary;

//DEC
PC *calculus;
VectorsInHeat<PC> *VHM;

//surface mesh global
SurfMesh surfmesh;
std::vector<std::vector<size_t>> faces;
std::vector<RealPoint> positions;

/**
 * @brief block
 * @return the sub sparse matrix that starts at (row,col) of size (height,width)
 */
SparseMatrix block(const SparseMatrix& mat, size_t row, size_t col,size_t height,size_t width)
{
    SparseMatrix B(height,width);
    auto last_col = col + width;
    auto last_row = row + height;
    std::vector<Triplet> T;
    for (size_t i = col;i<last_col;i++)
        for (SparseMatrix::InnerIterator it(mat, i); it; ++it)
            if (it.row() >= (long)row)
            {
                if (it.row() < (long)last_row)
                {
                    T.push_back(Triplet(it.row()-row,i-col,it.value()));
                }
                else
                    break;
            }
    B.setFromTriplets(T.begin(),T.end());
    return B;
}

/**
 * @brief computeManifoldBoundaryChains
 * @return returns the list of chains that borders the mesh
 */
std::vector<chain> computeManifoldBoundaryChains(int nb_chains = -1){

    std::map<Vertex,bool> visited;
    std::map<Vertex,std::vector<Vertex>> adjacent;

    //computes unordered list of boundary vertices
    auto MBE = surfmesh.computeManifoldBoundaryEdges();

    assert(MBE.size());//if null then no boundary

    //BFS on edge
    for (auto e : MBE){
        auto ij = surfmesh.edgeVertices(e);

        visited[ij.first] = false;
        visited[ij.second] = false;

        adjacent[ij.first].push_back(ij.second);
        //vertex linked to more than 2 other vertices, hence cannot form a chain
        assert(adjacent[ij.first].size()<=2);

        adjacent[ij.second].push_back(ij.first);
        //vertex linked to more than 2 other vertices, hence cannot form a chain
        assert(adjacent[ij.second].size()<=2);
    }

    std::vector<chain> boundaries;
    auto boundary_it = visited.begin();
    do{
        Vertex first = (*boundary_it).first;
        visited[first] = true;

        chain boundary;
        boundary.push_back(first);

        Vertex current = first;

        size_t nb_iter = 0;
        while (nb_iter < MBE.size()*2){
            bool ok = false;
            for (auto other : adjacent[current])
                if (!visited[other]){
                    boundary.push_back(other);
                    current = other;
                    visited[other] = true;
                    ok = true;
                    break;
                }
            if (!ok){//all neighboors are visited
                for (auto other : adjacent[current])
                    if (other == first){
                        boundaries.push_back(boundary);
                        break;
                    }
                    //if first vertex isn't found then this chain is not
                    //homeomorphic to a circle, hence isn't added to boundaries
            }
            nb_iter++;
            if (nb_chains >= 0 && boundaries.size() >= (unsigned long)nb_chains )
                return boundaries;
        }
        boundary_it = std::find_if(visited.begin(), visited.end(),
                                   []
                                   (std::pair<Vertex,bool> x){return !x.second;});
    //loop as long as all boundary vertices aren't visited
    } while(boundary_it != visited.end());
    return boundaries;
}

/**
 * @brief edgeLength
 * @return length of the vector between vertex i and j (the edge doesn't have to exist)
 */
double edgeLength(Vertex i,Vertex j){
    return(surfmesh.position(i)-surfmesh.position(j)).norm();
}

/**
 * @brief FixBoundaryParametrization maps the give boundary chain to uv coordinates (forms a circle, with arc-length parametrization)
 * @param boundary
 * @return
 */
std::pair<Vector,Vector> FixBoundaryParametrization(const std::vector<Vertex>& boundary){
    auto nb = boundary.size();
    Vector u(nb),v(nb);
    double totalBoundaryLength = 0;
    for (Vertex i = 0;i<nb;i++)
        totalBoundaryLength += edgeLength(boundary[(i+1)%nb],boundary[i]);

    double partialSum = 0;
    for (Vertex i = 0;i<nb;i++){
        double th = 2*M_PI*partialSum/totalBoundaryLength;
        u(i) = std::cos(th);
        v(i) = std::sin(th);
        partialSum += edgeLength(boundary[(i+1)%nb],boundary[i]);
    }
    return {u,v};
}

/**
 * @brief VisualizeParametrizationOnCircle creates a polyscope mesh in 2D
 * with UV coordinates
 * @param UV inpute parametrization
 */
void VisualizeParametrizationOnCircle(const DenseMatrix& UV){
    auto n= surfmesh.nbVertices();
    std::vector<RealPoint> pos(n);
    for (size_t v = 0;v<n;v++){
        pos[v] = {UV(v,0),UV(v,1),0.};
    }
    polyscope::registerSurfaceMesh("Bunny param", pos, faces)->setEnabled(false);
}

/**
 * @brief HarmonicParametrization computes the harmonic parametrization of the
 * loaded mesh, the fixed boundary is the largest one,
 * all holes must be homeomorphic to circles
 * WARNING: reorders surfaceMesh indices.
 * @return (n,2) matrix with uv coordinates in columns
 */
DenseMatrix HarmonicParametrization(){
    auto chains = computeManifoldBoundaryChains(2);
    std::cout << chains.size() << " chains found" << std::endl;
    //choose longest chain as boundary of the parametrization
    auto B = *std::max_element(chains.begin(),chains.end(),[] (const chain& A,const chain& B) {return A.size() < B.size();});

    std::vector<bool> isBoundary(surfmesh.nbVertices(),false);
    auto uv_b = FixBoundaryParametrization(B);//maps boundary to circle

    std::vector<RealPoint> new_pos;
    std::vector<std::vector<size_t>> new_faces;

    std::vector<RealPoint> b_pos;

    //reorder vertices to facilitate boundary conditions
    //{ reordering
    std::vector<size_t> new_index(surfmesh.nbVertices());
    size_t i = 0;
    for (auto b : B){
        new_pos.push_back(surfmesh.position(b));
        b_pos.push_back(surfmesh.position(b));
        new_index[b] = i;
        isBoundary[b] = true;
        i++;
    }

    for (size_t v = 0;v<surfmesh.nbVertices();v++)
        if (!isBoundary[v]){
            new_pos.push_back(surfmesh.position(v));
            new_index[v] = i;
            i++;
        }

    for (size_t f = 0;f<surfmesh.nbFaces();f++){
        std::vector<size_t> ids;
        for (auto v :surfmesh.incidentVertices(f) )
            ids.push_back(new_index[v]);
        new_faces.push_back(ids);
    }

    surfmesh = SurfMesh(new_pos.begin(),
                new_pos.end(),
                new_faces.begin(),
                new_faces.end());

    faces = new_faces;
    positions = new_pos;
    //}

    uint n = surfmesh.nbVertices();
    calculus = new PC(surfmesh);

    //Impose dirichlet boundary condition to laplace problem
    auto nb = B.size();
    SparseMatrix L   = calculus->globalLaplaceBeltrami();

    SparseMatrix L11 = block(L,nb,nb,n-nb,n-nb);
    SparseMatrix L10 = block(L,nb,0,n-nb,nb);

    PC::Solver solver;
    solver.compute(L11);

    Vector rslt_u = solver.solve(-L10*uv_b.first);
    Vector rslt_v = solver.solve(-L10*uv_b.second);

    DenseMatrix uv(n,2);
    uv.block(0,0,nb,1) = uv_b.first;
    uv.block(nb,0,n-nb,1) = rslt_u;
    uv.block(0,1,nb,1) = uv_b.second;
    uv.block(nb,1,n-nb,1) = rslt_v;

    return uv;
}

int main(int, char **argv)
{
    //Import Voxel Model
    auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
    params("surfaceComponents", "All");

    //load .vol
    auto binary_image    = SH3::makeBinaryImage(argv[1], params );

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
    psMesh->addVertexParameterizationQuantity("Harmonic parametrization",UV);

    //set correct view for voxel mesh
    polyscope::view::upDir = polyscope::view::UpDir::XUp;

    polyscope::show();
    return EXIT_SUCCESS;
}
