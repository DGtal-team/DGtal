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
 * @date 2022/07/19
 *
 * This file is part of the DGtal library.
 */
#include <iostream>
#include <string>
#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>
#include <DGtal/shapes/SurfaceMesh.h>
#include <DGtal/geometry/surfaces/DigitalSurfaceRegularization.h>

#include <DGtal/dec/PolygonalCalculus.h>
#include <DGtal/dec/GeodesicsInHeat.h>
#include <DGtal/dec/VectorsInHeat.h>

#include <DGtal/math/linalg/DirichletConditions.h>

#include <DGtal/shapes/Mesh.h>
#include <DGtal/io/readers/MeshReader.h>

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/curve_network.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "ConfigExamples.h"

using namespace DGtal;
using namespace Z3i;

// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
// The following typedefs are useful
typedef SurfaceMesh< RealPoint, RealVector >  SurfMesh;
typedef SurfMesh::Face                   Face;
typedef SurfMesh::Vertex                  Vertex;

typedef PolygonalCalculus<SH3::RealPoint,SH3::RealVector> PC;
typedef std::vector<Vertex> chain;
typedef DirichletConditions< PC::LinAlg > Conditions;
typedef Conditions::IntegerVector IntegerVector;
typedef PC::SparseMatrix SparseMatrix;
typedef PC::DenseMatrix DenseMatrix;
typedef PC::Solver Solver;

//Polyscope global
polyscope::SurfaceMesh *psMesh;
SurfMesh surfmesh;
float scale = 10;
PC::Vector phiEigen;
int nbSources=1;
std::vector< std::vector<SurfMesh::Index> > faces;
std::vector<RealPoint> positions;

//DEC
PC *calculus;



/**
 * @brief FixBoundaryParametrization maps the given boundary chain to uv
 *  coordinates (forms a circle, with arc-length parametrization)
 * @param boundary
 * @return the pair of uv parametrization as two vectors
 */
std::pair<PC::Vector,PC::Vector> FixBoundaryParametrization(const std::vector<Vertex>& boundary)
{
  auto nb = boundary.size();
  auto n = surfmesh.nbVertices();
  PC::Vector u = PC::Vector::Zero(n);
  PC::Vector v = PC::Vector::Zero(n);
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

  auto B = *std::max_element(chains.begin(),chains.end(),[] (const chain& A,const chain& B) {return A.size() < B.size();});

  //Visualization of the boundary edges
  std::vector<RealPoint> pos;
  for(const auto v: B)
    pos.push_back(surfmesh.position(v));
  polyscope::registerCurveNetworkLoop("Longest boundary", pos);

  IntegerVector boundary = IntegerVector::Zero(n);
  for (Vertex v : B)
    boundary(v) = 1;

  std::pair<PC::Vector,PC::Vector> uv_b = FixBoundaryParametrization(B);//maps boundary to circle

  calculus = new PC(surfmesh);

  //Impose dirichlet boundary condition to laplace problem
  PC::Vector Z = PC::Vector::Zero(n);
  SparseMatrix L = calculus->globalLaplaceBeltrami();
  SparseMatrix L_d = Conditions::dirichletOperator( L, boundary );

  PC::Solver solver;
  solver.compute(L_d);

  PC::Vector b_u = Conditions::dirichletVector( L, Z,boundary, uv_b.first );
  PC::Vector b_v = Conditions::dirichletVector( L, Z,boundary, uv_b.second );

  PC::Vector rslt_u_d = solver.solve(b_u);
  PC::Vector rslt_v_d = solver.solve(b_v);

  PC::Vector rslt_u = Conditions::dirichletSolution(rslt_u_d,boundary,uv_b.first);
  PC::Vector rslt_v = Conditions::dirichletSolution(rslt_v_d,boundary,uv_b.second);

  DenseMatrix uv(n,2);
  uv.col(0) = rslt_u;
  uv.col(1) = rslt_v;

  return uv;
}


//Restriction of a scalar function to vertices
double phiVertex(const Vertex v)
{
  return  cos(scale*(surfmesh.position(v)[0]))*sin(scale*surfmesh.position(v)[1]);
}

//Restriction of a scalar function to vertices
PolygonalCalculus<SH3::RealPoint,SH3::RealVector>::Vector phi(const Face f)
{
  auto vertices = surfmesh.incidentVertices(f);
  auto nf = vertices.size();
  Eigen::VectorXd ph(nf);
  size_t cpt=0;
  for(auto v: vertices)
  {
    ph(cpt) =  phiVertex(v);
    ++cpt;
  }
  return  ph;
}

void initPhi()
{
  phiEigen.resize(surfmesh.nbVertices());
  for(auto i = 0; i < surfmesh.nbVertices(); ++i)
    phiEigen(i) = phiVertex(i);
  psMesh->addVertexScalarQuantity("Phi", phiEigen);
}

void initQuantities()
{
  trace.beginBlock("Basic quantities");
  PolygonalCalculus<SH3::RealPoint,SH3::RealVector> calculus(surfmesh);

  std::vector<PC::Vector> gradients;
  std::vector<PC::Vector> cogradients;
  std::vector<PC::Real3dPoint> normals;
  std::vector<PC::Real3dPoint> vectorArea;
  std::vector<PC::Real3dPoint> centroids;

  std::vector<double> faceArea;

  for(auto f=0; f < surfmesh.nbFaces(); ++f)
  {
    PC::Vector grad = calculus.gradient(f) * phi(f);
    gradients.push_back( grad );

    PC::Vector cograd =  calculus.coGradient(f) * phi(f);
    cogradients.push_back( cograd );

    normals.push_back(calculus.faceNormalAsDGtalVector(f));

    PC::Vector vA = calculus.vectorArea(f);
    vectorArea.push_back({vA(0) , vA(1), vA(2)});

    faceArea.push_back( calculus.faceArea(f));
  }
  trace.endBlock();

  psMesh->addFaceVectorQuantity("Gradients", gradients);
  psMesh->addFaceVectorQuantity("co-Gradients", cogradients);
  psMesh->addFaceVectorQuantity("Normals", normals);
  psMesh->addFaceScalarQuantity("Face area", faceArea);
  psMesh->addFaceVectorQuantity("Vector area", vectorArea);
}

void computeGeodesics()
{
  PC calculus(surfmesh);
  GeodesicsInHeat<PC> GHM(calculus);
  std::vector<double> X_0;

  auto nv = surfmesh.nbVertices();
  auto ael = surfmesh.averageEdgeLength();
  GHM.init(ael*ael);//init vector heat method solvers

  X_0.resize(nv,0);//extrinsic Source vectors

  //Random Sources
  for(auto i=0; i < nbSources; ++i)
  {
    size_t id = rand()%surfmesh.nbVertices();
    GHM.addSource(id);
    X_0[id] = 42.0;
  }
  psMesh->addVertexScalarQuantity("X_0",X_0);
  psMesh->addVertexDistanceQuantity("GeodeiscInHeat", GHM.compute());
}


void computeHeatVectors()
{
  PC calculus(surfmesh);
  VectorsInHeat<PC> VHM(calculus);
  typedef PC::Vector Vector;
  std::vector<Vector> X_0;

  auto nv = surfmesh.nbVertices();
  auto ael = surfmesh.averageEdgeLength();
  VHM.init(ael*ael);//init vector heat method solvers

  X_0.resize(nv,Vector::Zero(3));//extrinsic Source vectors

  //Random Sources
  for(auto i=0; i < nbSources; ++i)
  {
    size_t id = rand()%surfmesh.nbVertices();
    VHM.addSource(id,Eigen::Vector3d::Random(3).normalized());
    X_0[id] = VHM.extrinsicVectorSourceAtVertex(id);
  }

  psMesh->addVertexVectorQuantity("X_0",X_0);
  psMesh->addVertexVectorQuantity("VHM field",VHM.compute());
}

void myCallback()
{
  ImGui::SliderFloat("Phi scale", &scale, 0., 10.);
  ImGui::SliderInt("Nb Sources", &nbSources, 1, 10);

  if (ImGui::Button("Phi and basic operators"))
  {
    initPhi();
    initQuantities();
  }
  if (ImGui::Button("Geodesics in heat"))
    computeGeodesics();
  if (ImGui::Button("Heat Vectors"))
    computeHeatVectors();
  if(ImGui::Button("Harmonic parametrization"))
  {
    //compute parametrization
    DenseMatrix UV = HarmonicParametrization();

    //visualize parametrization on 2D circle
    VisualizeParametrizationOnCircle(UV);
    psMesh->addVertexParameterizationQuantity("Harmonic parametrization",UV)->setEnabled(true);
  }
}

int main()
{
  std::string inputFilename(examplesPath + "samples/bunnyheadhole.obj" );

  Mesh<RealPoint> a3DMesh;
  a3DMesh << inputFilename;

  for(auto it = a3DMesh.faceBegin(), itend = a3DMesh.faceEnd(); it != itend; ++it)
    faces.push_back(*it);

  //Need to convert the faces
  surfmesh = SurfMesh(a3DMesh.vertexBegin(),
                      a3DMesh.vertexEnd(),
                      faces.begin(),
                      faces.end());

  std::cout<<"number of non-manifold Edges = " << surfmesh.computeNonManifoldEdges().size()<<std::endl;
  psMesh = polyscope::registerSurfaceMesh("Bimba surface", surfmesh.positions(), faces);

  // Initialize polyscope
  polyscope::init();

  // Set the callback function
  polyscope::state::userCallback = myCallback;
  polyscope::show();
  return EXIT_SUCCESS;
}
