#include <iostream>
#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>
#include <DGtal/shapes/SurfaceMesh.h>
#include <DGtal/geometry/surfaces/DigitalSurfaceRegularization.h>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/point_cloud.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace DGtal;
using namespace Z3i;

// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
// The following typedefs are useful
typedef SurfaceMesh< RealPoint, RealVector >  SurfMesh;
typedef SurfMesh::Vertices                    Vertices;
typedef SurfMesh::RealPoint                   RealPoint;
typedef SurfMesh::Face                   Face;
typedef SurfMesh::Vertex                  Vertex;


//Polyscope global
float dt=2.0;
// Polyscope visualization handle, to quickly add data to the surface
polyscope::SurfaceMesh *psMesh;
SurfMesh surfmesh;
polyscope::SurfaceMesh *psMeshReg;
SurfMesh surfmeshReg;


template<typename TSurfaceMesh>
struct SurfaceMeshDEC {
  
  typedef typename TSurfaceMesh::Vertex Vertex;
  typedef typename TSurfaceMesh::Face  Face;

  SurfaceMeshDEC(const ConstAlias<TSurfaceMesh> surf): mySurf(&surf)
  {
    myEmbedder =[&](Face f, Vertex v){ return mySurf->position(v);};
  };
  
  SurfaceMeshDEC(const ConstAlias<TSurfaceMesh> surf,
                const std::function<RealPoint(Face,Vertex)> &embedder): mySurf(&surf), myEmbedder(embedder)
  {};
  
  const TSurfaceMesh *mySurf;
  std::function<RealPoint(Face,Vertex)> myEmbedder;

  void setEmbedder(const std::function<RealVector(Face,Vertex)> &externalFunctor)
  {
    myEmbedder = externalFunctor;
  }
  
  //Restriction of a scalar function to vertices
  double phiVertex(const Vertex v) const
  {
    return  cos(10*(mySurf->position(v)[0]))*sin(10*mySurf->position(v)[1]);
  }
  
  //Restriction of a scalar function to vertices
  Eigen::VectorXd phi(const Face f) const
  {
    auto vertices = mySurf->incidentVertices(f);
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

  
  // nf x 3 position matrix
  Eigen::MatrixXd X(const Face f) const
  {
    auto vertices = mySurf->incidentVertices(f);
    auto nf = vertices.size();
    Eigen::MatrixXd Xt(nf,3);
    size_t cpt=0;
    for(auto v: vertices)
    {
      Xt(cpt,0) = myEmbedder(f,v)[0];
      Xt(cpt,1) = myEmbedder(f,v)[1];
      Xt(cpt,2) = myEmbedder(f,v)[2];
      ++cpt;
    }
    return  Xt;
  }
  
  //d0 per face
  Eigen::MatrixXd D(const Face f) const
  {
    auto vertices = mySurf->incidentVertices(f);
    auto nf = vertices.size();
    Eigen::MatrixXd d = Eigen::MatrixXd::Zero(nf ,nf);
    for(auto i=0; i < nf; ++i)
    {
      d(i,i) = -1.;
      d(i, (i+1)%nf) = 1.;
    }
    return d;
  }
  
  
  //Average operator on edges
  Eigen::MatrixXd A(const Face f) const
  {
    auto vertices = mySurf->incidentVertices(f);
    auto nf = vertices.size();
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(nf ,nf);
    for(auto i=0; i < nf; ++i)
    {
      a(i, (i+1)%nf) = 0.5;
      a(i,i) = 0.5;
    }
    return a;
  }
  
  //Vector area per face
  Eigen::Vector3d vectorArea(const Face f) const
  {
    RealVector af({0.0,0.0,0.0});
    auto vertices = mySurf->incidentVertices(f);
    auto it     = vertices.begin();
    auto itnext = vertices.begin();
    ++itnext;
    while (it != vertices.end())
    {
      auto xi  = mySurf->position(*it);
      auto xip = mySurf->position(*itnext);
      af += xi.crossProduct(xip);
      ++it;
      ++itnext;
      if (itnext == vertices.end())
        itnext =vertices.begin();
    }
    Eigen::Vector3d output({af[0],af[1],af[2]});
    return 0.5*output;
  }
  
  //Face area from vector
  double areaFace(const Face f) const
  {
    return vectorArea(f).norm();
  }
  
  //Normal vector per face
  Eigen::Vector3d normalFace(const Face f) const
  {
    Eigen::Vector3d v = vectorArea(f);
    v.normalize();
    return v;
  }
  
  //Return [n] as the operator such that [n]q = n x q
  Eigen::Matrix3d bracket(const Eigen::Vector3d &n) const
  {
    Eigen::Matrix3d brack;
    brack << 0.0 , -n(2), n(1),
    n(2), 0.0 , -n(0),
    -n(1) , n(0),0.0 ;
    return brack;
  }
  
  //Edge vector Operator E_f
  Eigen::MatrixXd E(const Face f) const
  {
    return D(f)*X(f);
  }
  
  //coGradient operator
  Eigen::MatrixXd coG(const Face f) const
  {
    return  E(f).transpose() * A(f);
  }
  
  //Gradient operator
  Eigen::MatrixXd G(const Face f) const
  {
    return -1.0/areaFace(f) * bracket( normalFace(f) ) * coG(f);
  }
  
  //Flat
  Eigen::MatrixXd V(const Face f) const
  {
    return E(f)*( Eigen::MatrixXd::Identity(3,3) - normalFace(f)*normalFace(f).transpose());
  }
  
  //Edge midPoints
  Eigen::MatrixXd B(const Face f) const
  {
    return A(f) * X(f);
  }
  
  //Centroids
  Eigen::VectorXd centroid(const Face f) const
  {
    auto nf = mySurf->incidentVertices(f).size();
    return 1/(double)nf * X(f).transpose() * Eigen::VectorXd::Ones(nf);
  }
  
  //Sharp
  Eigen::MatrixXd U(const Face f) const
  {
    auto nf = mySurf->incidentVertices(f).size();
    return 1/areaFace(f) * bracket(normalFace(f)) * ( B(f).transpose() - centroid(f)* Eigen::VectorXd::Ones(nf).transpose() );
  }
  
  //Projection
  Eigen::MatrixXd P(const Face f) const
  {
    auto nf = mySurf->incidentVertices(f).size();
    return Eigen::MatrixXd::Identity(nf,nf) - V(f)*U(f);
  }
  
  //Mass Matrix
  Eigen::MatrixXd M(const Face f, const double lambda=1.0) const
  {
    return areaFace(f) * U(f).transpose()*U(f) + lambda * P(f).transpose()*P(f);
  }
  
  //weak Laplacian
  Eigen::MatrixXd L(const Face f, const double lambda=1.0) const
  {
    return D(f).transpose() * M(f,lambda) * D(f);
  }
  
};

template<typename TSurfaceMesh>
struct SurfaceMeshDECVF {
  
  typedef typename TSurfaceMesh::Vertex Vertex;
  typedef typename TSurfaceMesh::Face  Face;
  
  SurfaceMeshDECVF(const ConstAlias<TSurfaceMesh> surf): mySurf(&surf)
  {
    myEmbedder =[&](Face f, Vertex v){ return mySurf->position(v);};
  };
  
  SurfaceMeshDECVF(const ConstAlias<TSurfaceMesh> surf,
                 const std::function<RealPoint(Face,Vertex)> &embedder): mySurf(&surf), myEmbedder(embedder)
  {};
  
  const TSurfaceMesh *mySurf;
  std::function<RealPoint(Face,Vertex)> myEmbedder;
  
  void setEmbedder(const std::function<RealVector(Face,Vertex)> &externalFunctor)
  {
    myEmbedder = externalFunctor;
  }
  
  //Restriction of a scalar function to vertices
  Eigen::Vector2d uVertex(const Vertex v) const
  {
    return  {1.0,0.0};
  }
  
  //Restriction of a scalar function to vertices
  Eigen::MatrixXd u(const Face f) const
  {
    auto vertices = mySurf->incidentVertices(f);
    auto nf = vertices.size();
    Eigen::MatrixXd u_f(nf);
    size_t cpt=0;
    for(auto v: vertices)
    {
      u_f(cpt) =  u(v);
      ++cpt;
    }
    return  u_f;
  }
};




template<typename Surf,typename Norm>
SHG3::RealVectors regularize(const Surf &surface,
                             const Norm &normals,
                             const unsigned int nbsteps,
           SH3::Cell2Index c2i )
{
  DigitalSurfaceRegularization<SH3::DigitalSurface> regul(surface);
  regul.init(0.001,1.0,0.0); //No Fairness
  auto surfelIndex = regul.getSurfelIndex();
  regul.attachNormalVectors([&](SH3::SCell &c){ return normals[ surfelIndex[c] ];});
  regul.regularize(nbsteps);

  auto reg_c2i = regul.getCellIndex();
  auto reg_pos = regul.getRegularizedPositions();
  auto ord_pos = reg_pos;
  for ( auto p : reg_c2i )
    {
      const auto pointel = p.first;
      const auto     idx = p.second;
      const auto new_idx = c2i[ pointel ];
      ord_pos[ new_idx ] = reg_pos[ idx ];
    }
  return ord_pos;
  //  return regul.getRegularizedPositions();
}




void doWorkHeat(const SurfMesh &mesh, const SurfaceMeshDEC<SurfMesh> &meshDEC ,unsigned int code)
{
  //Laplace-Beltrami
  Eigen::SparseMatrix<double> lapGlobal(mesh.nbVertices(), mesh.nbVertices());
  Eigen::SparseMatrix<double> Mass(mesh.nbVertices(), mesh.nbVertices());
  
  typedef Eigen::Triplet<double> T;
  for(auto f = 0; f <  mesh.nbFaces(); ++f)
  {
    auto vertices = mesh.incidentVertices(f);
    auto nf = vertices.size();
    Eigen::MatrixXd lf = meshDEC.L(f);
    Eigen::MatrixXd gf = meshDEC.G(f);
    Eigen::SparseMatrix<double> local(mesh.nbVertices(), mesh.nbVertices());
    Eigen::SparseMatrix<double> localMass(mesh.nbVertices(), mesh.nbVertices());
    std::vector<T> triplets;
    std::vector<T> tripletsMass;
    std::vector<size_t> reorder( nf );
    
    //FIXME: Mass matrix à la main
    auto cpt=0;
    for(auto v: vertices )
    {
      reorder[ cpt ]= v;
      tripletsMass.push_back(T(v,v, meshDEC.areaFace(f)/(double)nf));
      ++cpt;
    }
    
    for(auto i=0; i < nf; ++i)
    for(auto j=0; j < nf; ++j)
    {
      triplets.push_back(T(reorder[i],reorder[j],lf(i,j)));
    }
    
    local.setFromTriplets(triplets.begin(), triplets.end());
    localMass.setFromTriplets(tripletsMass.begin(), tripletsMass.end());
    lapGlobal += local;
    Mass += localMass;
  }
  Eigen::VectorXd PhiEigen( mesh.nbVertices());
  for(auto cpt=0; cpt < mesh.nbVertices(); ++cpt)
      PhiEigen(cpt) = meshDEC.phiVertex( cpt );
  
  
  Eigen::VectorXd LapPhiEigen = lapGlobal * PhiEigen;
  psMesh->addVertexScalarQuantity("LapPhiGlobal", LapPhiEigen);
  
  std::cout<<"Solve..."<<std::endl;
  
  //Source
  Eigen::VectorXd U = Eigen::VectorXd::Zero(mesh.nbVertices());
  Eigen::SparseMatrix<double> I(mesh.nbVertices(),mesh.nbVertices());
  I.setIdentity();
  U(3739) = 1.0; //dirac
  
 
  //Solver
  Eigen::SparseMatrix<double> heatOperator = Mass + dt*lapGlobal;
  geometrycentral::PositiveDefiniteSolver<double> heatSolver(heatOperator);
  geometrycentral::PositiveDefiniteSolver<double> poissonSolver(lapGlobal);
  
  // === Solve heat
  Eigen::VectorXd  heatVec = heatSolver.solve(U);
  
  //  // === Normalize in each face and evaluate divergence
  std::vector<RealPoint> gradHeat(mesh.nbFaces());
  Eigen::VectorXd divergenceVec = Eigen::VectorXd::Zero(mesh.nbVertices());
  for(auto f = 0; f < mesh.nbFaces(); ++f)
  {
    auto vertices = mesh.incidentVertices(f);
    auto nf = vertices.size();
    
    //Construct div per vertex of the heatVec gradient
    Eigen::VectorXd Heatf( nf );
    auto cpt=0;
    for(auto v: vertices)
    {
      Heatf(cpt) = heatVec( v );
      ++cpt;
    }
    Eigen::Vector3d g = meshDEC.G(f) * Heatf;
    g.normalize();
    gradHeat[f] = {g(0), g(1), g(2)};
    Eigen::MatrixXd oneForm = meshDEC.V(f)*g;
    Eigen::VectorXd divergence = meshDEC.D(f).transpose()*meshDEC.M(f)*oneForm;
    cpt=0;
    for(auto v: vertices)
    {
      divergenceVec(v) += divergence(cpt);
      ++cpt;
    }
  }
  
  // === Integrate divergence to get distance
  Eigen::VectorXd distVec = Eigen::VectorXd::Ones(mesh.nbVertices()) + poissonSolver.solve(divergenceVec);
  
  Eigen::VectorXd lu = lapGlobal*U;
  psMesh->addVertexDistanceQuantity("Diffusion distVec "+std::to_string(code), distVec);
  psMeshReg->addVertexDistanceQuantity("Diffusion distVec "+std::to_string(code), distVec);
}


void myCallback()
{
  ImGui::SliderFloat("dt", &dt, 0.1, 10.0);
  if (ImGui::Button("RedoHeat"))
  {
    SurfaceMeshDEC<SurfMesh> meshDEC(surfmesh);
    SurfaceMeshDEC<SurfMesh> meshRegDEC(surfmeshReg);
    doWorkHeat(surfmesh, meshDEC, 0);
    doWorkHeat(surfmeshReg, meshRegDEC, 1);
    
    }
}

int main()
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  
  auto h=.3 ; //gridstep
  params( "polynomial", "goursat" )( "gridstep", h );
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto K               = SH3::getKSpace( params );
  auto binary_image    = SH3::makeBinaryImage( digitized_shape, params );
  auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
  auto embedder        = SH3::getCellEmbedder( K );
  SH3::Cell2Index c2i;
  auto surfels         = SH3::getSurfelRange( surface, params );
  auto primalSurface   = SH3::makePrimalPolygonalSurface(c2i, surface);
  
  // Convert faces to appropriate indexed format
  std::vector<std::vector<unsigned long>> faces;
  for(auto &face: primalSurface->allFaces())
    faces.push_back(primalSurface->verticesAroundFace( face ));
  
  //Recasting to vector of vertices
  auto pos = primalSurface->positions();
  std::vector<RealPoint> positions(primalSurface->nbVertices());
  for(auto i=0; i < primalSurface->nbVertices(); ++i)
  positions[i] = pos(i);
  
  auto normalsII = SHG3::getIINormalVectors(binary_image, surfels, params);
    
  auto normalsTri = SHG3::getTrivialNormalVectors(K,surfels);
  
  //Reg
  auto  newpos = regularize(surface, normalsII, 100, c2i );
    
  
  surfmesh = SurfMesh(positions.begin(),
                      positions.end(),
                      faces.begin(),
                      faces.end());
  
  surfmeshReg = SurfMesh(newpos.begin(),
                         newpos.end(),
                         faces.begin(),
                         faces.end());
   
  
 
  // Initialize polyscope
  polyscope::init();
  
  // Set the callback function
  polyscope::state::userCallback = myCallback;
  
  // Register surface with quadrilateral faces
  psMesh = polyscope::registerSurfaceMesh("Primal surface 0",
                                          positions,
                                          faces)->setEdgeColor({1.,1.,1.})->setEdgeWidth(1.0);
  
  psMesh->addFaceVectorQuantity("normal II", normalsII);
  psMesh->addFaceVectorQuantity("normal triv", normalsTri);

  // Register surface with quadrilateral faces
  psMeshReg = polyscope::registerSurfaceMesh("Reg surface 1",
                                          newpos,
                                          faces)->setEdgeColor({1.,1.,1.})->setEdgeWidth(1.0);
  
  SurfaceMeshDEC<SurfMesh> meshDEC(surfmesh);
  SurfaceMeshDEC<SurfMesh> meshDEConReg(surfmeshReg);
  
  SurfaceMeshDEC<SurfMesh> meshDECwithEmbedder(surfmesh);
  auto myProjEmbedder = [&](Face f, Vertex v)
  {
    auto nn = normalsII[f];
    RealPoint centroid(0.0,0.0,0.0); //centroid of the original face
    auto cpt=0;
    for(auto v: meshDECwithEmbedder.mySurf->incidentVertices(f))
    { cpt++;
      centroid += meshDECwithEmbedder.mySurf->position(v);
    }
    centroid = centroid / (double)cpt;
    RealPoint p = meshDECwithEmbedder.mySurf->position(v);
    auto cp = p-centroid;
    RealPoint q = p - nn.dot(cp)*nn;
    return q;
  };
  
  //New TpM embedder
  meshDECwithEmbedder.setEmbedder(myProjEmbedder);
  
  doWorkHeat(surfmesh, meshDEC, 0);
  doWorkHeat(surfmeshReg, meshDEConReg, 1);
  doWorkHeat(surfmesh, meshDECwithEmbedder, 2);

  
  
  ///------------ DEBUG the TpM mesh ----------
  //Explicit mesh with projected quads
  std::vector<RealPoint> projPos;
  std::vector<Vertices> projFaces;
  auto idVert=0;
  for(auto f = 0; f < surfmesh.nbFaces(); ++f)
  {
    Vertices newface;
    auto cpt=0;
    for(auto v: surfmesh.incidentVertices(f))
    {
      auto p = myProjEmbedder(f, v);
      projPos.push_back(p);
      cpt++;
    }
    for(auto i = 0; i < cpt; ++i)
      newface.push_back(idVert+i);
    idVert += cpt;
    projFaces.push_back(newface);
  }
  polyscope::registerSurfaceMesh("Proj", projPos, projFaces);
  
  polyscope::show();
  return EXIT_SUCCESS;
  
}
