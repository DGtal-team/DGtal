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

#pragma once

/**
 * @file
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2021/09/02
 *
 * Header file for module PolygonalCalculus.cpp
 *
 * This file is part of the DGtal library.
 */
//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <functional>
#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include "DGtal/base/ConstAlias.h"
#include "DGtal/base/Common.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/math/linalg/EigenSupport.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  namespace functors {
    /**
     *
     * \brief Functor that projects a face vertex of a surface mesh onto the tangent plane
     * given by a per-face normal vector.
     * This functor can be used in PolygonalCalculus to correct the embedding of
     * digital surfaces using an estimated normal vector field (see @cite coeurjolly2022simple).
     *
     * @note when used in PolygonalCalculus, all operators being invariant by translation, all
     * tangent planes pass through the origin (0,0,0) (no offset).
     *
     * @tparam TRealPoint a model of points @f$\mathbb{R}^3@f$ (e.g. PointVector).
     * @tparam TRealVector a model of vectors in @f$\mathbb{R}^3@f$ (e.g. PointVector).
     */    template <typename TRealPoint, typename TRealVector>
    struct EmbedderFromNormalVectors
    {
      ///Type of SurfaceMesh
      typedef SurfaceMesh<TRealPoint, TRealVector> MySurfaceMesh;
      ///Vertex type
      typedef typename MySurfaceMesh::Vertex Vertex;
      ///Face type
      typedef typename MySurfaceMesh::Face Face;
      ///Position type
      typedef typename MySurfaceMesh::RealPoint Real3dPoint;

      EmbedderFromNormalVectors() = delete;

      /// Constructor from an array of normal vectors and a surface mesh instance.
      /// @param normals a vector of per face normal vectors (same ordering as the SurfaceMesh face indices).
      /// @param surfmesh an instance of SurfaceMesh
      EmbedderFromNormalVectors(ConstAlias<std::vector<Real3dPoint>> normals,
                                ConstAlias<MySurfaceMesh> surfmesh)
      {
        myNormals     = &normals;
        mySurfaceMesh = &surfmesh;
      }

      /// Project a face vertex onto its tangent plane (given by the per-face estimated
      /// normal vector).
      ///
      /// @param f the face that contains the vertex
      /// @param v the vertex to project
      Real3dPoint operator()(const Face &f,const Vertex &v)
      {
        const auto nn = (*myNormals)[f];
        Real3dPoint p = mySurfaceMesh->position(v);
        return p - nn.dot(p)*nn;
      }

      ///Alias to the normal vectors
      const std::vector<Real3dPoint> *myNormals;
      ///Alias to the surface mesh
      const MySurfaceMesh *mySurfaceMesh;
    };
  }



/////////////////////////////////////////////////////////////////////////////
// template class PolygonalCalculus
/**
 * Description of template class 'PolygonalCalculus' <p>
 * \brief Implements differential operators on polygonal surfaces from
 * @cite degoes2020discrete
 *
 * See @ref modulePolygonalCalculus for details.
 *
 * @note The sign convention for the divergence and the Laplacian
 * operator is opposite to the one of @cite degoes2020discrete. This
 * is to match the usual mathematical convention that the Laplacian
 * (and the Laplacian-Beltrami) has negative eigenvalues (and is the
 * sum of second derivatives in the cartesian grid). It also follows
 * the formal adjointness of exterior derivative and opposite of
 * divergence as relation \f$ \langle \mathrm{d} u, v \rangle = -
 * \langle u, \mathrm{div} v \rangle \f$. See also
 * https://en.wikipedia.org/wiki/Laplace–Beltrami_operator
 *
 * @tparam TRealPoint a model of points @f$\mathbb{R}^3@f$ (e.g. PointVector).
 * @tparam TRealVector a model of vectors in @f$\mathbb{R}^3@f$ (e.g. PointVector).
 */
template <typename TRealPoint, typename TRealVector>
class PolygonalCalculus
{
  // ----------------------- Standard services ------------------------------
public:

  ///Concept checking
  static const Dimension dimension = TRealPoint::dimension;
  BOOST_STATIC_ASSERT( ( dimension == 3 ) );

  ///Self type
  typedef PolygonalCalculus<TRealPoint, TRealVector> Self;

  ///Type of SurfaceMesh
  typedef SurfaceMesh<TRealPoint, TRealVector> MySurfaceMesh;
  ///Vertex type
  typedef typename MySurfaceMesh::Vertex Vertex;
  ///Face type
  typedef typename MySurfaceMesh::Face Face;
  ///Position type
  typedef typename MySurfaceMesh::RealPoint Real3dPoint;
  ///Real vector type
  typedef typename MySurfaceMesh::RealVector Real3dVector;

  ///Linear Algebra Backend from Eigen
  typedef EigenLinearAlgebraBackend LinAlg;
  ///Type of Vector
  typedef LinAlg::DenseVector Vector;
  ///Global 0-form, 1-form, 2-form are Vector
  typedef Vector Form;
  ///Type of dense matrix
  typedef LinAlg::DenseMatrix DenseMatrix;
  ///Type of sparse matrix
  typedef LinAlg::SparseMatrix SparseMatrix;
  ///Type of sparse matrix triplet
  typedef LinAlg::Triplet Triplet;

  ///Type of a sparse matrix solver
  typedef LinAlg::SolverSimplicialLDLT Solver;

  /// @name Standard services
  /// @{

  /// Create a Polygonal DEC structure from a surface mesh (@a surf)
  /// using an default identity embedder.
  /// @param surf an instance of SurfaceMesh
  /// @param globalInternalCacheEnabled enable the internal cache for all operators (default: false)
  PolygonalCalculus(const ConstAlias<MySurfaceMesh> surf,
                    bool globalInternalCacheEnabled = false):
          mySurfaceMesh(&surf),  myGlobalCacheEnabled(globalInternalCacheEnabled)
  {
    myEmbedder =[&](Face f,Vertex v){ (void)f; return mySurfaceMesh->position(v);};
    myVertexNormalEmbedder = [&](Vertex v){ return toReal3dVector(computeVertexNormal(v));};
    init();
  };

  /// Create a Polygonal DEC structure from a surface mesh (@a surf)
  /// and an embedder for the vertex position: function with two parameters, a face and a vertex
  /// which outputs the embedding in R^3 of the vertex w.r.t. to the face.
  /// @param surf an instance of SurfaceMesh
  /// @param embedder an embedder for the vertex position
  /// @param globalInternalCacheEnabled if true, the global operator cache is enabled
  PolygonalCalculus(const ConstAlias<MySurfaceMesh> surf,
                    const std::function<Real3dPoint(Face,Vertex)> &embedder,
                    bool globalInternalCacheEnabled = false):
  mySurfaceMesh(&surf), myEmbedder(embedder), myGlobalCacheEnabled(globalInternalCacheEnabled)
  {
    myVertexNormalEmbedder = [&](Vertex v){ return toReal3dVector(computeVertexNormal(v)); };
    init();
  };

  /// Create a Polygonal DEC structure from a surface mesh (@a surf)
  /// and an embedder for the vertex normal: function with a vertex as
  /// parameter which outputs the embedding in R^3 of the vertex normal.
  /// @param surf an instance of SurfaceMesh
  /// @param embedder an embedder for the vertex position
  /// @param globalInternalCacheEnabled if true, the global operator cache is enabled
  PolygonalCalculus(const ConstAlias<MySurfaceMesh> surf,
                    const std::function<Real3dVector(Vertex)> &embedder,
                    bool globalInternalCacheEnabled = false):
      mySurfaceMesh(&surf), myVertexNormalEmbedder(embedder),
      myGlobalCacheEnabled(globalInternalCacheEnabled)
  {
    myEmbedder = [&](Face f,Vertex v){(void)f; return mySurfaceMesh->position(v); };
    init();
  };

  /// Create a Polygonal DEC structure from a surface mesh (@a surf)
  /// and an embedder for the vertex position: function with two parameters, a
  /// face and a vertex which outputs the embedding in R^3 of the vertex
  /// w.r.t. to the face. and an embedder for the vertex normal: function with
  /// a vertex as parameter which outputs the embedding in R^3 of the vertex
  /// normal.
  /// @param surf an instance of SurfaceMesh
  /// @param pos_embedder an embedder for the position
  /// @param normal_embedder an embedder for the position
  /// @param globalInternalCacheEnabled
  PolygonalCalculus(const ConstAlias<MySurfaceMesh> surf,
                    const std::function<Real3dPoint(Face,Vertex)> &pos_embedder,
                    const std::function<Vector(Vertex)> &normal_embedder,
                    bool globalInternalCacheEnabled = false) :
  mySurfaceMesh(&surf), myEmbedder(pos_embedder),
  myVertexNormalEmbedder(normal_embedder),
  myGlobalCacheEnabled(globalInternalCacheEnabled)
  {
    init();
  };

  /**
   * Deleted default constructor.
   */
  PolygonalCalculus() = delete;

  /**
   * Destructor (default).
   */
  ~PolygonalCalculus() = default;

  /**
   * Deleted copy constructor.
   * @param other the object to clone.
   */
  PolygonalCalculus ( const PolygonalCalculus & other ) = delete;

  /**
   * Deleted move constructor.
   * @param other the object to move.
   */
  PolygonalCalculus ( PolygonalCalculus && other ) = delete;

  /**
   * Deleted copy assignment operator.
   * @param other the object to copy.
   * @return a reference on 'this'.
   */
  PolygonalCalculus & operator= ( const PolygonalCalculus & other ) = delete;

  /**
   * Deleted move assignment operator.
   * @param other the object to move.
   * @return a reference on 'this'.
   */
  PolygonalCalculus & operator= ( PolygonalCalculus && other ) = delete;

  /// @}

  // ----------------------- embedding  services  --------------------------
  //MARK: Embedding services
 /// @name Embedding services
  /// @{

  /// Update the embedding function.
  /// @param externalFunctor a new embedding functor (Face,Vertex)->RealPoint.
  void setEmbedder(const std::function<Real3dPoint(Face,Vertex)> &externalFunctor)
  {
    myEmbedder = externalFunctor;
  }
  /// @}

  // ----------------------- Per face operators --------------------------------------
  //MARK: Per face operator on scalars
  /// @name Per face operators on scalars
  /// @{

  /// Return the vertex position matrix degree x 3 of the face.
  /// @param f a face
  /// @return the n_f x 3 position matrix
  DenseMatrix X(const Face f) const
  {
    if (checkCache(X_,f))
      return myGlobalCache[X_][f];

    const auto vertices = mySurfaceMesh->incidentVertices(f);
    const auto nf = myFaceDegree[f];
    DenseMatrix Xt(nf,3);
    size_t cpt=0;
    for(auto v: vertices)
    {
      Xt(cpt,0) = myEmbedder(f,v)[0];
      Xt(cpt,1) = myEmbedder(f,v)[1];
      Xt(cpt,2) = myEmbedder(f,v)[2];
      ++cpt;
    }

    setInCache(X_,f,Xt);
    return  Xt;
  }


  /// Derivative operator (d_0) of a face.
  /// @param f the face
  /// @return a degree x degree matrix
  DenseMatrix D(const Face f) const
  {
    if (checkCache(D_,f))
      return myGlobalCache[D_][f];

    const auto nf = myFaceDegree[f];
    DenseMatrix d = DenseMatrix::Zero(nf ,nf);
    for(auto i=0u; i < nf; ++i)
    {
      d(i,i) = -1.;
      d(i, (i+1)%nf) = 1.;
    }

    setInCache(D_,f,d);
    return d;
  }

  /// Edge vector operator per face.
  /// @param f the face
  /// @return degree x 3 matrix
  DenseMatrix E(const Face f) const
  {
    if (checkCache(E_,f))
      return myGlobalCache[E_][f];

    DenseMatrix op = D(f)*X(f);

    setInCache(E_,f,op);
    return op;
  }

  /// Average operator to average, per edge, its vertex values.
  /// @param f the face
  /// @return a degree x degree matrix
  DenseMatrix A(const Face f) const
  {
    if (checkCache(A_,f))
      return myGlobalCache[A_][f];

    const auto nf = myFaceDegree[f];
    DenseMatrix a = DenseMatrix::Zero(nf ,nf);
    for(auto i=0u; i < nf; ++i)
    {
      a(i, (i+1)%nf) = 0.5;
      a(i,i) = 0.5;
    }

    setInCache(A_,f,a);
    return a;
  }


  /// Polygonal (corrected) vector area.
  /// @param f the face
  /// @return a vector oriented in the (corrected) normal direction and with length equal to the (corrected) area of the face \a f.
  Vector vectorArea(const Face f) const
  {
    Real3dPoint af(0.0,0.0,0.0);
    const auto vertices = mySurfaceMesh->incidentVertices(f);
    auto it     = vertices.cbegin();
    auto itnext = vertices.cbegin();
    ++itnext;
    while (it != vertices.cend())
    {
      auto xi  = myEmbedder(f,*it);
      auto xip = myEmbedder(f,*itnext);
      af += xi.crossProduct(xip);
      ++it;
      ++itnext;
      if (itnext == vertices.cend())
        itnext = vertices.cbegin();
    }
    Eigen::Vector3d output = {af[0],af[1],af[2]};
    return 0.5*output;
  }

  /// Area of a face from the vector area.
  /// @param f the face
  /// @return the corrected area of the face
  double faceArea(const Face f) const
  {
    return vectorArea(f).norm();
  }

  /// Corrected normal vector of a face.
  /// @param f the face
  /// @return a vector (Eigen vector)
  Vector faceNormal(const Face f) const
  {
    Vector v = vectorArea(f);
    v.normalize();
    return v;
  }

  /// Corrected normal vector of a face.
  /// @param f the face
  /// @return a vector (DGtal RealVector/RealPoint)
  Real3dVector faceNormalAsDGtalVector(const Face f) const
  {
    Vector v = faceNormal(f);
    return {v(0),v(1),v(2)};
  }

  /// co-Gradient operator of the face
  /// @param f the face
  /// @return a 3 x degree matrix
  DenseMatrix coGradient(const Face f) const
  {
    if (checkCache(COGRAD_,f))
      return myGlobalCache[COGRAD_][f];
    DenseMatrix op = E(f).transpose() * A(f);
    setInCache(COGRAD_, f, op);
    return op;
  }

  ///Return [n] as the 3x3 operator such that [n]q = n x q
  ///@param n a vector
  DenseMatrix bracket(const Vector &n) const
  {
    DenseMatrix brack(3,3);
    brack << 0.0 , -n(2), n(1),
             n(2), 0.0 , -n(0),
            -n(1) , n(0),0.0 ;
    return brack;
  }

  /// Gradient operator of the face.
  /// @param f the face
  /// @return 3 x degree matrix
  DenseMatrix gradient(const Face f) const
  {
    if (checkCache(GRAD_,f))
      return myGlobalCache[GRAD_][f];

    DenseMatrix op = -1.0/faceArea(f) * bracket( faceNormal(f) ) * coGradient(f);

    setInCache(GRAD_,f,op);
    return op;
  }

  /// Flat operator for the face.
  /// @param f the face
  /// @return a degree x 3 matrix
  DenseMatrix  flat(const Face f) const
  {
    if (checkCache(FLAT_,f))
      return myGlobalCache[FLAT_][f];
    DenseMatrix n = faceNormal(f);
    DenseMatrix op = E(f)*( DenseMatrix::Identity(3,3) - n*n.transpose());
    setInCache(FLAT_,f,op);
    return op;
  }

  /// Edge mid-point operator of the face.
  /// @param f the face
  /// @return a degree x 3 matrix
  DenseMatrix B(const Face f) const
  {
    if (checkCache(B_,f))
      return myGlobalCache[B_][f];
    DenseMatrix res = A(f) * X(f);
    setInCache(B_,f,res);
    return res;
  }

  /// @returns the centroid of the face
  /// @param f the face
  Vector centroid(const Face f) const
  {
    const auto nf = myFaceDegree[f];
    return 1.0/(double)nf * X(f).transpose() * Vector::Ones(nf);
  }

  /// @returns the centroid of the face as a DGtal RealPoint
  /// @param f the face
  Real3dPoint centroidAsDGtalPoint(const Face f) const
  {
    const Vector c = centroid(f);
    return {c(0),c(1),c(2)};
  }

  /// Sharp operator for the face.
  /// @param f the face
  /// @return a 3 x degree matrix
  DenseMatrix sharp(const Face f) const
  {
    if (checkCache(SHARP_,f))
      return myGlobalCache[SHARP_][f];

    const auto  nf = myFaceDegree[f];
    DenseMatrix op = 1.0/faceArea(f) * bracket(faceNormal(f)) *
          ( B(f).transpose() - centroid(f)* Vector::Ones(nf).transpose() );

    setInCache(SHARP_,f,op);
    return op;
  }

  /// Projection operator for the face.
  /// @param f the face
  /// @return a degree x degree matrix
  DenseMatrix P(const Face f) const
  {
    if (checkCache(P_,f))
      return myGlobalCache[P_][f];

    const auto  nf = myFaceDegree[f];
    DenseMatrix op = DenseMatrix::Identity(nf,nf) - flat(f)*sharp(f);

    setInCache(P_, f, op);
    return op;
  }

  /// Inner product on 1-forms associated with the face
  /// @param f the face
  /// @param lambda the regularization parameter
  /// @return a degree x degree matrix
  DenseMatrix M(const Face f, const double lambda=1.0) const
  {
    if (checkCache(M_,f))
      return myGlobalCache[M_][f];

    DenseMatrix Uf = sharp(f);
    DenseMatrix Pf = P(f);
    DenseMatrix op = faceArea(f) * Uf.transpose()*Uf + lambda * Pf.transpose()*Pf;

    setInCache(M_,f,op);
    return op;
  }

  /// Divergence operator of a one-form.
  /// @param f the face
  /// @return a degree x degree matrix
  ///
  /// @note The sign convention for the divergence and the Laplacian
  /// operator is opposite to the one of @cite degoes2020discrete .
  /// This is to match the usual mathematical
  /// convention that the Laplacian (and the Laplacian-Beltrami) has
  /// negative eigenvalues (and is the sum of second derivatives in
  /// the cartesian grid). It also follows the formal adjointness of
  /// exterior derivative and opposite of divergence as relation \f$
  /// \langle \mathrm{d} u, v \rangle = - \langle u, \mathrm{div} v
  /// \rangle \f$. See also
  /// https://en.wikipedia.org/wiki/Laplace–Beltrami_operator
  DenseMatrix divergence(const Face f) const
  {
    if (checkCache(DIVERGENCE_,f))
      return myGlobalCache[DIVERGENCE_][f];

    DenseMatrix op = -1.0 * D(f).transpose() * M(f);
    setInCache(DIVERGENCE_,f,op);

    return op;
  }

  /// Curl operator of a one-form (identity matrix).
  /// @param f the face
  /// @return a degree x degree matrix
  DenseMatrix curl(const Face f) const
  {
    if (checkCache(CURL_,f))
      return myGlobalCache[CURL_][f];

    DenseMatrix op = DenseMatrix::Identity(myFaceDegree[f],myFaceDegree[f]);

    setInCache(CURL_,f,op);
    return op;
  }


  /// (weak) Laplace-Beltrami operator for the face.
  /// @param f the face
  /// @param lambda the regularization parameter
  /// @return a degree x degree matrix
  ///
  /// @note The sign convention for the divergence and the Laplacian
  /// operator is opposite to the one of @cite degoes2020discrete .
  /// This is to match the usual mathematical
  /// convention that the Laplacian (and the Laplacian-Beltrami) has
  /// negative eigenvalues (and is the sum of second derivatives in
  /// the cartesian grid). It also follows the formal adjointness of
  /// exterior derivative and opposite of divergence as relation \f$
  /// \langle \mathrm{d} u, v \rangle = - \langle u, \mathrm{div} v
  /// \rangle \f$. See also https://en.wikipedia.org/wiki/Laplace–Beltrami_operator
  DenseMatrix laplaceBeltrami(const Face f, const double lambda=1.0) const
  {
    if (checkCache(L_,f))
      return myGlobalCache[L_][f];

    DenseMatrix Df = D(f);
    // Laplacian is a negative operator.
    DenseMatrix op = -1.0 * Df.transpose() * M(f,lambda) * Df;

    setInCache(L_, f, op);
    return op;
  }
  ///@}

  // ----------------------- Vector calculus----------------------------------
  //MARK: Vector Field Calculus
  ///@name Per face operators on vector fields
  ///@{

public:
  ///@return 3x2 matrix defining the tangent space at vertex v, with basis
  ///vectors in columns
  DenseMatrix Tv(const Vertex & v) const
  {
    Eigen::Vector3d nv = n_v(v);
    ASSERT(std::abs(nv.norm() - 1.0) < 0.001);
    const auto & N            = getSurfaceMeshPtr()->neighborVertices(v);
    auto neighbor             = *N.begin();
    Real3dPoint tangentVector = getSurfaceMeshPtr()->position(v) -
                                getSurfaceMeshPtr()->position(neighbor);
    Eigen::Vector3d w  = toVec3(tangentVector);
    Eigen::Vector3d uu = project(w,nv).normalized();
    Eigen::Vector3d vv = nv.cross(uu);

    DenseMatrix tanB(3,2);
    tanB.col(0) = uu;
    tanB.col(1) = vv;
    return tanB;
  }

  ///@return 3x2 matrix defining the tangent space at face f, with basis
  ///vectors in columns
  DenseMatrix Tf(const Face & f) const
  {
    Eigen::Vector3d nf = faceNormal(f);
    ASSERT(std::abs(nf.norm() - 1.0) < 0.001);
    const auto & N = getSurfaceMeshPtr()->incidentVertices(f);
    auto v1        = *(N.begin());
    auto v2        = *(N.begin() + 1);
    Real3dPoint tangentVector =
    getSurfaceMeshPtr()->position(v2) - getSurfaceMeshPtr()->position(v1);
    Eigen::Vector3d w  = toVec3(tangentVector);
    Eigen::Vector3d uu = project(w,nf).normalized();
    Eigen::Vector3d vv = nf.cross(uu);

    DenseMatrix tanB(3,2);
    tanB.col(0) = uu;
    tanB.col(1) = vv;
    return tanB;
  }

  /// \brief toExtrinsicVector
  /// \param v the vertex
  /// \param I the intrinsic vector at Tv
  /// \return 3D extrinsic vector from intrinsic 2D vector I expressed from
  /// tangent frame at vertex v
  Vector toExtrinsicVector(const Vertex v, const Vector & I) const
  {
    DenseMatrix T = Tv(v);
    return T.col(0) * I(0) + T.col(1) * I(1);
  }

  /// \param I  set of intrinsic vectors, vectors indices must be the same as
  /// their associated vertex
  ///@return converts a set of intrinsic vectors to their extrinsic
  ///equivalent, expressed in corresponding tangent frame
  std::vector<Vector> toExtrinsicVectors(const std::vector<Vector> & I) const
  {
    std::vector<Vector> ext(mySurfaceMesh->nbVertices());
    for (auto v = 0; v < mySurfaceMesh->nbVertices(); v++)
      ext[v] = toExtrinsicVector(v,I[v]);
    return ext;
  }

  /// https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
  ///@return 3x3 Rotation matrix to align n_v to n_f
  DenseMatrix Qvf(const Vertex & v, const Face & f) const
  {
    Eigen::Vector3d nf = faceNormal(f);
    Eigen::Vector3d nv = n_v(v);
    double c           = nv.dot(nf);
    ASSERT(std::abs( c + 1.0) > 0.0001);
    //Special case for opposite nv and nf vectors.
    if (std::abs( c + 1.0) < 0.00001)
      return -Eigen::Matrix3d::Identity();

    auto vv          = nv.cross(nf);
    DenseMatrix skew = bracket(vv);
    return Eigen::Matrix3d::Identity() + skew +
           1.0 / (1.0 + c) * skew * skew;
  }

  ///@return Levi-Civita connection from vertex v tangent space to face f
  ///tangent space (2x2 rotation matrix)
  DenseMatrix Rvf(const Vertex & v, const Face & f) const
  {
    return Tf(f).transpose() * Qvf(v,f) * Tv(v);
  }

  /// Shape operator on the face @a f (2x2 operator).
  ///@return the shape operator at face f
  DenseMatrix shapeOperator(const Face f) const
  {
    DenseMatrix N(myFaceDegree[f],3);
    uint cpt = 0;
    for (Vertex v : mySurfaceMesh->incidentVertices(f))
    {
      N.block(cpt,0,3,1) = n_v(v).transpose();
      cpt++;
    }
    DenseMatrix GN = gradient(f) * N, Tf = T_f(f);

    return 0.5 * Tf.transpose() * (GN + GN.transpose()) * Tf;
  }

  /// @return to fit @cite degoes2020discrete paper's notations,
  /// this function maps all the per vertex vectors (expressed in the (2*nf)
  /// vector form) into the nfx2 matrix with transported vectors (to face f) in
  /// each row.
  /// @note Unlike the rest of the per face operators, the
  /// covariant operators need to be applied directly to the restriction of
  /// the vector field to the face,
  DenseMatrix transportAndFormatVectorField(const Face f, const Vector & uf)
  {
    DenseMatrix uf_nabla(myFaceDegree[f], 2);
    size_t cpt = 0;
    for (auto v : mySurfaceMesh->incidentVertices(f))
    {
      uf_nabla.block(cpt,0,1,2) =
      (Rvf(v,f) * uf.block(2 * cpt,0,2,1)).transpose();
      ++cpt;
    }
    return uf_nabla;
  }

  /// Covarient gradient at a face a @a f of intrinsic vectors @a uf.
  /// @param uf list of all intrinsic vectors per vertex concatenated in a
  /// column vector
  /// @param f the face
  /// @return the covariant gradient of the given vector field uf (expressed
  /// in corresponding vertex tangent frames), wrt face f
  /// @note Unlike the rest of the per face operators, the
  /// covariant operators need to be applied directly to the restriction of
  /// the vector field to the face,
  DenseMatrix covariantGradient(const Face f, const Vector & uf)
  {
    return Tf(f).transpose() * gradient(f) *
           transportAndFormatVectorField(f,uf);
  }

  /// Compute the covariance projection at a face @a f of intrinsic vectors @a uf.
  /// @param uf list of all intrinsic vectors per vertex concatenated in a
  /// column vector
  /// @param f the face
  /// @return the covariant projection of the given vector field uf (
  /// restricted to face f and expressed in corresponding vertex tangent
  /// frames)
  /// @note Unlike the rest of the per face operators, the
  /// covariant operators need to be applied directly to the restriction of
  /// the vector field to the face
  DenseMatrix covariantProjection(const Face f, const Vector & uf)
  {
    return P(f) * D(f) * transportAndFormatVectorField(f,uf);
  }

  /// @return Covariant Gradient Operator, returns the operator that acts on
  /// the concatenated vectors. When applied, gives the associated 2x2 matrix
  /// in the isomorphic vector form (a b c d)^t to be used in the dirichlet
  /// energy (vector laplacian) G∇_f.
  /// Used to define the connection Laplacian.
  DenseMatrix covariantGradient_f(const Face & f) const
  {
    return kroneckerWithI2(Tf(f).transpose() * gradient(f)) * blockConnection(f);
  }

  /// @return Projection Gradient Operator, returns the operator that acts on
  /// the concatenated vectors. When applied, gives the associated nfx2 matrix
  /// in the isomorphic vector form (a b c d ...)^t to be used in the
  /// dirichlet energy (vector laplacian) P∇_f.
  /// Used to define the connection Laplacian.
  DenseMatrix covariantProjection_f(const Face & f) const
  {
    return kroneckerWithI2(P(f) * D(f)) * blockConnection(f);
    ;
  }

  /// L∇ := -(afG∇tG∇+λP∇tP∇)
  /// @return Connection/Vector laplacian at face f
  /// @note The sign convention for the divergence and the Laplacian
  /// operator is opposite to the one of @cite degoes2020discrete.
  /// This is to match the usual mathematical
  /// convention that the laplacian (and the Laplacian-Beltrami) has
  /// negative eigenvalues (and is the sum of second derivatives in
  /// the cartesian grid). It also follows the formal adjointness of
  /// exterior derivative and opposite of divergence as relation \f$
  /// \langle \mathrm{d} u, v \rangle = - \langle u, \mathrm{div} v
  /// \rangle \f$. See also
  /// https://en.wikipedia.org/wiki/Laplace–Beltrami_operator
  DenseMatrix connectionLaplacian(const Face & f, double lambda = 1.0) const
  {
    if (checkCache(CON_L_,f))
      return myGlobalCache[CON_L_][f];
    DenseMatrix G = covariantGradient_f(f);
    DenseMatrix P = covariantProjection_f(f);
    DenseMatrix L = -(faceArea(f) * G.transpose() * G + lambda * P.transpose() * P);
    setInCache(CON_L_,f,L);
    return L;
  }
  /// @}

  // ----------------------- Global operators --------------------------------------
  //MARK: Global Operators
  /// @name Global operators
  /// @{

  /// @return a 0-form initialized to zero
  Form form0() const
  {
    return Form::Zero( nbVertices() );
  }
  /// @return the identity linear operator for 0-forms
  SparseMatrix identity0() const
  {
    SparseMatrix Id0( nbVertices(), nbVertices() );
    Id0.setIdentity();
    return Id0;
  }

  /// Computes the global Laplace-Beltrami operator by assembling the
  /// per face operators.
  ///
  /// @param lambda the regularization parameter for the local Laplace-Beltrami operators
  /// @return a sparse nbVertices x nbVertices matrix
  ///
  /// @note The sign convention for the divergence is opposite to the
  /// one of @cite degoes2020discrete. This is also true for the
  /// Laplacian operator. This is to match the usual mathematical
  /// convention that the Laplacian (and the Laplacian-Beltrami) has
  /// negative eigenvalues (and is the sum of second derivatives in
  /// the cartesian grid). It also follows the formal adjointness of
  /// exterior derivative and opposite of divergence as relation \f$
  /// \langle \mathrm{d} u, v \rangle = - \langle u, \mathrm{div} v
  /// \rangle \f$. See also https://en.wikipedia.org/wiki/Laplace–Beltrami_operator
  SparseMatrix globalLaplaceBeltrami(const double lambda=1.0) const
  {
    SparseMatrix lapGlobal(mySurfaceMesh->nbVertices(), mySurfaceMesh->nbVertices());
    std::vector<Triplet> triplets;
    for( typename MySurfaceMesh::Index f = 0; f < mySurfaceMesh->nbFaces(); ++f )
    {
      auto nf = myFaceDegree[f];
      DenseMatrix Lap = this->laplaceBeltrami(f,lambda);
      const auto vertices = mySurfaceMesh->incidentVertices(f);
      for(auto i=0u; i < nf; ++i)
        for(auto j=0u; j < nf; ++j)
        {
          auto v = Lap(i,j);
          if (v!= 0.0)
            triplets.emplace_back( Triplet( (SparseMatrix::StorageIndex)vertices[ i ], (SparseMatrix::StorageIndex)vertices[ j ],
                                            Lap( i, j ) ) );
        }
    }
    lapGlobal.setFromTriplets(triplets.begin(), triplets.end());
    return lapGlobal;
  }

  /// Compute and returns the global lumped mass matrix
  /// (diagonal matrix with Max's weights for each vertex).
  ///    M(i,i) =   ∑_{adjface f} faceArea(f)/degree(f) ;
  ///
  /// @return the global lumped mass matrix.
  SparseMatrix globalLumpedMassMatrix() const
  {
    SparseMatrix M(mySurfaceMesh->nbVertices(), mySurfaceMesh->nbVertices());
    std::vector<Triplet> triplets;
    for ( typename MySurfaceMesh::Index v = 0; v < mySurfaceMesh->nbVertices(); ++v )
      {
        auto faces = mySurfaceMesh->incidentFaces(v);
        auto varea = 0.0;
        for(auto f: faces)
          varea += faceArea(f) /(double)myFaceDegree[f];
        triplets.emplace_back(Triplet(v,v,varea));
      }
    M.setFromTriplets(triplets.begin(),triplets.end());
    return M;
  }

  /// Compute and returns the inverse of the global lumped mass matrix
  /// (diagonal matrix with Max's weights for each vertex).
  ///
  /// @return the inverse of the global lumped mass matrix.
  SparseMatrix globalInverseLumpedMassMatrix() const
  {
    SparseMatrix iM0 = globalLumpedMassMatrix();
    for ( int k = 0; k < iM0.outerSize(); ++k )
      for ( typename SparseMatrix::InnerIterator it( iM0, k ); it; ++it )
        it.valueRef() = 1.0 / it.value();
    return iM0;
  }

  /// Computes the global Connection-Laplace-Beltrami operator by accumulating
  /// the per face operators.
  ///
  /// @param lambda the regualrization parameter for the local
  /// Connection-Laplace-Beltrami operators
  /// @return a sparse 2*nbVertices x 2*nbVertices matrix
  ///
  /// @note The sign convention for the divergence is opposite to the
  /// one of @cite degoes2020discrete. This is also true for the
  /// Laplacian operator. This is to match the usual mathematical
  /// convention that the Laplacian (and the Laplacian-Beltrami) has
  /// negative eigenvalues (and is the sum of second derivatives in
  /// the cartesian grid). It also follows the formal adjointness of
  /// exterior derivative and opposite of divergence as relation \f$
  /// \langle \mathrm{d} u, v \rangle = - \langle u, \mathrm{div} v
  /// \rangle \f$. See also
  /// https://en.wikipedia.org/wiki/Laplace–Beltrami_operator
  SparseMatrix globalConnectionLaplace(const double lambda = 1.0) const
  {
    auto nv = mySurfaceMesh->nbVertices();
    SparseMatrix lapGlobal(2 * nv, 2 * nv);
    SparseMatrix local(2 * nv, 2 * nv);
    std::vector<Triplet> triplets;
    for (typename MySurfaceMesh::Index f = 0; f < mySurfaceMesh->nbFaces(); f++)
    {
      auto nf             = degree(f);
      DenseMatrix Lap     = connectionLaplacian(f,lambda);
      const auto vertices = mySurfaceMesh->incidentVertices(f);
      for (auto i = 0u; i < nf; ++i)
        for (auto j = 0u; j < nf; ++j)
          for (short k1 = 0; k1 < 2; k1++)
            for (short k2 = 0; k2 < 2; k2++)
            {
              auto v = Lap(2 * i + k1, 2 * j + k2);
              if (v != 0.0)
                triplets.emplace_back(Triplet(2 * vertices[i] + k1,
                                                2 * vertices[j] + k2, v));
            }
    }
    lapGlobal.setFromTriplets(triplets.begin(), triplets.end());
    return lapGlobal;
  }

  /// Compute and returns the global lumped mass matrix tensorized with Id_2
  /// (used for connection laplacian) (diagonal matrix with Max's weights for
  /// each vertex).
  ///    M(2*i,2*i) 		=   ∑_{adjface f} faceArea(f)/degree(f) ;
  ///    M(2*i+1,2*i+1)   =   M(2*i,2*i)
  /// @return the global lumped mass matrix.
  SparseMatrix doubledGlobalLumpedMassMatrix() const
  {
    auto nv = mySurfaceMesh->nbVertices();
    SparseMatrix M(2 * nv, 2 * nv);
    std::vector<Triplet> triplets;
    for (typename MySurfaceMesh::Index v = 0; v < mySurfaceMesh->nbVertices(); ++v)
    {
      auto faces = mySurfaceMesh->incidentFaces(v);
      auto varea = 0.0;
      for (auto f : faces)
        varea += faceArea(f) / (double)myFaceDegree[f];
      triplets.emplace_back(Triplet(2 * v, 2 * v, varea));
      triplets.emplace_back(Triplet(2 * v + 1, 2 * v + 1, varea));
    }
    M.setFromTriplets(triplets.begin(), triplets.end());
    return M;
  }
  /// @}

  // ----------------------- Cache mechanism --------------------------------------
  /// @name Cache mechanism
  /// @{

  /// Generic method to compute all the per face DenseMatrices and store them in an
  /// indexed container.
  ///
  /// Usage example:
  /// @code
  ///auto opM = [&](const PolygonalCalculus<Mesh>::Face f){ return calculus.M(f);};
  ///auto cacheM = boxCalculus.getOperatorCacheMatrix(opM);
  ///...
  /////Now you have access to the cached values and mixed them with un-cached ones
  ///  Face f = ...;
  ///  auto res = cacheM[f] * calculus.D(f) * phi;
  /// ...
  ///@endcode
  ///
  /// @param perFaceOperator the per face operator
  /// @return an indexed container of all DenseMatrix operators (indexed per Face).
  std::vector<DenseMatrix> getOperatorCacheMatrix(const std::function<DenseMatrix(Face)> &perFaceOperator) const
  {
    std::vector<DenseMatrix> cache;
    for( typename MySurfaceMesh::Index f=0; f < mySurfaceMesh->nbFaces(); ++f)
      cache.push_back(perFaceOperator(f));
    return cache;
  }

  /// Generic method to compute all the per face Vector and store them in an
  /// indexed container.
  ///
  /// Usage example:
  /// @code
  ///auto opCentroid = [&](const PolygonalCalculus<Mesh>::Face f){ return calculus.centroid(f);};
  ///auto cacheCentroid = boxCalculus.getOperatorCacheVector(opCentroid);
  ///...
  /////Now you have access to the cached values and mixed them with un-cached ones
  ///  Face f = ...;
  ///  auto res = calculus.P(f) * cacheCentroid[f] ;
  /// ...
  ///@endcode
  ///
  /// @param perFaceVectorOperator the per face operator
  /// @return an indexed container of all Vector quantities (indexed per Face).
  std::vector<Vector> getOperatorCacheVector(const std::function<Vector(Face)> &perFaceVectorOperator) const
  {
    std::vector<Vector> cache;
    for( typename MySurfaceMesh::Index f=0; f < mySurfaceMesh->nbFaces(); ++f)
      cache.push_back(perFaceVectorOperator(f));
    return cache;
  }

  /// Enable the internal global cache for operators.
  ///
  void enableInternalGlobalCache()
  {
    myGlobalCacheEnabled = true;
  }

  /// Disable the internal global cache for operators.
  /// This method will also clean up the
  void disableInternalGlobalCache()
  {
    myGlobalCacheEnabled = false;
    myGlobalCache.clear();
  }

  /// @}

  // ----------------------- Common --------------------------------------
public:
  /// @name Common services
  /// @{

  /// Update the internal cache structures
  /// (e.g. degree of each face).
  void init()
  {
    updateFaceDegree();
  }

  /// Helper to retrieve the degree of the face from the cache.
  /// @param f the face
  /// @return the number of vertices of the face.
  size_t faceDegree(Face f) const
  {
    return myFaceDegree[f];
  }

  /// @return the number of vertices of the underlying surface mesh.
  size_t nbVertices() const
  {
    return mySurfaceMesh->nbVertices();
  }

  /// @return the number of faces of the underlying surface mesh.
  size_t nbFaces() const
  {
    return mySurfaceMesh->nbFaces();
  }

  /// @returns the degree of the face f (number of vertices)
  /// @param f the face
  size_t degree(const Face f) const
  {
    return myFaceDegree[f];
  }

  /// @returns an pointer to the underlying SurfaceMash object.
  const MySurfaceMesh * getSurfaceMeshPtr() const
  {
    return mySurfaceMesh;
  }

  /**
   * Writes/Displays the object on an output stream.
   * @param out the output stream where the object is written.
   */
  void selfDisplay ( std::ostream & out ) const
  {
    out << "[PolygonalCalculus]: ";
    if (myGlobalCacheEnabled)
      out<< "internal cache enabled, ";
    else
      out<<"internal cache disabled, ";
    out <<"SurfaceMesh="<<*mySurfaceMesh;
  }

  /**
   * Checks the validity/consistency of the object.
   * @return 'true' if the object is valid, 'false' otherwise.
   */
  bool isValid() const
  {
    return true;
  }

  /// @}

  // ------------------------- Protected Data ------------------------------
  //MARK: Protected

protected:
  /// @name Protected services and types
  /// @{

  ///Enum for operators in the internal cache strategy
  enum OPERATOR { X_, D_, E_, A_, COGRAD_, GRAD_, FLAT_, B_, SHARP_, P_, M_, DIVERGENCE_, CURL_, L_,CON_L_};

  /// Update the face degree cache
  void updateFaceDegree()
  {
    myFaceDegree.resize(mySurfaceMesh->nbFaces());
    for(typename MySurfaceMesh::Index f = 0; f <  mySurfaceMesh->nbFaces(); ++f)
    {
      auto vertices = mySurfaceMesh->incidentVertices(f);
      auto nf = vertices.size();
      myFaceDegree[f] = nf;
    }
  }

  /// Check internal cache if enabled.
  /// @param key the operator name
  /// @param f the face
  /// @returns true if the operator "key" for the face f has been computed.
  bool checkCache(OPERATOR key, const Face f) const
  {
    if (myGlobalCacheEnabled)
      if (myGlobalCache[key].find(f) != myGlobalCache[key].end())
        return true;
    return false;
  }

  /// Set an operator in the internal cache.
  /// @param key the operator name
  /// @param f the face
  /// @param ope the operator to store
  void setInCache(OPERATOR key, const Face f,
                  const DenseMatrix &ope) const
  {
    if (myGlobalCacheEnabled)
      myGlobalCache[key][f]  = ope;
  }

  /// Project u on the orthgonal of n
  /// \param u vector to project
  /// \param n vector to build orthogonal space from
  /// \return projected vector
  static Vector project(const Vector & u, const Vector & n)
  {
    return u - (u.dot(n) / n.squaredNorm()) * n;
  }

  /// Conversion routines.
  /// \brief toVector convert Real3dPoint to Eigen::VectorXd
  /// \param x the vector
  /// \return the same vector in eigen type
  static Vector toVector(const Eigen::Vector3d & x)
  {
    Vector X(3);
    for (int i = 0; i < 3; i++)
      X(i) = x(i);
    return X;
  }

  /// \brief toVec3 convert Real3dPoint to Eigen::Vector3d
  /// \param x the vector
  /// \return the same vector in eigen type
  static Eigen::Vector3d toVec3(const Real3dPoint & x)
  {
    return Eigen::Vector3d(x(0),x(1),x(2));
  }

  /// Conversion routines.
    /// \brief toReal3dVector converts Eigen::Vector3d to Real3dVector.
    /// \param x the vector
    /// \return the same vector in DGtal type
    static Real3dVector toReal3dVector(const Eigen::Vector3d & x)
    {
      return { x(0), x(1), x(2)};
    }


  /// Compute the (normalized) normal vector at a Vertex by averaging
  /// the adjacent face normal vectors.
  /// \param v the vertex to compute the normal from
  /// \return 3D normal vector at vertex v
  ///
  Vector computeVertexNormal(const Vertex & v) const
  {
    Vector n(3);
    n(0) = 0.;
    n(1) = 0.;
    n(2) = 0.;
   /* for (auto f : mySurfaceMesh->incidentFaces(v))
      n += vectorArea(f);
    return n.normalized();
    */
    auto faces = mySurfaceMesh->incidentFaces(v);
    for (auto f : faces)
      n += vectorArea(f);

    if (fabs(n.norm() - 0.0) < 0.00001)
    {
      //On non-manifold edges touching the boundary, n may be null.
      trace.warning()<<"[PolygonalCalculus] Trying to compute the normal vector at a boundary vertex incident to pnon-manifold edge, we return a random vector."<<std::endl;
      n << Vector::Random(3);
    }
    n = n.normalized();
    return n;
  }

  ///@return the normal vector at vertex v, if no normal vertex embedder is
  ///set, the normal will be computed
  Eigen::Vector3d n_v(const Vertex & v) const
  {
    return toVec3(myVertexNormalEmbedder(v));
  }

  //Covariant operators routines
  /// @return Block Diagonal matrix with Rvf for each vertex v in face f
  DenseMatrix blockConnection(const Face & f) const
  {
    auto nf           = degree(f);
    DenseMatrix RU_fO = DenseMatrix::Zero(nf * 2,nf * 2);
    size_t cpt        = 0;
    for (auto v : getSurfaceMeshPtr()->incidentVertices(f))
    {
      auto Rv                               = Rvf(v,f);
      RU_fO.block<2,2>(2 * cpt,2 * cpt) = Rv;
      ++cpt;
    }
    return RU_fO;
  }

  /// @return the tensor-kronecker product of M with 2x2 identity matrix
  DenseMatrix kroneckerWithI2(const DenseMatrix & M) const
  {
    size_t h       = M.rows();
    size_t w       = M.cols();
    DenseMatrix MK = DenseMatrix::Zero(h * 2,w * 2);
    for (size_t j = 0; j < h; j++)
      for (size_t i = 0; i < w; i++)
      {
        MK(2 * j, 2 * i)         = M(j, i);
        MK(2 * j + 1, 2 * i + 1) = M(j, i);
      }
    return MK;
  }



  /// @}

  // ------------------------- Internals ------------------------------------
  //MARK: Internals
private:

  ///Underlying SurfaceMesh
  const MySurfaceMesh *mySurfaceMesh;

  ///Embedding function (face,vertex)->R^3 for the vertex position wrt. the face.
  std::function<Real3dPoint(Face, Vertex)> myEmbedder;

  ///Embedding function (vertex)->R^3 for the vertex normal.
  std::function<Real3dVector(Vertex)> myVertexNormalEmbedder;

  ///Cache containing the face degree
  std::vector<size_t> myFaceDegree;

  ///Global cache
  bool myGlobalCacheEnabled;
  mutable std::array<std::unordered_map<Face,DenseMatrix>, 15> myGlobalCache;

}; // end of class PolygonalCalculus

/**
 * Overloads 'operator<<' for displaying objects of class 'PolygonalCalculus'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'PolygonalCalculus' to write.
 * @return the output stream after the writing.
 */
template <typename TP, typename TV>
std::ostream&
operator<< ( std::ostream & out, const PolygonalCalculus<TP,TV> & object )
{
  object.selfDisplay( out );
  return out;
}

} // namespace DGtal
///////////////////////////////////////////////////////////////////////////////
