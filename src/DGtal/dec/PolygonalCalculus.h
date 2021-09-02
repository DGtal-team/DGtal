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

#if defined(PolygonalCalculus_RECURSES)
#error Recursive header files inclusion detected in PolygonalCalculus.h
#else // defined(PolygonalCalculus_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PolygonalCalculus_RECURSES

#if !defined PolygonalCalculus_h
/** Prevents repeated inclusion of headers. */
#define PolygonalCalculus_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <functional>
#include "DGtal/base/ConstAlias.h"
#include "DGtal/base/Common.h"
#include "DGtal/math/linalg/EigenSupport.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// template class PolygonalCalculus
/**
 * Description of template class 'PolygonalCalculus' <p>
 * \brief Aim: Implements differential operators on polygonal surfaces.
 *
 *
 *  E.g.
 *
 *  PolygonalCalculus polydec(mesh);
 *  auto Grad = polydec.gradient();
 *  auto Grad_f  = polydec.gradient(face)
 */
template <typename TSurfaceMesh>
class PolygonalCalculus
{
  // ----------------------- Standard services ------------------------------
public:
  
  ///Type of SurfaceMesh
  typedef TSurfaceMesh SurfaceMesh;
  
  ///Vertex type
  typedef typename TSurfaceMesh::Vertex Vertex;
  
  ///Face type
  typedef typename TSurfaceMesh::Face Face;
  
  ///Face type
  typedef typename TSurfaceMesh::RealPoint RealPoint;
  
  ///Linear Algebra Backend from Eigen
  typedef EigenLinearAlgebraBackend LinAlg;
  ///Type of Vector
    typedef LinAlg::DenseVector Vector;
  ///Type of dense matrix
  typedef LinAlg::DenseMatrix DenseMatrix;
  ///Type of sparse matrix
  typedef LinAlg::SparseMatrix SparseMatrix;
  

  /// Create a Polygonal DEC structure from a surface mesh (@a surf)
  /// using an default identity embedder.
  /// @param surf an instance of SurfaceMesh
  PolygonalCalculus(const ConstAlias<TSurfaceMesh> surf): mySurfaceMesh(&surf)
  {
    myEmbedder =[&](Face f,Vertex v){ return mySurfaceMesh->position(v);};
    init();
  };
  
  /// Create a Polygonal DEC structure from a surface mesh (@a surf)
  /// and an embedder for the vertex position: function with two parameters, a face and a vertex
  /// which outputs the embedding in R^3 of the vertex w.r.t. to the face.
  /// @param surf an instance of SurfaceMesh
  /// @param embedder an embedder
  PolygonalCalculus(const ConstAlias<TSurfaceMesh> surf,
               const std::function<RealPoint(Face,Vertex)> &embedder):
  mySurfaceMesh(&surf), myEmbedder(embedder)
  {
    init();
  };
  
  /**
   * Default constructor.
   */
  PolygonalCalculus() = delete;
  
  /**
   * Destructor.
   */
  ~PolygonalCalculus() = default;
  
  /**
   * Copy constructor.
   * @param other the object to clone.
   */
  PolygonalCalculus ( const PolygonalCalculus & other ) = delete;
  
  /**
   * Move constructor.
   * @param other the object to move.
   */
  PolygonalCalculus ( PolygonalCalculus && other ) = delete;
  
  /**
   * Copy assignment operator.
   * @param other the object to copy.
   * @return a reference on 'this'.
   */
  PolygonalCalculus & operator= ( const PolygonalCalculus & other ) = delete;
  
  /**
   * Move assignment operator.
   * @param other the object to move.
   * @return a reference on 'this'.
   */
  PolygonalCalculus & operator= ( PolygonalCalculus && other ) = delete;
  
  // ----------------------- Interface --------------------------------------
  
  
  /// Update the embedding function.
  /// @param externalFunctor a new embedding functor (Face,Vertex)->RealPoint.
  void setEmbedder(const std::function<PolygonalCalculus<TSurfaceMesh>::RealPoint(PolygonalCalculus<TSurfaceMesh>::Face,
                                                                                  PolygonalCalculus<TSurfaceMesh>::Vertex)> &externalFunctor)
  {
    myEmbedder = externalFunctor;
  }
  
  // ----------------------- Per face operators --------------------------------------
  
  /// Return the vertex position matrix degree x 3 of a face
  /// @param f a face
  /// @return the n_f x 3 position matrix
  DenseMatrix X(const Face f) const;
  

  /// Derivative operator (d_0) of a face.
  /// @param f the face
  /// @return a degree x degree matrix
  DenseMatrix D(const Face f) const;
  
  /// Average operator to average, per edge, its vertex values.
  /// @param f the face
  /// @return a degree x degree matrix
  DenseMatrix A(const Face f) const;
  
  
  /// Vector area per face
  /// @param f the face
  /// @return a vector
  Vector vectorArea(const Face f) const;
  
  /// Area of a face from the vector area
  /// @param f the face
  /// @return the corrected area of the face
  double correctedFaceArea(const Face f) const
  {
    return vectorArea(f).norm();
  }
  
  /// Corrected normal vector of a face.
  /// @param f the face
  /// @return a vector
  Vector correctedFaceNormal(const Face f) const
  {
    Vector v = vectorArea(f);
    v.normalize();
    return v;
  }
  
  /// Operator of edge vectors per face
  /// @param f the face
  /// @return degree x 3 matrix
  DenseMatrix E(const Face f) const
  {
    return D(f)*X(f);
  }
  
  /// co-Gradient operator of the face
  /// @param f the face
  /// @param a 3 x degree matrix
  DenseMatrix coGradient(const Face f) const
  {
    return  E(f).transpose() * A(f);
  }
  
  /// Gradient operator of the face
  /// @param f the face
  /// @return 3 x degree matrix
  DenseMatrix Gradient(const Face f) const
  {
    return -1.0/areaFace(f) * bracket( normalFace(f) ) * coG(f);
  }
  
  //Flat
  /// Flat operator for the face
  /// @param f the face
  /// @return a degree x 3 matrix
  DenseMatrix  V(const Face f) const
  {
    auto n = correctedFaceNormal(f);
    return E(f)*( DenseMatrix::Identity(3,3) - n*n.transpose());
  }
  
  //Edge midPoints
  /// Edge mid-point operator of the face
  /// @param f the face
  /// @return a degree x 3 matrix
  DenseMatrix B(const Face f) const
  {
    return A(f) * X(f);
  }
  
  /// @returns the centroid of the face
  /// @param f the face
  Vector centroid(const Face f) const
  {
    auto nf = myFaceDegree[f];
    return 1/(double)nf * X(f).transpose() * DenseMatrix::Ones(nf);
  }
  
  /// Sharp operator for the face
  /// @param f the face
  /// @return a 3 x degree matrix
  DenseMatrix U(const Face f) const
  {
    auto nf = myFaceDegree[f];
    return 1/correctedFaceArea(f) * bracket(correctedFaceNormal(f)) * ( B(f).transpose() - centroid(f)* DenseMatrix::Ones(nf).transpose() );
  }
  
  /// Projection operator for the face
  /// @param f the face
  /// @return a degree x degree matrix
  DenseMatrix P(const Face f) const
  {
    auto nf = myFaceDegree[f];
    return DenseMatrix::Identity(nf,nf) - V(f)*U(f);
  }
  
  /// Mass operator associated with the face
  /// @param f the face
  /// @param lambda the regularization parameter
  /// @return a degree x degree matrix
  DenseMatrix M(const Face f, const double lambda=1.0) const
  {
    return correctedFaceArea(f) * U(f).transpose()*U(f) + lambda * P(f).transpose()*P(f);
  }
  
  /// (weak) Laplace-Beltrami operator for the face
  /// @param f the face
  /// @param lambda the regularization parameter
  /// @return a degree x degree matrix
  DenseMatrix LaplaceBeltrami(const Face f, const double lambda=1.0) const
  {
    return D(f).transpose() * M(f,lambda) * D(f);
  }
  
  // ----------------------- Common --------------------------------------
public:
  
  /// Update the internal cache structures
  /// (e.g. degree of each face).
  void init()
  {
    updateFaceDegree();
  }
  
  
  /**
   * Writes/Displays the object on an output stream.
   * @param out the output stream where the object is written.
   */
  void selfDisplay ( std::ostream & out ) const;
  
  /**
   * Checks the validity/consistency of the object.
   * @return 'true' if the object is valid, 'false' otherwise.
   */
  bool isValid() const;
  
  // ------------------------- Protected Datas ------------------------------
protected:
  
  /// Update the face degree cache
  void updateFaceDegree()
  {
    myFaceDegree.resize(mySurfaceMesh->nbFaces());
    for(auto f = 0; f <  mySurfaceMesh->nbFaces(); ++f)
    {
      auto vertices = mySurfaceMesh->incidentVertices(f);
      auto nf = vertices.size();
      myFaceDegree[f] = nf;
    }
  }
  
  // ------------------------- Internals ------------------------------------
private:
  
  ///Underlying SurfaceMesh
  const TSurfaceMesh *mySurfaceMesh;
  
  ///Embedding function (face,vertex)->R^3 for the vertex position wrt. the face.
  std::function<RealPoint( Face, Vertex)> myEmbedder;
  
  ///Cache containing the face degree
  std::vector<size_t> myFaceDegree;
    
  
}; // end of class PolygonalCalculus


/**
 * Overloads 'operator<<' for displaying objects of class 'PolygonalCalculus'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'PolygonalCalculus' to write.
 * @return the output stream after the writing.
 */
template <typename T>
std::ostream&
operator<< ( std::ostream & out, const PolygonalCalculus<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/dec/PolygonalCalculus.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PolygonalCalculus_h

#undef PolygonalCalculus_RECURSES
#endif // else defined(PolygonalCalculus_RECURSES)
