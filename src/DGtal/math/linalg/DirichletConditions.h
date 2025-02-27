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
 * @file DirichletConditions.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2022/04/10
 *
 * Header file for module DirichletConditions.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DirichletConditions_RECURSES)
#error Recursive header files inclusion detected in DirichletConditions.h
#else // defined(DirichletConditions_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DirichletConditions_RECURSES

#if !defined DirichletConditions_h
/** Prevents repeated inclusion of headers. */
#define DirichletConditions_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include <DGtal/math/linalg/CDynamicMatrix.h>
#include <DGtal/math/linalg/CDynamicVector.h>
#include <DGtal/math/linalg/CLinearAlgebra.h>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class DirichletConditions
  /**
     Description of template class 'DirichletConditions' <p> \brief Aim:
     A helper class to solve a system with Dirichlet boundary conditions.

     Typically you have a system of the form \f$ A x = b \f$, where
     you wish to set up Dirichlet boundary conditions \f$ u \f$ at
     some places \f$ p \f$ where \f$ p=1 \f$ ( \f$ p=0 \f$ within the
     domain). In this case, you solve a modified smaller system \f$
     A_d x_d = b_d \f$.

     A typical usage is:

     \code 
     typedef DirichletConditions< EigenLinearAlgebraBackend > DC;
     DC::SparseMatrix  A = ...; // the matrix of the linear system
     DC::DenseVector   b = ...; // contains right-hand side and boundary conditions
     // set up boundary locations and values
     DC::IntegerVector p = IntegerVector::Zero( b.rows() );
     DC::DenseVector   u = DenseVector::Zero( b.rows() );
     std::vector<DC::Index>  boundary_nodes  = { 12, 17, 25, 38, ... };
     std::vector<DC::Scalar> boundary_values = { 10.0, 20.0, 15.0, 7.0, ... };
     for ( int i = 0; i < boundary_nodes.size(); i++ )
     { 
        auto j = boundary_nodes[ i ];
        p[ j ] = 1;
        u[ j ] = boundary_values[ i ];
     }
     DC::SparseMatrix A_d = DC::dirichletOperator( A, p );
     DC::DenseVector  b_d = DC::dirichletVector  ( A, b, p, u );
     solver.compute( A_d ); // prefactorization
     ASSERT(solver.info()==Eigen::Success);
     DC::DenseVector x_d = solver.solve( b_d ); // solve Dirichlet system
     DC::DenseVector x   = DC::dirichletSolution( x_d, p, u ); // get whole solution
     \endcode

     It is a model of boost::CopyConstructible,
     boost::DefaultConstructible, boost::Assignable.

     @tparam TLinearAlgebraBackend linear algebra backend used (i.e. EigenLinearAlgebraBackend).
  */
  template < typename TLinearAlgebraBackend >
  requires concepts::CDynamicVector<typename TLinearAlgebraBackend::DenseVector>
  class DirichletConditions
  {
  public:
    typedef TLinearAlgebraBackend LinearAlgebraBackend;

    typedef typename LinearAlgebraBackend::DenseVector::Index  Index;
    typedef typename LinearAlgebraBackend::DenseVector::Scalar Scalar;
    typedef typename LinearAlgebraBackend::DenseVector         DenseVector;
    typedef typename LinearAlgebraBackend::IntegerVector       IntegerVector;
    typedef typename LinearAlgebraBackend::DenseMatrix         DenseMatrix;
    typedef typename LinearAlgebraBackend::SparseMatrix        SparseMatrix;
    typedef typename LinearAlgebraBackend::Triplet             Triplet;

    BOOST_CONCEPT_ASSERT(( concepts::CDynamicMatrix<DenseMatrix> ));
    BOOST_CONCEPT_ASSERT(( concepts::CDynamicMatrix<SparseMatrix> ));
    BOOST_CONCEPT_ASSERT(( concepts::CLinearAlgebra<DenseVector, SparseMatrix> ));
    BOOST_CONCEPT_ASSERT(( concepts::CLinearAlgebra<DenseVector, DenseMatrix> ));

    /// Typically you have a system of the form \f$ A x = b \f$, where
    /// you wish to set up Dirichlet boundary conditions \f$ u \f$ at
    /// some places \f$ p \f$ where \f$ p=1 \f$ ( \f$ p=0 \f$ within the
    /// domain). In this case, you solve a modified smaller system \f$
    /// A_d x_d = b_d \f$.
    ///
    /// @param A the matrix representing the linear operator in Ax=b.
    ///
    /// @param p the vector such that `p[i]=1` whenever the i-th node
    /// is a boundary node, `p[i]=0` otherwise.
    ///
    /// @pre `#row(A) = #col(A) = #col(p)`
    /// @return the (smaller) linear matrix \f$ A_d \f$ to prefactor.
    static 
    SparseMatrix dirichletOperator( const SparseMatrix& A,
                                    const IntegerVector& p )
    {
      ASSERT( A.cols() == A.rows() );
      ASSERT( p.rows() == A.rows() );
      const auto n = p.rows();
      std::vector< Index > relabeling( n );
      Index j = 0;
      for ( Index i = 0; i < p.rows(); i++ )
        relabeling[ i ] = ( p[ i ] == 0 ) ? j++ : n;
      // Building matrix
      SparseMatrix Ap( j, j );
      std::vector< Triplet > triplets;
      for ( int k = 0; k < A.outerSize(); ++k )
        for ( typename SparseMatrix::InnerIterator it( A, k ); it; ++it )
          {
            if ( ( relabeling[ it.row() ] != n ) && ( relabeling[ it.col() ] != n ) )
              triplets.push_back( { relabeling[ it.row() ], relabeling[ it.col() ],
                  it.value() } );
          }
      Ap.setFromTriplets( triplets.cbegin(), triplets.cend() );
      return Ap;
    }

    /// Typically you have a system of the form \f$ A x = b \f$, where
    /// you wish to set up Dirichlet boundary conditions \f$ u \f$ at
    /// some places \f$ p \f$ where \f$ p=1 \f$ ( \f$ p=0 \f$ within the
    /// domain). In this case, you solve a modified smaller system \f$
    /// A_d x_d = b_d \f$.
    ///
    /// @param A the matrix representing the linear operator in \f$ Ax=b \f$.
    /// @param b the vector representing the vector to solve for in \f$ Ax=b \f$.
    /// @param p the vector such that `p[i]=1` whenever the i-th node
    /// is a boundary node, `p[i]=0` otherwise.
    /// @param u the vector giving the constrained Dirichlet values on
    /// places such that `p[i]=1`.
    ///
    /// @pre `#row(A) = #col(A) = #col(p) = #col(b) = #col(u)`
    /// @return the (smaller) form \f$ b_d \f$ to solve for
    static
    DenseVector dirichletVector( const SparseMatrix& A,
                                 const DenseVector& b,
                                 const IntegerVector& p,
                                 const DenseVector& u ) 
    {
      ASSERT( A.cols() == A.rows() );
      ASSERT( p.rows() == A.rows() );
      const auto n = p.rows();
      DenseVector  up = p.array().template cast<double>() * u.array();
      DenseVector tmp = b.array() - (A * up).array();
      std::vector< Index > relabeling( n );
      Index j = 0;
      for ( Index i = 0; i < p.rows(); i++ )
        relabeling[ i ] = ( p[ i ] == 0 ) ? j++ : n;
      DenseVector  bp( j );
      for ( Index i = 0; i < p.rows(); i++ )
        if ( p[ i ] == 0 ) bp[ relabeling[ i ] ] = tmp[ i ];
      return bp;
    }

    /// Typically you have a system of the form \f$ A x = b \f$, where
    /// you wish to set up Dirichlet boundary conditions \f$ u \f$ at
    /// some places \f$ p \f$ where \f$ p=1 \f$ ( \f$ p=0 \f$ within the
    /// domain). In this case, you solve a modified smaller system \f$
    /// A_d x_d = b_d \f$.
    ///
    /// @param xd the solution to the smaller system \f$ A_d x_d = b_d \f$.
    /// @param p the vector such that `p[i]=1` whenever the i-th node
    /// is a boundary node, `p[i]=0` otherwise.
    /// @param u the vector giving the constrained Dirichlet values on
    /// places such that `p[i]=1`.
    /// @return the solution \f$ x \f$ to the original system \f$ A x = b \f$,
    static
    DenseVector dirichletSolution( const DenseVector& xd,
                                   const IntegerVector& p,
                                   const DenseVector& u ) 
    {
      DenseVector  x( p.rows() );
      Index j = 0;
      for ( Index i = 0; i < p.rows(); i++ )
        x[ i ] = ( p[ i ] == 0 ) ? xd[ j++ ] : u[ i ];
      return x;
    }

    /// Utility method to build a null vector that will be useful to
    /// define the boundary characteristic set for Dirichlet
    /// conditions.
    ///
    /// @param b any vector of the size of your problem (e.g., \f$ b
    /// \f$ in your problem \f$ Ax = b \f$)
    ///
    /// @return an integer vector of same size as \a b with zero everywhere.
    static
    IntegerVector nullBoundaryVector( const DenseVector& b )
    {
      return IntegerVector::Zero( b.rows() );
    }
  };

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DirichletConditions_h

#undef DirichletConditions_RECURSES
#endif // else defined(DirichletConditions_RECURSES)
  
