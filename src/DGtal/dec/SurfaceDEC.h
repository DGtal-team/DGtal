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
 * @author
 *
 * @date 2024/06/21
 *
 * Header file for module SurfaceDEC.h
 *
 * This file is part of the DGtal library.
 */

#if !defined SurfaceDEC_h
/** Prevents repeated inclusion of headers. */
#define SurfaceDEC_h

#include "DGtal/math/linalg/EigenSupport.h"
namespace DGtal
{

  /**
     Description of template class 'SurfaceDEC' <p> \brief
     Provides methods and operators definitions for DEC.

   * @tparam T An implementation that also inherit `SurfaceDEC`
     @tparam TLinearAlgebraBackend linear algebra backend used (i.e.
     EigenSparseLinearAlgebraBackend).
  */
  template <typename T, typename TLinearAlgebraBackend>
  class SurfaceDEC
  {
    typedef typename TLinearAlgebraBackend::DenseVector::Index Index;
    typedef typename TLinearAlgebraBackend::DenseMatrix DenseMatrix;
    typedef typename TLinearAlgebraBackend::SparseMatrix LinearOperator;

    public:
    /// @name Global operators
    /// @{

    /// \brief Global differential
    /// @return the n_e x n_v diffential matrix
    LinearOperator D0() const
    {
      return static_cast<T const *>( this )->buildD0();
    }

    /// \brief Global Laplace-Beltrami
    /// @return the n_v x n_v striffness matrix
    LinearOperator L0() const
    {
      return static_cast<T const *>( this )->buildL0();
    }

    /// \brief Global inner product between 0 forms
    ///
    /// For a diagonal version see lumpedM0()
    /// @return the n_v x n_v mass matrix
    LinearOperator M0() const
    {
      return static_cast<T const *>( this )->buildM0();
    }

    /// \brief Global diagonal inner product between 0 forms
    ///
    /// @param f a face
    /// @return the n_v x n_v diagonal mass matrix
    LinearOperator lumpedM0() const
    {
      return static_cast<T const *>( this )->buildLumpedM0();
    }

    /// \brief Global inner product between 1 forms
    /// @return the n_e x n_e mass matrix
    LinearOperator M1() const
    {
      return static_cast<T const *>( this )->buildM1();
    }

    /// \brief Global inner product between 2 forms
    /// @return the n_f x n_f mass matrix
    LinearOperator M2() const
    {
      return static_cast<T const *>( this )->buildM2();
    }

    /// \brief Global Sharp operator
    /// @return a 3*n_f x n_e matrix
    LinearOperator Sharp() const
    {
      return static_cast<T const *>( this )->buildSharp();
    }

    /// \brief Global Flat operator
    /// @return a n_e x 3*n_f matrix
    LinearOperator Flat() const
    {
      return static_cast<T const *>( this )->buildFlat();
    }

    /// @}

    /// @name Per face operators
    /// @{

    /// \brief Differential inside a face
    /// @param f a face
    /// @return the degree x degree differential matrix
    DenseMatrix localD0( Index f ) const
    {
      return static_cast<T const *>( this )->buildLocalD0( f );
    }

    /// \brief Laplace-Beltrami inside a face
    /// @param f a face
    /// @return the degree x degree stiffness matrix
    DenseMatrix localL0( Index f ) const
    {
      return static_cast<T const *>( this )->buildLocalL0( f );
    }

    /// \brief Inner product between 0 forms inside a face
    /// @param f a face
    /// @return the degree x degree mass matrix
    DenseMatrix localM0( Index f ) const
    {
      return static_cast<T const *>( this )->buildLocalM0( f );
    }

    /// \brief Inner product between 1 forms inside a face
    /// @param f a face
    /// @return the degree x degree mass matrix
    DenseMatrix localM1( Index f ) const
    {
      return static_cast<T const *>( this )->buildLocalM1( f );
    }

    /// \brief Inner product between 2 forms inside a face
    /// @param f a face
    /// @return the 1 x 1 mass matrix
    DenseMatrix localM2( Index f ) const
    {
      return static_cast<T const *>( this )->buildLocalM2( f );
    }

    /// \brief Sharp operator for inside a face
    /// @param f a face
    /// @return a 3 x degree matrix
    DenseMatrix localSharp( Index f ) const
    {
      return static_cast<T const *>( this )->buildLocalSharp( f );
    }

    /// \brief Flat operator inside a face
    /// @param f a face
    /// @return a degree x 3 matrix
    DenseMatrix localFlat( Index f ) const
    {
      return static_cast<T const *>( this )->buildLocalFlat( f );
    }

    /// @}
  };

} // namespace DGtal

#endif
