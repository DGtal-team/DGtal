#pragma once

#include "DGtal/math/linalg/EigenSupport.h"
namespace DGtal
{

  /**
     Description of template class 'SurfaceMeshDEC' <p> \brief Aim:
     Provides methods and operators that builds different variants of
     discrete calculii onto a surface mesh.

   * @tparam TRealPoint an arbitrary model of 3D RealPoint.
     @tparam TRealVector an arbitrary model of 3D RealVector.
  */
  template <typename T, typename EigenLinearAlgebraBackend>
  class SurfaceDEC
  {
    typedef typename EigenLinearAlgebraBackend::DenseVector::Index Index;
    typedef typename EigenLinearAlgebraBackend::DenseMatrix DenseMatrix;
    typedef typename EigenLinearAlgebraBackend::SparseMatrix LinearOperator;

    public:
    DenseMatrix localD0( Index f ) const
    {
      return static_cast<T const *>( this )->buildLocalD0( f );
    }

    LinearOperator D0() const
    {
      return static_cast<T const *>( this )->buildD0();
    }

    DenseMatrix localL0( Index f ) const
    {
      return static_cast<T const *>( this )->buildLocalL0( f );
    }

    LinearOperator L0() const
    {
      return static_cast<T const *>( this )->buildL0();
    }

    DenseMatrix localM0( Index f ) const
    {
      return static_cast<T const *>( this )->buildLocalM0( f );
    }

    LinearOperator M0() const
    {
      return static_cast<T const *>( this )->buildM0();
    }

    LinearOperator lumpedM0() const
    {
      return static_cast<T const *>( this )->buildLumpedM0();
    }

    DenseMatrix localM1( Index f ) const
    {
      return static_cast<T const *>( this )->buildLocalM1( f );
    }

    LinearOperator M1() const
    {
      return static_cast<T const *>( this )->buildM1();
    }

    DenseMatrix localM2( Index f ) const
    {
      return static_cast<T const *>( this )->buildLocalM2( f );
    }

    LinearOperator M2() const
    {
      return static_cast<T const *>( this )->buildM2();
    }

    DenseMatrix localSharp( Index f ) const
    {
      return static_cast<T const *>( this )->buildLocalSharp( f );
    }

    LinearOperator Sharp() const
    {
      return static_cast<T const *>( this )->buildSharp();
    }

    DenseMatrix localFlat( Index f ) const
    {
      return static_cast<T const *>( this )->buildLocalFlat( f );
    }

    LinearOperator Flat() const
    {
      return static_cast<T const *>( this )->buildFlat();
    }
  };

} // namespace DGtal
