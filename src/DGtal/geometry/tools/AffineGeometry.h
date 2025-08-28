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
 * @file AffineGeometry.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2025/08/22
 *
 * Header file for module AffineGeometry.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(AffineGeometry_RECURSES)
#error Recursive header files inclusion detected in AffineGeometry.h
#else // defined(AffineGeometry_RECURSES)
/** Prevents recursive inclusion of headers. */
#define AffineGeometry_RECURSES

#if !defined AffineGeometry_h
/** Prevents repeated inclusion of headers. */
#define AffineGeometry_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/arithmetic/IntegerComputer.h"
#include "DGtal/math/linalg/SimpleMatrix.h"

namespace DGtal
{
  // Forward declaration of AffineGeometry (needed for friend declaration).
  template < typename T > struct AffineGeometry;

  namespace detail {

    /////////////////////////////////////////////////////////////////////////////
    // template class AffineGeometryPointOperations

    /// Description of template class 'AffineGeometryPointOperations' <p>
    /// \brief Aim: Internal class used by AffineGeometry to
    /// differentiate operations on lattice points and operations on
    /// points with floating-point coordinates. 
    ///
    /// @tparam TScalar any integer or floating point number type.
    template < typename TPoint >
    struct AffineGeometryPointOperations
    {
      template <typename T> friend struct DGtal::AffineGeometry;
      typedef TPoint                     Point;
      typedef typename Point::Coordinate Scalar;
      /// In the generic class, the type scalar should be an integral type.
      typedef Scalar Integer; 

      // ----------------------- internal services --------------------------
    private:
      /// @name static internal services
      /// @{

      /// Generic method to normalize a vector.
      /// By default, assume it is an integral type.
      template <typename TInteger >
      static void normalizeVector( Point& w, TInteger )
      {
        DGtal::Dimension i = 0;
        while ( i < Point::dimension && w[ i ] == 0 ) i++;
        if ( i == Point::dimension ) return;
        TInteger g = abs( w[ i ] );
        for ( ; i < Point::dimension; i++ )
          g = DGtal::IntegerComputer< TInteger >::staticGcd( g, abs( w[ i ] ) );
        w /= g;
      }

      /// Specialized version to normalize a vector in case of double
      /// value parameter.
      static
      void normalizeVector( Point& w, double x )
      {
        w /= x;
      }
      
      /// Specialized version to normalize a vector in case of float
      /// value parameter.
      static
      void normalizeVector( Point& w, float x )
      {
        w /= x;
      }
      
    }; // end of class AffineGeometryPointOperations

    

    /////////////////////////////////////////////////////////////////////////////
    // template class AffineGeometryScalarOperations

    /// Description of template class 'AffineGeometryScalarOperations'
    /// <p> \brief Aim: Internal class used by AffineGeometry to
    /// differentiate operations on point coordinates, which may be
    /// integer or floating-point numbers.. The generic class assume
    /// integer coordinates, while there are two specializations for
    /// float and double.
    ///
    /// @tparam TScalar any integer or floating point number type.
    template < typename TScalar >
    struct AffineGeometryScalarOperations
    {
      template <typename T> friend struct DGtal::AffineGeometry;

      typedef TScalar Scalar;
      /// In the generic class, the type scalar should be an integral type.
      typedef Scalar Integer; 
      
      // ----------------------- internal services --------------------------
    private:
      /// @name static internal services
      /// @{

      /// @param[in] a any integer number
      /// @param[in] b any integer number
      ///
      /// @return the pair ( b/g, a/g ), where g is gcd(a,b), which
      /// allows to cancel a component in a Gauss pivoting algorithm.
      static
      std::pair< Integer, Integer > getMultipliers( Integer a, Integer b )
      {
        Integer g = IntegerComputer< Integer >::staticGcd( abs( a ), abs( b ) );
        return std::make_pair( b / g, a / g );
      }

      /// @param[in] x any integer number
      ///
      /// @return 'true' iff x is non zero.
      static
      bool isNonZero( Integer x, double )
      {
        return x != Integer( 0 );
      }

    }; // end of class AffineGeometryScalarOperations

    /// Description of template class 'AffineGeometryScalarOperations' <p>
    /// \brief Aim: Internal class used by AffineGeometry to
    /// differentiate operations on lattice points and operations on
    /// points with floating-point coordinates. This specialization
    /// assume double for coordinates.
    template <>
    struct AffineGeometryScalarOperations< double >
    {
      template <typename T> friend struct DGtal::AffineGeometry;
    
      /// @param[in] a any number
      /// @param[in] b any number
      ///
      /// @return the pair ( b, a ), which allows to cancel a
      /// component in a Gauss pivoting algorithm.
      static
      std::pair< double, double > getMultipliers( double a, double b )
      {
        return std::make_pair( b, a );
      }

      /// @param[in] x any  number
      ///
      /// @param[in] tol the accepted tolerance value below which the
      /// number is considered null (typically 1e-12).
      ///
      /// @return 'true' iff x is non zero.
      static
      bool isNonZero( double x, double tol )
      {
        return ( x > tol ) || ( x < -tol );
      }

    }; // end of class AffineGeometryScalarOperations< double >

    /// Description of template class 'AffineGeometryScalarOperations' <p>
    /// \brief Aim: Internal class used by AffineGeometry to
    /// differentiate operations on lattice points and operations on
    /// points with floating-point coordinates. This specialization
    /// assume float for coordinates.
    template <>
    struct AffineGeometryScalarOperations< float >
    {
      template <typename T> friend struct DGtal::AffineGeometry;
    
      /// @param[in] a any number
      /// @param[in] b any number
      ///
      /// @return the pair ( b, a ), which allows to cancel a
      /// component in a Gauss pivoting algorithm.
      static
      std::pair< float, float > getMultipliers( float a, float b )
      {
        return std::make_pair( b, a );
      }

      /// @param[in] x any  number
      ///
      /// @param[in] tol the accepted tolerance value below which the
      /// number is considered null (typically 1e-12).
      ///
      /// @return 'true' iff x is non zero.
      static
      bool isNonZero( float x, double tol )
      {
        const double dx = double(x);
        return (dx > tol) || ( dx < -tol );
      }
      
    }; // end of class AffineGeometryScalarOperations< float >

    template < typename TScalar, bool Robust >
    struct AffineGeometryInternalNumber {
      typedef TScalar type; 
    };
    template <>
    struct AffineGeometryInternalNumber<int32_t, false> {
      typedef int64_t type; 
    };
    template <>
    struct AffineGeometryInternalNumber<int32_t, true> {
      typedef BigInteger type; 
    };
    template <>
    struct AffineGeometryInternalNumber<int64_t, false> {
      typedef int64_t type; 
    };
    template <>
    struct AffineGeometryInternalNumber<int64_t, true> {
      typedef BigInteger type; 
    };
    template <>
    struct AffineGeometryInternalNumber<float, false> {
      typedef double type; 
    };
    template <>
    struct AffineGeometryInternalNumber<float, true> {
      typedef double type; 
    };
    template <>
    struct AffineGeometryInternalNumber<double, false> {
      typedef double type; 
    };
    template <>
    struct AffineGeometryInternalNumber<double, true> {
      typedef double type; 
    };
    
  } // namespace detail

} // namespace DGtal


namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class AffineGeometry

  /// Description of template class 'AffineGeometry' <p> \brief Aim:
  /// Utility class to determine the affine geometry of an input set
  /// of points. It provides exact results when the input is composed
  /// of lattice points, and may determine a basis, the dimension, or
  /// an orthogonal vector.
  ///
  /// @note Useful for algorithms that requires the exact
  /// dimensionality of a set of points before processing, like
  /// QuickHull.
  ///
  /// @warning You can use this class to test the dimensionality of a
  /// set of points with floating point coordinates, but this approach
  /// is less robust than singular value decomposition.
  ///
  /// @tparam TPoint the type for points, which may be lattice points
  /// or points with floating-point coordinates.
  ///
  /// @code
  /// #include <iostream>
  /// #include <vector>
  /// #include "DGtal/base/Common.h"
  /// #include "DGtal/kernel/SpaceND.h"
  /// #include "DGtal/geometry/tools/AffineGeometry.h"
  /// ...
  /// typedef SpaceND< 3, int >                Space;      
  /// typedef Space::Point                     Point;
  /// typedef AffineGeometry< Point >            Affine;
  /// std::vector<Point> X = { Point{1, 0, 0}, Point{2, 1, 0}, Point{3, 2, 0}, Point{3, 1, 1}, Point{5, 2, 2}, Point{4, 2, 1} };
  /// auto I = Affine::affineSubset( X ); /// 3 points with indices (0,1,3)
  /// auto B = Affine::affineBasis( X ); /// (1,0,0) and 2 basis vectors (1,1,0) and (2,1,1).
  /// auto d = Affine::affineDimension( X ); /// 2
  /// @endcode
  ///
  /// @see testAffineGeometry.cpp
  template < typename TPoint >
  struct AffineGeometry
  {
    typedef TPoint                     Point;
    typedef typename Point::Coordinate Scalar;
    typedef std::size_t                Size;
    typedef std::vector< Point >       Points; ///< type for range of points.
    typedef std::vector< Size >        Sizes;  ///< type for range of sizes.
    static const Size  dimension  = Point::dimension;
    typedef DGtal::detail::AffineGeometryPointOperations< Point >   PointOps;
    typedef DGtal::detail::AffineGeometryScalarOperations< Scalar > ScalarOps;
    
    // ----------------------- standard services --------------------------
  public:
    /// @name static affine services
    /// @{

    /// Given a range of points \a X, returns the affine dimension of
    /// its spanned affine subspace.
    ///
    /// @param[in] X the range of input points (may be lattice points or not).
    ///
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return the affine dimension of \a X
    static
    DGtal::int64_t affineDimension( const Points& X, const double tolerance = 1e-12 )
    {
      return DGtal::int64_t( affineSubset( X, tolerance ).size() ) - 1;
    }
    
    /// Given a range of points \a X, returns a subset of these points
    /// that form an affine basis of \a X. Equivalently it is a
    /// simplex whose affine space spans all the points of \a X.
    ///
    /// @param[in] X the range of input points (may be lattice points or not).
    ///
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return a subset of these points as a range of indices.
    ///
    /// @note Complexity is \f$O( m n^2 )\f$, where m=Cardinal(X) and n=dimension.
    static
    std::vector< Size > affineSubset( const Points& X, const double tolerance = 1e-12 )
    {
      Size m = X.size();
      // Process trivial cases.
      if ( m == 0 ) return { };
      if ( m == 1 ) return { Size( 0 ) };
      // Process general case.
      Points basis;  //< direction vectors
      Sizes  chosen; //< selected points
      chosen.push_back( 0 ); //< reference point (first one, as it may be any one)
      for ( Size i = 1; i < m; i++ )
        {
          Point v = X[ i ] - X[ 0 ];
          if ( addIfIndependent( basis, v, tolerance ) )
            chosen.push_back( i );
          if ( chosen.size() > dimension ) break;
        }
      return chosen;
    }

    /// Given a range of points \a X, returns a point and a range of
    /// vectors forming an affine basis containing \a X.
    ///
    /// @param[in] X the range of input points (may be lattice points or not).
    ///
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return a point and a range of vectors forming an affine basis containing X.
    ///
    /// @note Complexity is O( m n^2 ), where m=Cardinal(X) and n=dimension.
    static
    std::pair< Point, Points > affineBasis( const Points& X, const double tolerance = 1e-12 )
    {
      std::vector< Size > indices = affineSubset( X, tolerance );
      Points basis( indices.size() - 1 );
      for ( Size i = 0; i < basis.size(); i++ )
        basis[ i ] = X[ indices[ i+1 ] ] - X[ indices[ 0 ] ];
      return std::make_pair( X[ indices[ 0 ] ], basis );
    }

    /// Given a partial basis of vectors, returns a new vector that is independent.
    ///
    /// @param[in] basis a range of independent vectors that defines a
    /// partial basis of the space.
    ///
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return a canonic unit vector independent of all vectors of \a
    /// basis, or the null vector if the basis was not partial.
    static
    Point independentVector( const Points& basis, const double tolerance = 1e-12 )
    {
      // If basis has already d independent vectors, then there is no
      // other independent vector.
      if ( basis.size() >= dimension ) return Point::zero;
      // At least one trivial canonic vector should be independant.
      Dimension k = 0;
      for ( ; k < dimension; k++ )
        {
          Point e_k = Point::base( k );
          Point w   = reductionOnBasis( e_k, basis, tolerance );
          if ( ScalarOps::isNonZero( w.normInfinity(), tolerance ) )
            return e_k;
        }
      trace.error() << "[AffineGeometry::independentVector]"
                    << " Unable to find independent vector." << std::endl;
      return Point::zero;
    }

    /// Complete the vectors \a basis with independent vectors so as
    /// to form a basis of the space. The last added vector is
    /// guaranteed to be \b orthogonal to all the previous vectors.
    ///
    /// @note In 3D, given two independent vectors as input, then the
    /// added vector is the \b cross \b product of these two
    /// vectors. In nD, it is thus a generalization of the cross
    /// product.
    ///
    /// @param[in,out] basis a range of independent vectors of size less than dimension.
    ///
    /// @param safe when 'true' uses a safer internal number type for computations.
    ///
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    static
    void completeBasis( Points& basis, bool safe = true, const double tolerance = 1e-12 )
    {
      if ( basis.size() >= dimension ) return;
      while ( ( basis.size() + 1 ) < dimension )
        { // add an independent vector
          const Point u = independentVector( basis, tolerance );
          basis.push_back( u );
        }
      // Use cofactors to determine normal vectors.
      Point p;
      if ( safe )
        {
          typedef typename DGtal::detail::AffineGeometryInternalNumber< Scalar, true >::type
            InternalNumber;
          std::vector<InternalNumber> u = orthogonalVector<InternalNumber>( basis );
          p = convertToPoint( u );
        }
      else
        {
          typedef typename DGtal::detail::AffineGeometryInternalNumber< Scalar, false >::type
            InternalNumber;
          std::vector<InternalNumber> u = orthogonalVector<InternalNumber>( basis );
          p = convertToPoint( u );
        }
      PointOps::normalizeVector( p, (Scalar) p.normInfinity() );
      basis.push_back( p );
    }
    
    /// @}

    // ----------------------- specific services --------------------------
  public:
    /// @name static specific services
    /// (Used internally)
    /// @{

    /// Reduces the vector \a v on the (partial or not) basis of vectors \a basis.
    ///
    /// @param v any vector
    /// @param basis a range of vectors forming a (partial or not) basis of the space.
    /// @param tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return the part of \a v that cannot be expressed as a linear
    /// combination of vectors of \a basis, and hence a null vector if
    /// \a v is a linear combination of the vectors of the basis.
    static
    Point reductionOnBasis( const Point& v, const Points& basis,
                            const double tolerance )
    {
      Point w( v );
      for ( const auto& b : basis ) reduceVector( w, b, tolerance );
      return w;
    }

    /// Checks if the vector \a v can be decomposed as a linear
    /// combination of the vectors of the basis. If yes, returns
    /// 'false', otherwise returns 'true' and adds the obtained
    /// reduced vector to the basis after normalizing it.
    ///
    /// @param[in,out] basis a range of vectors forming a (partial or
    /// not) basis of the space.
    ///
    /// @param v any vector.
    ///
    /// @param tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return 'true' iff the vector \a v was not a linear
    /// combination of the vectors of the basis and the basis is then
    /// enriched by the direction indicated by \a v, otherwise returns 'false'.
    ///
    /// @note If the points have integer coordinates, the
    /// normalization of the new basis vector is its reduction by its
    /// gcd, otherwise the maximum absolute component value is
    /// normalized to 1.0.
    static
    bool addIfIndependent( Points& basis, const Point& v, const double tolerance )
    {
      Point        w = reductionOnBasis( v, basis, tolerance );
      const Scalar x = w.normInfinity(); 
      if ( ScalarOps::isNonZero( x, tolerance ) )
        {
          // Useful to reduce the norm of vectors for lattice vectors
          // and necessary for real vectors so that `tolerance` keeps
          // the same meaning.
          PointOps::normalizeVector( w, x ); 
          basis.push_back( w );
          return true;
        }
      return false;
    }
    
    /// Reduces vector \a w by the vector \a b
    ///
    /// @param[in,out] w any vector
    /// @param[in] b a non-null vector of the current basis.
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @note This reduction is an elementary Gauss elimination.
    static
    void reduceVector( Point& w, const Point& b, const double tolerance )
    {
      Size n = w.size();
      // Find index of first non null pivot in b.
      Size lead = n;
      for ( Size j = 0; j < n; j++)
        if ( ScalarOps::isNonZero( b[j], tolerance ) ) { lead = j; break; }
      if ( lead == n ) return; // b is null vector

      Scalar mul_w, mul_b;
      std::tie( mul_w, mul_b ) = ScalarOps::getMultipliers( w[ lead ], b[ lead ] );
      
      for (Size j = 0; j < n; j++) 
        w[j] = mul_w * w[j] - mul_b * b[j];
    }

    /// Converts a range of coefficients into a Point of the space.
    ///
    /// @tparam TInternalNumber the number type used for coefficients.
    ///
    /// @param[in] w any vector represented as a range of number.
    /// @return the conversion of \a w to the type Point. 
    template < typename TInternalNumber >
    static
    Point convertToPoint( const std::vector<TInternalNumber>& w )
    {
      Point u;
      std::size_t k = std::min( w.size(), std::size_t( dimension ) );
      for ( std::size_t i = 0; i < k; i++ )
        u[ i ] = (Scalar) w[ i ];
      return u;
    }

    /// Given `d-1` independent vectors in dD, returns a vector that
    /// is orthogonal to each of them.
    ///
    /// @note In 3D, given two independent vectors as input, then the
    /// added vector is the \b cross \b product of these two
    /// vectors. In nD, it is thus a generalization of the cross
    /// product.
    ///
    /// @tparam TInternalNumber the number type used for internal computations.
    ///
    /// @param[in] basis a range of independent vectors of size dimension-1.
    ///
    /// @return a vector of coefficients (represented with the given number type).
    template < typename TInternalNumber >
    static
    std::vector<TInternalNumber>
    orthogonalVector( const Points& basis )
    {
      ASSERT( ( basis.size() + 1 ) == dimension );
      const std::size_t n = dimension;
      const std::size_t m = basis.size();
      SimpleMatrix< TInternalNumber, dimension-1, dimension> A;
      for ( std::size_t i = 0; i < m; ++i )
        for ( std::size_t j = 0; j < n; ++j )
          A( i, j ) = TInternalNumber( basis[ i ][ j ] );
      std::vector<TInternalNumber> w( n );
      for ( std::size_t col = 0; col < n; ++col)
        { // construct sub-matrix removing column col
          SimpleMatrix< TInternalNumber, dimension-1, dimension-1> M;
          for ( std::size_t i = 0; i < m; ++i )
            {
              std::size_t c = 0;
              for ( std::size_t j = 0; j < n; ++j)
                if ( j != col ) M( i, c++ ) = A( i, j );
            }
          functions::determinantBareiss( M, w[ col ] );
          if ( (col+dimension) % 2 == 0 ) w[ col ] = -w[ col ];
        }
      return w;
    }

    /// Given a vector, returns the aligned vector with its component
    /// simplified by the gcd of all components (when the components
    /// are integers) or the aligned vector with a maximum oo-norm of
    /// 1 (when the components are floating-point numbers).
    ///
    /// @param[in] v any vector.
    ///
    /// @return a simplified vector aligned with \a v.
    static 
    TPoint simplifiedVector( TPoint v )
    {
      PointOps::normalizeVector( v, (Scalar) v.normInfinity() );
      return v;
    }
    
    /// @}
    
  };

  namespace functions {
    
    /// Given a range of points \a X, returns the affine dimension of
    /// its spanned affine subspace.
    ///
    /// @tparam TPoint any type of lattice point or real point.
    ///
    /// @param X the range of input points (may be lattice points or not).
    ///
    /// @param tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return the affine dimension of \a X.
    ///
    /// @note Complexity is \f$O( m n^2 )\f$, where m=Cardinal(X) and n=dimension.
    template <typename TPoint>
    DGtal::int64_t
    computeAffineDimension( const std::vector< TPoint >& X, const double tolerance = 1e-12 )
    {
      return AffineGeometry<TPoint>::affineDimension( X, tolerance );
    }

    /// Given a range of points \a X, returns a subset of these points
    /// that form an affine basis of \a X. Equivalently it is a
    /// simplex whose affine space spans all the points of \a X.
    ///
    /// @tparam TPoint any type of lattice point or real point.
    ///
    /// @param X the range of input points (may be lattice points or not).
    ///
    /// @param tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return a subset of these points as a range of indices.
    ///
    /// @note Complexity is \f$O( m n^2 )\f$, where m=Cardinal(X) and n=dimension.
    template <typename TPoint>
    std::vector< std::size_t >
    computeAffineSubset( const std::vector< TPoint >& X, const double tolerance = 1e-12 )
    {
      return AffineGeometry<TPoint>::affineSubset( X, tolerance );
    }
  
    /// Given a range of points \a X, returns a point and a range of
    /// vectors forming an affine basis containing \a X.
    ///
    /// @tparam TPoint any type of lattice point or real point.
    ///
    /// @param X the range of input points (may be lattice points or not).
    ///
    /// @param tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return a point and a range of vectors forming an affine basis containing \a X.
    ///
    /// @note Complexity is O( m n^2 ), where m=Cardinal(X) and n=dimension.
    template <typename TPoint>
    std::pair< TPoint, std::vector< TPoint > >
    computeAffineBasis( const std::vector< TPoint >& X, const double tolerance = 1e-12 )
    {
      return AffineGeometry<TPoint>::affineBasis( X, tolerance );
    }

    /// Given a range of points \a X and the indices \a I of points in
    /// \a X which form an affine subset of \a X, returns a point and
    /// a range of vectors forming an affine basis containing \a X.
    ///
    /// @tparam TPoint any type of lattice point or real point.
    ///
    /// @tparam TIndexRange any type of range of indices, like
    /// std::vector<std::size_t> or std::array<std::size_t,N>.
    ///
    /// @param X the range of input points (may be lattice points or not).
    ///
    /// @param I a subset of these points as a range of indices, forming
    /// an affine subset of \a X, see computeAffineGeometry.
    ///
    /// @return a point and a range of vectors forming an affine basis containing \a X.
    ///
    /// @note Complexity is O( m n^2 ), where m=Cardinal(X) and n=dimension.
    template <typename TPoint, typename TIndexRange>
    std::pair< TPoint, std::vector< TPoint > >
    computeAffineBasis( const std::vector< TPoint >& X,
                        const TIndexRange& I )
    {
      std::vector< TPoint > basis( I.size() - 1 );
      for ( std::size_t i = 0; i < basis.size(); i++ )
        basis[ i ] = X[ I[ i+1 ] ] - X[ I[ 0 ] ];
      return std::make_pair( X[ I[ 0 ] ], basis );
    }

    /// Given a partial basis of vectors, returns a new vector that is independent.
    ///
    /// @tparam TPoint any type of lattice point or real point.
    ///
    /// @param[in] basis a range of independent vectors that defines a
    /// partial basis of the space.
    ///
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return a canonic unit vector independent of all vectors of \a
    /// basis, or the null vector if the basis was not partial.
    template <typename TPoint>
    TPoint
    computeIndependentVector( const std::vector< TPoint >& basis,
                              const double tolerance = 1e-12 )
    {
      return AffineGeometry<TPoint>::independentVector( basis, tolerance );
    }

    /// Complete the vectors \a basis with independent vectors so as
    /// to form a basis of the space. The last added vector is
    /// guaranteed to be \b orthogonal to all the previous vectors.
    ///
    /// @note In 3D, given two independent vectors as input, then the
    /// added vector is the \b cross \b product of these two
    /// vectors. In nD, it is thus a generalization of the cross
    /// product.
    ///
    /// @tparam TPoint any type of lattice point or real point.
    ///
    /// @param[in,out] basis a range of independent vectors of size less than dimension.
    ///
    /// @param safe when 'true' uses a safer internal number type for computations.
    ///
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    template <typename TPoint>
    static
    void
    computeCompleteBasis( std::vector< TPoint >& basis,
                   bool safe = true,
                   const double tolerance = 1e-12 )
    {
      AffineGeometry<TPoint>::completeBasis( basis, safe, tolerance );
    }

    /// Given `d-1` independent vectors in dD, returns a vector that
    /// is orthogonal to each of them.
    ///
    /// @note In 3D, given two independent vectors as input, then the
    /// added vector is the \b cross \b product of these two
    /// vectors. In nD, it is thus a generalization of the cross
    /// product.
    ///
    /// @tparam TPoint any type of lattice point or real point.
    ///
    /// @tparam TInternalVector the type of vector used for internal
    /// computations and outputing the result, e.g. some PointVector.
    ///
    /// @param[out] w the returned orthogonal vector to every vector of basis.
    ///
    /// @param[in] basis a range of independent vectors of size dimension-1.
    template < typename TPoint, typename TInternalVector >
    static
    void
    computeOrthogonalVector( TInternalVector& w, const std::vector<TPoint>& basis )
    {
      typedef typename TInternalVector::Component TInternalNumber;
      ASSERT( TPoint::dimension == TInternalVector::dimension );
      ASSERT( ( basis.size() + 1 ) == TInternalVector::dimension );
      constexpr std::size_t n = TInternalVector::dimension;
      const std::size_t m = basis.size();
      SimpleMatrix< TInternalNumber, n-1, n> A;
      for ( std::size_t i = 0; i < m; ++i )
        for ( std::size_t j = 0; j < n; ++j )
          A( i, j ) = TInternalNumber( basis[ i ][ j ] );
      //std::vector<TInternalNumber> w( n );
      for ( std::size_t col = 0; col < n; ++col)
        { // construct sub-matrix removing column col
          SimpleMatrix< TInternalNumber, n-1, n-1> M;
          for ( std::size_t i = 0; i < m; ++i )
            {
              std::size_t c = 0;
              for ( std::size_t j = 0; j < n; ++j)
                if ( j != col ) M( i, c++ ) = A( i, j );
            }
          functions::determinantBareiss( M, w[ col ] );
          if ( (col + n) % 2 == 0 ) w[ col ] = -w[ col ];
        }
    }

    /// Given a vector, returns the aligned vector with its component
    /// simplified by the gcd of all components (when the components
    /// are integers) or the aligned vector with a maximum oo-norm of
    /// 1 (when the components are floating-point numbers).
    ///
    /// @tparam TPoint any type of lattice point or real point.
    ///
    /// @param[in] v any vector.
    ///
    /// @return a simplified vector aligned with \a v.
    template <typename TPoint>
    TPoint
    computeSimplifiedVector( const TPoint& v )
    {
      return AffineGeometry<TPoint>::simplifiedVector( v );
    }
    
  } // namespace functions
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined AffineGeometry_h

#undef AffineGeometry_RECURSES
#endif // else defined(AffineGeometry_RECURSES)
    
