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
#include "DGtal/kernel/IntegerConverter.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/arithmetic/IntegerComputer.h"
#include "DGtal/math/linalg/SimpleMatrix.h"
#include "DGtal/math/linalg/IntegerMatrixFunctions.h"

namespace DGtal
{
  // Forward declaration of AffineGeometry (needed for friend declaration).
  template < typename T > struct AffineGeometry;
  // Forward declaration of AffineBasis (needed for friend declaration).
  template < typename T > struct AffineBasis;

  namespace detail {

    /////////////////////////////////////////////////////////////////////////////
    // template class AffineGeometryPointOperations

    /// Description of template class 'AffineGeometryPointOperations' <p>
    /// \brief Aim: Internal class used by AffineGeometry to
    /// differentiate operations on lattice points and operations on
    /// points with floating-point coordinates.
    template < DGtal::Dimension dim,
               typename TEuclideanRing,
               typename TContainer=std::array<TEuclideanRing,dim> >
    struct AffineGeometryPointOperations
    {
      template <typename T> friend struct DGtal::AffineGeometry;
      template <typename T> friend struct DGtal::AffineBasis;
      typedef PointVector< dim, TEuclideanRing, TContainer > Point;
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
        for ( DGtal::Dimension k = i+1; k < Point::dimension; k++ )
          g = DGtal::IntegerComputer< TInteger >::staticGcd( g, abs( w[ k ] ) );
        for ( DGtal::Dimension k = i; k < Point::dimension; k++ )
          w[ k ] /= g;
      }

      /// Specialized version to cast points to other point type.
      template <typename TOtherPoint>
      static
      Point cast( const TOtherPoint& other )
      {
        return IntegerConverter< Point::dimension, Scalar >::cast( other );
      }
    }; // end of class AffineGeometryPointOperations


    /// Description of template class 'AffineGeometryPointOperations'
    /// <p> \brief Aim: Internal class used by AffineGeometry to
    /// differentiate operations on lattice points and operations on
    /// points with floating-point coordinates. This specialization
    /// assumes `double` as components.
    template < DGtal::Dimension dim,
               typename TContainer >
    struct AffineGeometryPointOperations< dim, double, TContainer >
    {
      template <typename T> friend struct DGtal::AffineGeometry;
      template <typename T> friend struct DGtal::AffineBasis;
      typedef PointVector< dim, double, TContainer > Point;
      typedef typename Point::Coordinate Scalar;

      // ----------------------- internal services --------------------------
    private:
      /// @name static internal services
      /// @{

      /// Specialized version to normalize a vector in case of double
      /// value parameter.
      static
      void normalizeVector( Point& w, double x )
      {
        w /= sqrt( x );
      }

      /// Specialized version to cast points to other point type.
      template <typename TOtherPoint>
      static
      Point cast( const TOtherPoint& other )
      {
        typedef typename TOtherPoint::Coordinate OtherScalar;
        Point result;
        for ( std::size_t i = 0; i < Point::dimension; i++ )
          result[ i ] = NumberTraits<OtherScalar>::castToDouble( other[ i ] );
        return result;
      }
    }; // end of class AffineGeometryPointOperations

    /// Description of template class 'AffineGeometryPointOperations'
    /// <p> \brief Aim: Internal class used by AffineGeometry to
    /// differentiate operations on lattice points and operations on
    /// points with floating-point coordinates. This specialization
    /// assumes `float` as components.
    template < DGtal::Dimension dim,
               typename TContainer >
    struct AffineGeometryPointOperations< dim, float, TContainer >
    {
      template <typename T> friend struct DGtal::AffineGeometry;
      template <typename T> friend struct DGtal::AffineBasis;
      typedef PointVector< dim, float, TContainer > Point;
      typedef typename Point::Coordinate Scalar;

      // ----------------------- internal services --------------------------
    private:
      /// @name static internal services
      /// @{

      /// Specialized version to normalize a vector in case of float
      /// value parameter.
      static
      void normalizeVector( Point& w, float x )
      {
        w /= sqrt( x );
      }

      /// Specialized version to cast points to other point type.
      template <typename TOtherPoint>
      static
      Point cast( const TOtherPoint& other )
      {
        Point result;
        for ( std::size_t i = 0; i < Point::dimension; i++ )
          result[ i ] = float( other[ i ] );
        return result;
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
      template <typename T> friend struct DGtal::AffineBasis;

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
        const Integer g = gcd( a, b );
        // return std::make_pair( b / g, a / g );
        // left multiplier should be positive.
        return (b >= 0)
          ? std::make_pair( b / g, a / g )
          : std::make_pair( -b / g, -a / g );
      }

      /// @param[in] a any integer number
      /// @param[in] b any integer number
      ///
      /// @return the greatest common divisor of a and b.
      static
      Integer gcd( Integer a, Integer b )
      {
        return IntegerComputer< Integer >::staticGcd( abs( a ), abs( b ) );
      }

      /// @param[in] a any integer number
      /// @param[in] b any integer number
      ///
      /// @return the least common multiple of a and b.
      static
      Integer lcmPositive( Integer a, Integer b )
      {
        const Integer g = gcd( a, b );
        return (a / g) * b;
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
      template <typename T> friend struct DGtal::AffineBasis;

      /// @param[in] a any number
      /// @param[in] b any number
      ///
      /// @return the pair ( b, a ), which allows to cancel a
      /// component in a Gauss pivoting algorithm.
      static
      std::pair< double, double > getMultipliers( double a, double b )
      {
        return (b >= 0 )
          ? std::make_pair( b, a )
          : std::make_pair( -b, -a );
      }

      /// @return 1.0
      static
      double gcd( double, double )
      {
        return 1.0;
      }

      /// @return 1.0
      static
      double lcmPositive( double, double )
      {
        return 1.0;
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
      template <typename T> friend struct DGtal::AffineBasis;

      /// @param[in] a any number
      /// @param[in] b any number
      ///
      /// @return the pair ( b, a ), which allows to cancel a
      /// component in a Gauss pivoting algorithm.
      static
      std::pair< float, float > getMultipliers( float a, float b )
      {
        return (b >= 0 )
          ? std::make_pair( b, a )
          : std::make_pair( -b, -a );
      }

      /// @return 1.0f
      static
      float gcd( float, float )
      {
        return 1.0f;
      }

      /// @return 1.0f
      static
      float lcmPositive( float, float )
      {
        return 1.0f;
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
  /// typedef AffineGeometry< Point >          Affine;
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
    typedef TPoint               Point;
    typedef typename Point::Coordinate Scalar;
    typedef typename Point::Container  Container;
    typedef std::size_t                Size;
    typedef std::vector< Point > Points; ///< type for range of points.
    typedef std::vector< Size >        Sizes;  ///< type for range of sizes.
    static const Size  dimension  = Point::dimension;
    typedef DGtal::detail::AffineGeometryPointOperations
    < dimension, Scalar, Container >   PointOps;
    typedef DGtal::detail::AffineGeometryScalarOperations< Scalar > ScalarOps;

    // ----------------------- standard services --------------------------
  public:
    /// @name static affine services
    /// @{

    /// Given a range of points \a X, returns the affine dimension of
    /// its spanned affine subspace.
    ///
    /// @tparam TInputPoint the type of input points (may be less precise).
    ///
    /// @param[in] X the range of input points (may be lattice points or not).
    ///
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return the affine dimension of \a X (-1 is empty set, 0 is a
    /// point, 1 is a line, etc)
    template <typename TInputPoint>
    static
    DGtal::int64_t
    affineDimension( const std::vector<TInputPoint>& X, const double tolerance = 1e-12 )
    {
      return DGtal::int64_t( affineSubset( X, tolerance ).size() ) - 1;
    }

    /// Given a range of points \a X, returns a subset of these points
    /// that form an affine basis of \a X. Equivalently it is a
    /// simplex whose affine space spans all the points of \a X.
    ///
    /// @tparam TInputPoint the type of input points (may be less precise).
    ///
    /// @param[in] X the range of input points (may be lattice points or not).
    ///
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return a subset of these points as a range of indices.
    ///
    /// @note Complexity is \f$O( m n^2 )\f$, where m=Cardinal(X) and n=dimension.
    template <typename TInputPoint>
    static
    std::vector< Size >
    affineSubset( const std::vector<TInputPoint>& X, const double tolerance = 1e-12 )
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
          Point v = transform( X[ i ] - X[ 0 ] );
          if ( addIfIndependent( basis, v, tolerance ) )
            chosen.push_back( i );
          if ( chosen.size() > dimension ) break;
        }
      return chosen;
    }

    /// Given a range of points \a X and a subset of it given by
    /// indices \a I, returns a subset of these points that form an
    /// affine basis of \a X[I]. Equivalently it is a simplex whose
    /// affine space spans all the points of \a X[I].
    ///
    /// @tparam TInputPoint the type of input points (may be less precise).
    ///
    /// @tparam TIndexRange any type of range of indices that
    /// specifies the subset of points of interest.
    ///
    /// @param[in] X the range of input points (may be lattice points or not).
    ///
    /// @param[in] I the range of indices specifying the subset of interest.
    ///
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return a subset of these points as a range of indices.
    ///
    /// @note Complexity is \f$O( m n^2 )\f$, where m=Cardinal(X) and n=dimension.
    template <typename TInputPoint, typename TIndexRange>
    static
    std::vector< Size >
    affineSubset( const std::vector<TInputPoint>& X,
                  const TIndexRange& I,
                  const double tolerance = 1e-12 )
    {
      Size m = I.size();
      // Process trivial cases.
      if ( m == 0 ) return { };
      if ( m == 1 ) return { I[ 0 ] };
      // Process general case.
      Points basis;  //< direction vectors
      Sizes  chosen; //< selected points
      chosen.push_back( I[ 0 ] ); //< reference point (first one, as it may be any one)
      for ( Size i = 1; i < m; i++ )
        {
          Point v = transform( X[ I[ i ] ] - X[ I[ 0 ] ] );
          if ( addIfIndependent( basis, v, tolerance ) )
            chosen.push_back( I[ i ] );
          if ( chosen.size() > dimension ) break;
        }
      return chosen;
    }

    /// Given a range of points \a X, returns a point and a range of
    /// vectors forming an affine basis containing \a X.
    ///
    /// @tparam TInputPoint the type of input points (may be less precise).
    ///
    /// @param[in] X the range of input points (may be lattice points or not).
    ///
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return a point and a range of vectors forming an affine basis containing X.
    ///
    /// @note Complexity is O( m n^2 ), where m=Cardinal(X) and n=dimension.
    template <typename TInputPoint>
    static
    std::pair< Point, Points >
    affineBasis( const std::vector<TInputPoint>& X, const double tolerance = 1e-12 )
    {
      Points basis;  //< direction vectors
      Size m = X.size();
      // Process trivial cases.
      if ( m == 0 ) return std::make_pair( Point::zero, basis );
      const Point o = transform( X[ 0 ] );
      if ( m == 1 ) return std::make_pair( o, basis );
      // Process general case.
      basis.reserve( dimension );
      for ( Size i = 1; i < m; i++ )
        {
          Point v = transform( X[ i ] ) - o;
          if ( addIfIndependent( basis, v, tolerance ) )
            if ( basis.size() >= dimension ) break;
        }
      return std::make_pair( o, basis );
    }

    /// Given a range of points \a X and a range of indices \a I
    /// describing its subset of interest, returns a point and a range
    /// of vectors forming an affine basis containing the subset `X[I]`.
    ///
    /// @tparam TInputPoint the type of input points (may be less precise).
    ///
    /// @param[in] X the range of input points (may be lattice points or not).
    ///
    /// @param[in] I the range of indices within X that specifies the subset of interest.
    ///
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return a point and a range of vectors forming an affine basis containing `X[I]`.
    ///
    /// @note Complexity is O( m n^2 ), where m=Cardinal(I) and n=dimension.
    template < typename TInputPoint, typename TIndexRange >
    static
    std::pair< TInputPoint, Points >
    affineBasis( const std::vector<TInputPoint>& X,
                 const TIndexRange& I,
                 const double tolerance = 1e-12 )
    {
      Points basis;  //< direction vectors
      Size m = I.size();
      // Process trivial cases.
      if ( m == 0 ) return std::make_pair( TInputPoint::zero, basis );
      if ( m == 1 ) return std::make_pair( X[ I[ 0 ] ], basis );
      // Process general case.
      const Point  o = transform( X[ I[ 0 ] ] );
      basis.reserve( dimension );
      for ( Size i = 1; i < m; i++ )
        {
          Point v = transform( X[ I[ i ] ] ) - o;
          if ( addIfIndependent( basis, v, tolerance ) )
            if ( basis.size() > dimension ) break;
        }
      return std::make_pair( X[ I[ 0 ] ], basis );
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
    Point
    independentVector( const Points& basis, const double tolerance = 1e-12 )
    {
      // If basis has already d independent vectors, then there is no
      // other independent vector.
      if ( basis.size() >= dimension ) return Point::zero;
      // At least one trivial canonic vector should be independent.
      Dimension k = 0;
      for ( ; k < dimension; k++ )
        {
          Point e_k = Point::base( k );
          Point w   = reductionOnBasis( e_k, basis, tolerance );
          Scalar sql2;
          functions::getSquaredNormL2( sql2, w );
          if ( ScalarOps::isNonZero( sql2, tolerance ) )
            return e_k;
        }
      trace.error() << "[AffineGeometry::independentVector]"
                    << " Unable to find independent vector." << std::endl;
      return Point::zero;
    }

    /// Given a partial basis of vectors, returns a new vector that is independent.
    ///
    /// @tparam TOtherPoint any type of point
    ///
    /// @param[in] basis a range of independent vectors that defines a
    /// partial basis of the space.
    ///
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return a canonic unit vector independent of all vectors of \a
    /// basis, or the null vector if the basis was not partial.
    template < typename TOtherPoint >
    static
    TOtherPoint
    independentVector( const Points& basis, const double tolerance = 1e-12 )
    {
      // If basis has already d independent vectors, then there is no
      // other independent vector.
      if ( basis.size() >= dimension ) return TOtherPoint::zero;
      // At least one trivial canonic vector should be independent.
      Dimension k = 0;
      for ( ; k < dimension; k++ )
        {
          Point e_k = Point::base( k );
          Point w   = reductionOnBasis( e_k, basis, tolerance );
          Scalar sql2;
          functions::getSquaredNormL2( sql2, w );
          if ( ScalarOps::isNonZero( sql2, tolerance ) )
            return TOtherPoint::base( k );
        }
      trace.error() << "[AffineGeometry::independentVector]"
                    << " Unable to find independent vector." << std::endl;
      return TOtherPoint::zero;
    }

    /// Complete the vectors \a basis with independent vectors so as
    /// to form a basis of the space. When \a normal_vector is true,
    /// the last added vector is guaranteed to be \b orthogonal to all
    /// the previous vectors.
    ///
    /// @note In 3D, given two independent vectors as input, then the
    /// added vector is the \b cross \b product of these two
    /// vectors. In nD, it is thus a generalization of the cross
    /// product.
    ///
    /// @param[in,out] basis a range of independent vectors of size less than dimension.
    ///
    /// @param[in] normal_vector when 'true', the last vector of the
    /// basis is orthogonal to all the other vectors of the basis,
    /// otherwise it is just an independent canonic vector.
    ///
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    static
    void
    completeBasis( Points& basis,
                   bool normal_vector,
                   const double tolerance = 1e-12 )
    {
      if ( basis.size() >= dimension ) return;
      while ( ( basis.size() + 1 ) < dimension )
        { // add an independent vector
          const Point u = independentVector( basis, tolerance );
          basis.push_back( u );
        }
      if ( normal_vector )
        basis.push_back( orthogonalVector( basis ) );
      else
        basis.push_back( independentVector( basis, tolerance ) );
    }

    /// Given a primitive lattice vector \a N, returns a possible basis for
    /// its orthogonal d-1 dimensional lattice.
    ///
    /// @tparam TOtherPoint any type of lattice point
    ///
    /// @param[in] N a non null primitive lattice vector
    ///
    /// @param[in] shortened when 'false', the basis is in row echelon
    /// form, when 'true', the basis is shortened (with pairwise
    /// shortenings, may not be optimal, except in 3D).
    ///
    /// @return the d-1 vectors forming a basis of the orthogonal lattice to \a N.
    template < typename TInputPoint >
    static
    std::vector< Point >
    orthogonalLatticeBasis( const TInputPoint& N, bool shortened = false )
    {
      std::vector< Scalar > Np( N.begin(), N.end() );
      Np.resize( Point::dimension );
      auto B = functions::computeOrthogonalLattice( Np );
      if ( shortened )
        functions::shortenBasis( B );
      std::vector< Point > R( B.size() );
      for ( std::size_t i = 0; i < B.size(); i++ )
        for ( std::size_t j = 0; j < Point::dimension; j++ )
          R[ i ][ j ] = B[ i ][ j ];
      return R;
    }

    /// @}

    // ----------------------- specific services --------------------------
  public:
    /// @name static specific services
    /// (Used internally)
    /// @{

    /// Reduces the vector \a v on the (partial or not) basis of vectors \a basis.
    ///
    /// @param[in] v any vector
    ///
    /// @param[in] basis a range of vectors forming a (partial or not) basis of the space.
    ///
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return the part of \a v that cannot be expressed as a linear
    /// combination of vectors of \a basis, and hence a null vector if
    /// \a v is a linear combination of the vectors of the basis.
    static
    Point
    reductionOnBasis( const Point& v,
                      const Points& basis,
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
    /// @param[in] v any vector.
    ///
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return 'true' iff the vector \a v was not a linear
    /// combination of the vectors of the basis and the basis is then
    /// enriched by the direction indicated by \a v, otherwise returns 'false'.
    ///
    /// @note If the points have integer coordinates, the
    /// normalization of the new basis vector is its reduction by its
    /// gcd, otherwise the vector has unit L2-norm.
    static
    bool addIfIndependent( Points& basis,
                           const Point& v,
                           const double tolerance )
    {
      Point  w = reductionOnBasis( v, basis, tolerance );
      Scalar x;
      functions::getSquaredNormL2( x, w );
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
    /// @return the coefficients (alpha,beta) for reduction such that
    /// alpha * w - beta * b is the returned reduced vector, alpha >=
    /// 0.
    ///
    /// @note This reduction is an elementary Gauss elimination.
    static
    std::pair< Scalar, Scalar >
    reduceVector( Point& w, const Point& b, const double tolerance )
    {
      Scalar mul_w = 0;
      Scalar mul_b = 0;
      Size n = w.size();
      // Find index of first non null pivot in b.
      Size lead = n;
      for ( Size j = 0; j < n; j++)
        if ( ScalarOps::isNonZero( b[j], tolerance ) ) { lead = j; break; }
      if ( lead == n ) return std::make_pair( mul_w, mul_b ); // b is null vector

      std::tie( mul_w, mul_b ) = ScalarOps::getMultipliers( w[ lead ], b[ lead ] );

      for (Size j = 0; j < n; j++)
        w[j] = mul_w * w[j] - mul_b * b[j];
      return std::make_pair( mul_w, mul_b );
    }

    /// Reduces vector \a w by the vector \a b
    ///
    /// @param[in,out] w any vector
    /// @param[in] b a non-null vector of the current basis.
    /// @param[in] start the starting component index.
    /// @param[in] tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return the coefficients (alpha,beta) for reduction such that
    /// alpha * w - beta * b is the returned reduced vector, alpha >=
    /// 0.
    ///
    /// @note This reduction is an elementary Gauss elimination.
    static
    std::pair< Scalar, Scalar >
    reduceVector( Point& w, const Point& b,
                  Dimension start, const double tolerance )
    {
      Scalar mul_w = 0;
      Scalar mul_b = 0;
      Size n = w.size();
      // Find index of first non null pivot in b.
      Size lead = n;
      for ( Size j = start; j < n; j++)
        if ( ScalarOps::isNonZero( b[j], tolerance ) ) { lead = j; break; }
      if ( lead == n ) return std::make_pair( mul_w, mul_b ); // b is null vector

      std::tie( mul_w, mul_b ) = ScalarOps::getMultipliers( w[ lead ], b[ lead ] );

      for (Size j = 0; j < n; j++)
        w[j] = mul_w * w[j] - mul_b * b[j];
      return std::make_pair( mul_w, mul_b );
    }


    /// Transform a vector or point into the representation chosen for
    /// AffineGeometry class. This overloading takes care of the
    /// trivial case, where there is no need to change the
    /// representation.
    ///
    /// @param[in] w any vector or point
    /// @return the same vector or point (does nothing).
    static
    const Point&
    transform( const Point& w )
    {
      return w;
    }

    /// Transform a vector or point into the representation chosen for
    /// AffineGeometry class. This overloading takes care of the
    /// generic case, where a transformation of each component is required.
    ///
    /// @param[in] w any vector or point
    /// @return the same vector or point, but in the representation chosen for this class.
    template <typename TInputPoint>
    static
    Point
    transform( const TInputPoint& w )
    {
      return PointOps::cast( w );
    }

    /// Given `d-1` independent vectors in dD, returns a vector that
    /// is orthogonal to each of them.
    ///
    /// @note In 3D, given two independent vectors as input, then the
    /// added vector is the (reduced) \b cross \b product of these two
    /// vectors. In nD, it is thus a kind of generalization of the cross
    /// product.
    ///
    /// @param[in] basis a range of independent vectors of size dimension-1.
    ///
    /// @return a vector of coefficients (represented with the given
    /// number type), or the null vector if the basis is not d-1-dimensional.
    static
    Point
    orthogonalVector( const Points& basis )
    {
      Point w;
      const std::size_t n = dimension;
      if ( ( basis.size() + 1 ) != dimension ) return w;
      const std::size_t m = basis.size();
      SimpleMatrix< Scalar, dimension-1, dimension> A;
      for ( std::size_t i = 0; i < m; ++i )
        for ( std::size_t j = 0; j < n; ++j )
          A( i, j ) = basis[ i ][ j ];
      for ( std::size_t col = 0; col < n; ++col)
        { // construct sub-matrix removing column col
          SimpleMatrix< Scalar, dimension-1, dimension-1> M;
          for ( std::size_t i = 0; i < m; ++i )
            {
              std::size_t c = 0;
              for ( std::size_t j = 0; j < n; ++j)
                if ( j != col ) M( i, c++ ) = A( i, j );
            }
          functions::getDeterminantBareiss( w[ col ], M );
          if ( (col+dimension) % 2 == 0 ) w[ col ] = -w[ col ];
        }
      return simplifiedVector( w );
    }

    /// Given `d-1` independent vectors in dD, returns a vector that
    /// is orthogonal to each of them.
    ///
    /// @note In 3D, given two independent vectors as input, then the
    /// added vector is the (reduced) \b cross \b product of these two
    /// vectors. In nD, it is thus a kind of generalization of the cross
    /// product.
    ///
    /// @param[in] basis a range of independent vectors of size dimension-1.
    ///
    /// @return a vector of coefficients (represented with the given
    /// number type), or the null vector if the basis is not d-1-dimensional.
    template <typename TInternalNumber>
    static
    Point
    orthogonalVector( const Points& basis )
    {
      Point w;
      typedef  PointVector< dimension, TInternalNumber > InternalPoint;
      InternalPoint W;
      const std::size_t n = dimension;
      if ( ( basis.size() + 1 ) != dimension ) return w;
      const std::size_t m = basis.size();
      SimpleMatrix< Scalar, dimension-1, dimension> A;
      for ( std::size_t i = 0; i < m; ++i )
        for ( std::size_t j = 0; j < n; ++j )
          A( i, j ) = basis[ i ][ j ];
      for ( std::size_t col = 0; col < n; ++col)
        { // construct sub-matrix removing column col
          SimpleMatrix< Scalar, dimension-1, dimension-1> M;
          for ( std::size_t i = 0; i < m; ++i )
            {
              std::size_t c = 0;
              for ( std::size_t j = 0; j < n; ++j)
                if ( j != col ) M( i, c++ ) = A( i, j );
            }
          functions::getDeterminantBareiss( W[ col ], M );
          if ( (col+dimension) % 2 == 0 ) W[ col ] = -W[ col ];
        }
      W = AffineGeometry<InternalPoint>::simplifiedVector( W );
      return transform( W );
    }

    /// Given a vector, returns the aligned vector with its component
    /// simplified by the gcd of all components (when the components
    /// are integers) or the aligned vector with unit L2-norm (when
    /// the components are floating-point numbers).
    ///
    /// @param[in] v any vector.
    ///
    /// @return a simplified vector aligned with \a v.
    static
    Point simplifiedVector( Point v )
    {
      Scalar x;
      functions::getSquaredNormL2( x, v );
      PointOps::normalizeVector( v, x );
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
    /// @param tolerance the accepted squared L2-norm below which the vector is
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
    /// @param tolerance the accepted squared L2-norm below which the vector is
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

    /// Given a range of points \a X and a subset of it given by
    /// indices \a I, returns a subset of these points that form an
    /// affine basis of \a X[I]. Equivalently it is a simplex whose
    /// affine space spans all the points of \a X[I].
    ///
    /// @tparam TPoint any type of lattice point or real point.
    ///
    /// @tparam TIndexRange any type of range of indices that
    /// specifies the subset of points of interest.
    ///
    /// @param[in] X the range of input points (may be lattice points or not).
    ///
    /// @param[in] I the range of indices specifying the subset of interest.
    ///
    /// @param[in] tolerance the accepted squared L2-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @return a subset of these points as a range of indices.
    ///
    /// @note Complexity is \f$O( m n^2 )\f$, where m=Cardinal(X) and n=dimension.
    template <typename TPoint, typename TIndexRange>
    static
    std::vector< std::size_t >
    computeAffineSubset( const std::vector<TPoint>& X,
                         const TIndexRange& I,
                         const double tolerance = 1e-12 )
    {
      return AffineGeometry<TPoint>::affineSubset( X, I, tolerance );
    }


    /// Given a range of points \a X, returns a point and a range of
    /// vectors forming an affine basis containing \a X.
    ///
    /// @tparam TPoint any type of lattice point or real point (but
    /// one may choose a more precise type than TInputPoint.
    ///
    /// @tparam TInputPoint any type of lattice point or real point.
    ///
    /// @param[out] o the origin point
    ///
    /// @param[out] basis a range of vectors forming an affine basis containing \a X.
    ///
    /// @param[in] X the range of input points (may be lattice points or not).
    ///
    /// @param[in] tolerance the accepted squared L2-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @note Complexity is O( m n^2 ), where m=Cardinal(X) and n=dimension.
    template <typename TPoint, typename TInputPoint>
    void
    getAffineBasis( TInputPoint& o,
                    std::vector< TPoint >& basis,
                    const std::vector< TInputPoint >& X,
                    const double tolerance = 1e-12 )
    {
      std::tie( o, basis ) = AffineGeometry<TPoint>::affineBasis( X, tolerance );
    }

    /// Given a range of points \a X, returns a point and a range of
    /// vectors forming an affine basis containing \a X.
    ///
    /// @tparam TPoint any type of lattice point or real point (but
    /// one may choose a more precise type than TInputPoint.
    ///
    /// @tparam TInputPoint any type of lattice point or real point.
    ///
    /// @tparam TIndexRange any type of range of indices.
    ///
    /// @param[out] o the origin point
    ///
    /// @param[out] basis a range of vectors forming an affine basis containing \a X.
    ///
    /// @param[in] X the range of input points (may be lattice points or not).
    ///
    /// @param[in] I the range of indices within X that specifies the subset of interest.
    ///
    /// @param[in] tolerance the accepted squared L2-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @note Complexity is O( m n^2 ), where m=Cardinal(I) and n=dimension.
    template < typename TPoint, typename TInputPoint, typename TIndexRange >
    static
    void
    getAffineBasis( TInputPoint& o,
                    std::vector< TPoint >& basis,
                    const std::vector< TInputPoint >& X,
                    const TIndexRange& I,
                    const double tolerance = 1e-12 )
    {
      std::tie( o, basis ) = AffineGeometry<TPoint>::affineBasis( X, I, tolerance );
    }

    /// Given a partial basis of vectors, returns a canonic unit
    /// vector that is independent.
    ///
    /// @tparam TPoint any type of lattice point or real point.
    ///
    /// @param[in] basis a range of independent vectors that defines a
    /// partial basis of the space.
    ///
    /// @param[in] tolerance the accepted squared L2-norm below which the vector is
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
    /// to form a basis of the space. When \a normal_vector is true,
    /// the last added vector is guaranteed to be \b orthogonal to all
    /// the previous vectors, otherwise it is just an independent
    /// canonic vector.
    ///
    /// @note In 3D, given two independent vectors as input, then the
    /// added vector is the (reduced for integers, normalized with 1
    /// squared L2-norm for floats) \b cross \b product of these two
    /// vectors. In nD, it is thus a generalization of the cross
    /// product.
    ///
    /// @tparam TPoint any type of lattice point or real point.
    ///
    /// @param[in,out] basis a range of independent vectors of size
    /// less than dimension, which is completed so as to be a basis of
    /// the full space.
    ///
    /// @param[in] normal_vector when 'true', the last vector of the
    /// basis is orthogonal to all the other vectors of the basis,
    /// otherwise it is just an independent canonic vector.
    ///
    /// @param[in] tolerance the accepted squared L2-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    template <typename TPoint>
    static
    void
    getCompleteBasis( std::vector< TPoint >& basis,
                      bool normal_vector,
                      const double tolerance = 1e-12 )
    {
      AffineGeometry<TPoint>::completeBasis( basis, normal_vector, tolerance );
    }

    /// Given `d-1` independent vectors in dD, outputs a vector that
    /// is orthogonal to each of them. This function signature allows
    /// automatic internal type computations deduction.
    ///
    /// @note In 3D, given two independent vectors as input, then the
    /// added vector is the (reduced for integers, normalized with 1
    /// squared L2-norm for floats) \b cross \b product of these two
    /// vectors. In nD, it is thus a generalization of the cross
    /// product.
    ///
    /// @tparam TPoint any type of lattice point or real point.
    ///
    /// @param[in] basis a range of independent vectors of size dimension-1.
    ///
    /// @return an orthogonal vector to every vector of basis (reduced
    /// or normalized depending on integer/floating-point number
    /// type).
    template < typename TPoint >
    static
    TPoint
    computeOrthogonalVectorToBasis( const std::vector<TPoint>& basis )
    {
      if ( ( basis.size() + 1 ) != TPoint::dimension ) return TPoint::zero;
      return AffineGeometry<TPoint>::orthogonalVector( basis );
    }

    /// Given a range of points \a X, a range of indices \a I
    /// specifying the affine subset of interest, returns a vector
    /// that is orthogonal to this affine subset, if it is
    /// d-1-dimensional.
    ///
    /// @tparam TPoint any type of lattice point or real point.
    ///
    /// @tparam TInputPoint any type of lattice point or real point.
    ///
    /// @tparam TIndexRange any type of range of indices that
    /// specifies the subset of points of interest.
    ///
    /// @param[out] w an orthogonal vector to every vector of the affine subset (reduced
    /// or normalized depending on integer/floating-point number
    /// type).
    ///
    /// @param[in] X the range of input points (may be lattice points or not).
    ///
    /// @param[in] I the range of indices within X that specifies the subset of interest.
    ///
    /// @param[in] tolerance the accepted squared L2-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @note The orthogonal vector is computed as follows: (i)
    /// identifies an affine subset J of maximal affine dimension,
    /// (ii) computes the basis (X_J_k - X_J_0), k>0, (iii) computes
    /// the cofactors using Bareiss determinant. This method induces
    /// smaller integers than (i) reducing the basis, (ii) using
    /// cofactors with Bareiss determinant, or the method that solely
    /// cofactors with standard determinant computation.
    template < typename TPoint, typename TInputPoint, typename TIndexRange >
    static
    void
    getOrthogonalVector( TPoint& w,
                         const std::vector< TInputPoint >& X,
                         const TIndexRange& I,
                         const double tolerance = 1e-12 )
    {
      typedef AffineGeometry<TPoint> Affine;
      w = TPoint::zero;
      TInputPoint o;
      auto subset = Affine::affineSubset( X, I, tolerance );
      if ( ( subset.size() ) != TPoint::dimension ) return;
      std::vector<TPoint> basis( TPoint::dimension - 1);
      for ( auto i = 0; i < basis.size(); i++ )
        basis[ i ] = Affine::transform( X[ subset[ i+1 ] ] - X[ subset[ 0 ] ] );
      w = Affine::orthogonalVector( basis );
    }

    /// Given a range of points \a X, returns a vector that is
    /// orthogonal to this affine set, if it is d-1-dimensional.
    ///
    /// @tparam TPoint any type of lattice point or real point (type
    /// for computations and output).
    ///
    /// @tparam TInputPoint any type of lattice point or real point.
    ///
    /// @param[out] w an orthogonal vector to every vector of the affine subset (reduced
    /// or normalized depending on integer/floating-point number
    /// type).
    ///
    /// @param[in] X the range of input points (may be lattice points or not).
    ///
    /// @param[in] tolerance the accepted squared L2-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    ///
    /// @note The orthogonal vector is computed as follows: (i)
    /// identifies an affine subset J of maximal affine dimension,
    /// (ii) computes the basis (X_J_k - X_J_0), k>0, (iii) computes
    /// the cofactors using Bareiss determinant. This method induces
    /// smaller integers than (i) reducing the basis, (ii) using
    /// cofactors with Bareiss determinant, or the method that solely
    /// cofactors with standard determinant computation.
    template < typename TPoint, typename TInputPoint>
    static
    void
    getOrthogonalVector( TPoint& w,
                         const std::vector< TInputPoint >& X,
                         const double tolerance = 1e-12 )
    {
      typedef AffineGeometry<TPoint> Affine;
      w = TPoint::zero;
      TInputPoint o;
      auto subset = AffineGeometry<TPoint>::affineSubset( X, tolerance );
      if ( ( subset.size() ) != TPoint::dimension ) return;
      std::vector<TPoint> basis( TPoint::dimension - 1);
      for ( auto i = 0; i < basis.size(); i++ )
        basis[ i ] = Affine::transform( X[ subset[ i+1 ] ] - X[ subset[ 0 ] ] );
      w = Affine::orthogonalVector( basis );
    }

    /// Given a primitive lattice vector \a N, returns a possible basis for
    /// its orthogonal d-1 dimensional lattice.
    ///
    /// @tparam TPoint any type of lattice point (type for computations and output).
    /// @tparam TInputPoint any type of lattice point
    ///
    /// @param[out] B the d-1 dimension basis of the orthogonal lattice to \a N.
    ///
    /// @param[in] N a non null primitive lattice vector
    ///
    /// @param[in] shortened when 'false', the basis is in row echelon
    /// form, when 'true', the basis is shortened (with pairwise
    /// shortenings, may not be optimal, except in 3D).
    template < typename TPoint, typename TInputPoint>
    static
    void
    getOrthogonalLatticeBasis( std::vector< TPoint >& B,
                               const TInputPoint& N, bool shortened = false )
    {
      B = AffineGeometry<TPoint>::orthogonalLatticeBasis( N, shortened );
    }


    /// Given a vector, returns the aligned vector with its component
    /// simplified by the gcd of all components (when the components
    /// are integers) or the aligned vector with a squared L2-norm of
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
