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
 * @file AffineSubset.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2025/08/22
 *
 * Header file for module AffineSubset.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(AffineSubset_RECURSES)
#error Recursive header files inclusion detected in AffineSubset.h
#else // defined(AffineSubset_RECURSES)
/** Prevents recursive inclusion of headers. */
#define AffineSubset_RECURSES

#if !defined AffineSubset_h
/** Prevents repeated inclusion of headers. */
#define AffineSubset_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/arithmetic/IntegerComputer.h"

namespace DGtal
{
  // Forward declaration of AffineSubset (needed for friend declaration).
  template < typename T > struct AffineSubset;

  namespace detail {

    /////////////////////////////////////////////////////////////////////////////
    // template class AffineSubsetPointOperations

    /// Description of template class 'AffineSubsetPointOperations' <p>
    /// \brief Aim: Internal class used by AffineSubset to
    /// differentiate operations on lattice points and operations on
    /// points with floating-point coordinates. 
    ///
    /// @tparam TScalar any integer or floating point number type.
    template < typename TPoint >
    struct AffineSubsetPointOperations
    {
      template <typename T> friend struct DGtal::AffineSubset;
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
        TInteger g = std::abs( w[ i ] );
        for ( ; i < Point::dimension; i++ )
          g = DGtal::IntegerComputer< TInteger >::staticGcd( g, std::abs( w[ i ] ) );
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
      
    }; // end of class AffineSubsetPointOperations

    

    /////////////////////////////////////////////////////////////////////////////
    // template class AffineSubsetScalarOperations

    /// Description of template class 'AffineSubsetScalarOperations'
    /// <p> \brief Aim: Internal class used by AffineSubset to
    /// differentiate operations on point coordinates, which may be
    /// integer or floating-point numbers.. The generic class assume
    /// integer coordinates, while there are two specializations for
    /// float and double.
    ///
    /// @tparam TScalar any integer or floating point number type.
    template < typename TScalar >
    struct AffineSubsetScalarOperations
    {
      template <typename T> friend struct DGtal::AffineSubset;

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
        Integer g = IntegerComputer< Integer >::staticGcd( std::abs( a ), std::abs( b ) );
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

    }; // end of class AffineSubsetScalarOperations

    /// Description of template class 'AffineSubsetScalarOperations' <p>
    /// \brief Aim: Internal class used by AffineSubset to
    /// differentiate operations on lattice points and operations on
    /// points with floating-point coordinates. This specialization
    /// assume double for coordinates.
    template <>
    struct AffineSubsetScalarOperations< double >
    {
      template <typename T> friend struct DGtal::AffineSubset;
    
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

    }; // end of class AffineSubsetScalarOperations< double >

    /// Description of template class 'AffineSubsetScalarOperations' <p>
    /// \brief Aim: Internal class used by AffineSubset to
    /// differentiate operations on lattice points and operations on
    /// points with floating-point coordinates. This specialization
    /// assume float for coordinates.
    template <>
    struct AffineSubsetScalarOperations< float >
    {
      template <typename T> friend struct DGtal::AffineSubset;
    
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
      
    }; // end of class AffineSubsetScalarOperations< float >
    
  } // namespace detail

} // namespace DGtal


namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class AffineSubset

  /// Description of template class 'AffineSubset' <p> \brief Aim:
  /// Utility class to determine an affine subset of an input set of
  /// points which defines the same affine space as this set of
  /// points. It provides exact results when the input is
  /// composed of lattice points.
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
  /// #include "DGtal/geometry/tools/AffineSubset.h"
  /// ...
  /// typedef SpaceND< 3, int >                Space;      
  /// typedef Space::Point                     Point;
  /// typedef AffineSubset< Point >            Affine;
  /// std::vector<Point> X = { Point{1, 0, 0}, Point{2, 1, 0}, Point{3, 2, 0}, Point{3, 1, 1}, Point{5, 2, 2}, Point{4, 2, 1} };
  /// auto I = Affine::affineSubset( X ); /// 3 points with indices (0,1,3)
  /// auto B = Affine::affineBasis( X ); /// (1,0,0) and 2 basis vectors (1,1,0) and (2,1,1).
  /// auto d = Affine::affineDimension( X ); /// 2
  /// @endcode
  ///
  /// @see testAffineSubset.cpp
  template < typename TPoint >
  struct AffineSubset
  {
    typedef TPoint                     Point;
    typedef typename Point::Coordinate Scalar;
    typedef std::size_t                Size;
    typedef std::vector< Point >       Points; ///< type for range of points.
    typedef std::vector< Size >        Sizes;  ///< type for range of sizes.
    static const Size  dimension  = Point::dimension;
    typedef DGtal::detail::AffineSubsetPointOperations< Point >   PointOps;
    typedef DGtal::detail::AffineSubsetScalarOperations< Scalar > ScalarOps;
    
    // ----------------------- standard services --------------------------
  public:
    /// @name static affine services
    /// @{

    /// Given a range of points \a X, returns the affine dimension of
    /// its spanned affine subspace.
    ///
    /// @param X the range of input points (may be lattice points or not).
    /// @param tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    /// @return the affine dimension of \a X
    /// @param X the
    static
    DGtal::int64_t affineDimension( const Points& X, const double tolerance = 1e-12 )
    {
      return DGtal::int64_t( affineBasis( X, tolerance ).size() ) - 1;
    }
    
    /// Given a range of points \a X, returns a subset of these points
    /// that form an affine basis of \a X.
    ///
    /// @param X the range of input points (may be lattice points or not).
    /// @param tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    /// @return a subset of these points as a range of indices.
    ///
    /// @note Complexity is O( m n^2 ), where m=#X and n=dimension.
    static
    std::vector< Size > affineSubset( const Points& X, const double tolerance = 1e-12 )
    {
      Size m = X.size();
      // Process trivial cases.
      if ( m == 0 ) return { };
      if ( m == 1 ) return { Size( 0 ) };
      // Process general case.
      Dimension n = X[0].size();
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
    /// @param X the range of input points (may be lattice points or not).
    /// @param tolerance the accepted oo-norm below which the vector is
    /// null (used only for points with float/double coordinates).
    /// @return a point and a range of vectors forming an affine basis containing \a X.
    ///
    /// @note Complexity is O( m n^2 ), where m=#X and n=dimension.
    static
    std::pair< Point, Points > affineBasis( const Points& X, const double tolerance = 1e-12 )
    {
      std::vector< Size > indices = affineSubset( X, tolerance );
      Points basis( indices.size() - 1 );
      for ( Size i = 0; i < basis.size(); i++ )
        basis[ i ] = X[ indices[ i+1 ] ] - X[ indices[ 0 ] ];
      return std::make_pair( X[ indices[ 0 ] ], basis );
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
          // std::cout << "YES w=" << w << " |w|_1=" << x << " tol=" << tolerance << std::endl;
          basis.push_back( w );
          return true;
        }
      // else
      //   std::cout << "NO w=" << w << " |w|_1=" << x << " tol=" << tolerance << std::endl;
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

    /// @}
    
  };

} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined AffineSubset_h

#undef AffineSubset_RECURSES
#endif // else defined(AffineSubset_RECURSES)
    
