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
#include <iostream>
#include <string>
#include <vector>
#include <queue>
#include <set>
#include "DGtal/base/Common.h"
#include "DGtal/base/Clock.h"
#include "DGtal/arithmetic/IntegerComputer.h"

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
  template < typename TPoint >
  struct AffineSubset
  {
    typedef TPoint                     Point;
    typedef typename Point::Coordinate Scalar;
    typedef std::size_t                Size;
    typedef std::vector< Point >       Points; ///< type for range of points.
    typedef std::vector< Size >        Sizes;  ///< type for range of sizes.
    static const Size  dimension  = Point::dimension;

    // ----------------------- standard services --------------------------
  public:
    /// @name static affine services
    /// @{

    /// Given a range of points \a X, returns the affine dimension of
    /// its spanned affine subspace.
    ///
    /// @param X the range of input points (may be lattice points or not).
    /// @param tolerance the accepted 1-norm below which the vector is
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
    /// @param tolerance the accepted 1-norm below which the vector is
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
    /// @param tolerance the accepted 1-norm below which the vector is
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

    
    /// Reduces the vector \a v on the (partial or not) basis of vectors \a basis.
    ///
    /// @param v any vector
    /// @param basis a range of vectors forming a (partial or not) basis of the space.
    /// @param tolerance the accepted 1-norm below which the vector is
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
    /// @param v any vector.
    ///
    /// @param tolerance the accepted 1-norm below which the vector is
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
      if ( isNonZero( x, tolerance ) )
        {
          // Useful to reduce the norm of vectors for lattice vectors
          // and necessary for real vectors so that `tolerance` keeps
          // the same meaning.
          normalizeVector( w, x ); 
          // std::cout << "YES w=" << w << " |w|_1=" << x << " tol=" << tolerance << std::endl;
          basis.push_back( w );
          return true;
        }
      // else
      //   std::cout << "NO w=" << w << " |w|_1=" << x << " tol=" << tolerance << std::endl;
      return false;
    }
    
    /// Reduces vector \a w by a pivot vector \a b
    /// @param[in,out] w a 
    static
    void reduceVector( Point& w, const Point& b, const double tolerance )
    {
      Size n = w.size();
      // Find index of first non null pivot in b.
      Size lead = n;
      for ( Size j = 0; j < n; j++)
        if ( isNonZero( b[j], tolerance ) ) { lead = j; break; }
      if ( lead == n ) return; // b is null vector

      Scalar mul_w, mul_b;
      std::tie( mul_w, mul_b ) = getMultipliers( w[ lead ], b[ lead ] );
      
      for (Size j = 0; j < n; j++) 
        w[j] = mul_w * w[j] - mul_b * b[j];
    }
    
    template <typename TInteger>
    static
    std::pair< TInteger, TInteger > getMultipliers( TInteger a, TInteger b )
    {
      TInteger g = IntegerComputer< TInteger >::staticGcd( std::abs( a ), std::abs( b ) );
      return std::make_pair( b / g, a / g );
    }

    static
    std::pair< double, double > getMultipliers( double a, double b )
    {
      return std::make_pair( b, a );
    }

    static
    std::pair< float, float > getMultipliers( float a, float b )
    {
      return std::make_pair( b, a );
    }

    template <typename TInteger>
    static
    bool isNonZero( TInteger x, double )
    {
      return x != TInteger( 0 );
    }
    
    static
    bool isNonZero( float x, double tol )
    {
      const double dx = double(x);
      return (dx > tol) || ( dx < -tol );
    }
    
    static
    bool isNonZero( double x, double tol )
    {
      return ( x > tol ) || ( x < -tol );
    }

    template <typename TInteger>
    static
    bool isAbsGreater( TInteger x, TInteger y )
    {
      return std::abs( x ) > std::abs( y );
    }
    
    static
    bool isAbsGreater( float x, float y )
    {
      return std::fabs( x ) > std::fabs( y );      
    }
    
    static
    bool isAbsGreater( double x, double y )
    {
      return std::fabs( x ) > std::fabs( y );      
    }

    static void normalizeVector( Point& w, int32_t )
    {
      Dimension i = 0;
      while ( i < dimension && w[ i ] == 0 ) i++;
      if ( i == dimension ) return;
      int32_t g = std::abs( w[ i ] );
      for ( ; i < dimension; i++ )
        g = IntegerComputer< int32_t >::staticGcd( g, std::abs( w[ i ] ) );
      w /= g;
    }
    static void normalizeVector( Point& w , int64_t )
    {
      Dimension i = 0;
      while ( i < dimension && w[ i ] == 0 ) i++;
      if ( i == dimension ) return;
      int64_t g = std::abs( w[ i ] );
      for ( ; i < dimension; i++ )
        g = IntegerComputer< int32_t >::staticGcd( g, std::abs( w[ i ] ) );
      w /= g;
    }

    static
    void normalizeVector( Point& w, double x )
    {
      w /= x;
    }

    static
    void normalizeVector( Point& w, float x )
    {
      w /= x;
    }
    

  };

} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined AffineSubset_h

#undef AffineSubset_RECURSES
#endif // else defined(AffineSubset_RECURSES)
    
