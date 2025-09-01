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
 * @file AffineBasis.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2025/08/29
 *
 * Header file for module AffineBasis.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(AffineBasis_RECURSES)
#error Recursive header files inclusion detected in AffineBasis.h
#else // defined(AffineBasis_RECURSES)
/** Prevents recursive inclusion of headers. */
#define AffineBasis_RECURSES

#if !defined AffineBasis_h
/** Prevents repeated inclusion of headers. */
#define AffineBasis_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/tools/AffineGeometry.h"

namespace DGtal
{
    /////////////////////////////////////////////////////////////////////////////
  // template class AffineBasis

  /// Description of template class 'AffineBasis' <p> \brief Aim:
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
  /// #include "DGtal/geometry/tools/AffineBasis.h"
  /// ...
  /// typedef SpaceND< 3, int >                Space;      
  /// typedef Space::Point                     Point;
  /// typedef AffineBasis< Point >            Affine;
  /// std::vector<Point> X = { Point{1, 0, 0}, Point{2, 1, 0}, Point{3, 2, 0}, Point{3, 1, 1}, Point{5, 2, 2}, Point{4, 2, 1} };
  /// auto I = Affine::affineSubset( X ); /// 3 points with indices (0,1,3)
  /// auto B = Affine::affineBasis( X ); /// (1,0,0) and 2 basis vectors (1,1,0) and (2,1,1).
  /// auto d = Affine::affineDimension( X ); /// 2
  /// @endcode
  ///
  /// @see testAffineBasis.cpp
  template < typename TPoint >
  struct AffineBasis
  {
    typedef TPoint                     Point;
    typedef typename Point::Coordinate Scalar;
    typedef std::vector< Point >       Points;
    typedef AffineGeometry< Point >    Affine;
    
    // ----------------------- standard services --------------------------
  public:
    /// @name standard services
    /// @{

    /// Default constructor.
    /// This create an affine basis of dimension 0 at the origin.
    AffineBasis() = default;

    /// Constructor from points.
    /// @param[in] points the range of points belonging to the affine space.
    ///
    /// @param[in] tolerance the accepted oo-norm below which the
    /// vector is null (used only for points with float/double
    /// coordinates).
    AffineBasis( const Points& points,
                 const double tolerance = 1e-12 )
      : epsilon( tolerance )
    {
      if ( points.size() == 0 ) return;
      first  = points[ 0 ];
      second = Points( points.size() - 1 );
      for ( std::size_t i = 0; i < second.size(); i++ )
        second[ i ] = points[ i+1 ] - first;
      reduce();
    }
    
    /// Constructor from origin and basis.
    /// @param[in] origin the origin of the affine basis
    /// @param[in] basis the range of vectors forming the basis
    ///
    /// @param[in] is_reduced when 'true', assumes that the given
    /// basis is already reduced, otherwise it forces the reduction of
    /// the basis.
    ///
    /// @param[in] tolerance the accepted oo-norm below which the
    /// vector is null (used only for points with float/double
    /// coordinates).
    AffineBasis( const Point& origin, const Points& basis,
                 bool is_reduced = false,
                 const double tolerance = 1e-12 )
      : first( origin ), second( basis ), epsilon( tolerance )
    {
      if ( ! is_reduced ) reduce();
    }

    /// @returns the affine dimension of the basis
    /// @pre the basis is reduced.
    Dimension dimension() const
    {
      return second.size();
    }
    
    /// Reduces the basis so that each basis vector is reduced and
    /// removes linearly dependent vectors.
    void reduce()
    {
      for ( auto& v : second )
        v = Affine::simplifiedVector( v );
      std::vector< bool > is_independent( second.size() );
      for ( std::size_t i = 0; i < second.size(); i++ )
        {
          Point& w           = second[ i ];
          is_independent[ i ] = true;
          for ( std::size_t j = 0; j < i; j++ )
            if ( is_independent[ j ] )
              Affine::reduceVector( w, second[ j ], epsilon );
          if ( ! Affine::ScalarOps::isNonZero( w.normInfinity(), epsilon ) )
            is_independent[ i ] = false;
          else
            w = Affine::simplifiedVector( w );
        }
      Points new_basis;
      for ( std::size_t i = 0; i < second.size(); i++ )
        if ( is_independent[ i ] )
          new_basis.push_back( second[ i ] );
      std::swap( second, new_basis );
    }

    /// @return the origin of this affine basis.
    const Point& origin() const
    {
      return first;
    }
    
    /// @return the vectors spanning the vector space of this affine
    /// basis.
    const Points& basis() const
    {
      return second;
    }
    /// @}

    // ----------------------- geometry services --------------------------
  public:
    /// @name geometry services
    /// @{

    /// @param[in] basis a reduced basis of vectors
    ///
    /// @return 'true' if and only if 'this' basis spans the same
    /// vector space as \a basis.
    bool isSameBasis( const Points& basis ) const
    {
      if ( basis.size() != second.size() ) return false;
      for ( const auto& b : basis )
        if ( ! isParallel( b ) ) return false;
      return true;
    }

    /// @param[in] p any lattice point
    ///
    /// @return its rational coordinates `L/d` as a pair `(d, L)` in
    /// this affine basis, where `L` is a lattice vector and `d` is
    /// the common denominator for the components of `L`.
    std::pair< Scalar, Point > rationalCoordinates( const Point& p ) const
    {
      const auto [d, lambda, remainder] = decompose( p );
      return std::make_pair( d, lambda );
    }

    /// @param[in] p any lattice point
    ///
    /// @return 'true' if and only if the point belongs to the affine
    /// space spanned by 'this'.
    bool isOnAffineSpace( const Point& p ) const
    {
      const auto [d, lambda, r] = decompose( p );
      return ! Affine::ScalarOps::isNonZero( r.normInfinity(), epsilon );
    }

    /// @param[in] w any lattice vector
    ///
    /// @return 'true' if and only if the vector w is parallel to the
    /// affine space spanned by 'this'.
    bool isParallel( const Point& w ) const
    {
      const auto [d, lambda, r] = decomposeVector( w );
      return ! Affine::ScalarOps::isNonZero( r.normInfinity(), epsilon );
    }

    /// Given (d, lambda, r), recompose the point `p` such that `d *
    /// (p - o) = l[0]b[0] + ... l[i]b[i] + r`. Given only the
    /// rational coordinates (d,lambda), the point is assumed to lie
    /// on this affine space (i.e. r == 0 ).
    ///
    /// @param[in] d the common denominator that defines the rational
    /// coordinates with \a lambda.
    ///
    /// @param[in] lambda the numerators that defines the rational
    /// coordinates with \a d.
    ///
    /// @param[in] r the remainder vector (r is independent from this
    /// basis B=(b[0],...,b[i])), which represents the displacement
    /// from p to this affine space.
    ///
    /// @return the point `p` such that `d * (p - o) = B lambda + r`,
    /// for `B` the vectors of this basis (arranged as rows in matrix)
    /// and `o` its origin.
    Point recompose( Scalar d, const Point& lambda,
                     const Point& r = Point::zero ) const
    {
      return first + recomposeVector( d, lambda, r );
    }

    /// Given (d, lambda, r), recompose the vector `w` such that `d *
    /// w = l[0]b[0] + ... l[i]b[i] + r`. Given only the rational
    /// coordinates (d,lambda), the vector is assumed to be parallel
    /// to this affine space (i.e. r == 0 ).
    ///
    /// @param[in] d the common denominator that defines the rational
    /// coordinates with \a lambda.
    ///
    /// @param[in] lambda the numerators that defines the rational
    /// coordinates with \a d.
    ///
    /// @param[in] r the remainder vector (r is independent from this
    /// basis B=(b[0],...,b[i])), which represents the displacement
    /// vector to this vector space.
    ///
    /// @return the point `p` such that `d * w = B lambda + r`,
    /// for `B` the vectors of this basis (arranged as rows in matrix)
    /// and `o` its origin.
    Point recomposeVector( Scalar d, const Point& lambda,
                           const Point& r = Point::zero ) const
    {
      Point w = r;
      for ( std::size_t i = 0; i < second.size(); i++ )
        w += lambda[ i ] * second[ i ];
      return w / d;
    }
    
    /// Decompose p as ` d * (p - o) = l[0]b[0] + ... l[i]b[i] + r`, where r
    /// is independent from this basis B=(b[0],...,b[i]).
    ///
    /// @param[in] p any point.
    ///
    /// @return (d,l,r) where d is a scalar, l/d are the rational
    /// coordinates of (p-o) in the basis, r is the remainer vector if p
    /// does not belong to the affine space.
    std::tuple< Scalar, Point, Point > decompose( const Point& p ) const
    {
      return decomposeVector( p - first );
    }

    /// Decompose w as ` d * w = l[0]b[0] + ... l[i]b[i] + r`, where r
    /// is independent from B=(b[0],...,b[i]).
    ///
    /// @param[in] w any vector.
    ///
    /// @return (d,l,r) where d is a scalar, l/d are the rational
    /// coordinates of w in the basis, r is the remainer vector if w
    /// does not belong to the affine space.
    std::tuple< Scalar, Point, Point > decomposeVector( Point w ) const
    {
      Point r;
      Scalar alphas = 1;
      for ( auto i = 0; i < second.size(); i++ )
        {
          std::pair< Scalar, Scalar > c
            = Affine::reduceVector( w, second[ i ], epsilon );
          for ( auto j = 0; j < i; j++ )
            r[ j ] *= c.first;
          r[ i ]  = c.second;
          alphas *= c.first;
        }
      return alphas >= 0
        ? std::make_tuple(  alphas,  r,  w )
        : std::make_tuple( -alphas, -r, -w );
    }

    /// Projects the range of points \a input onto the affine basis
    /// and outputs it in \a result. A consistent choice for
    /// ProjectedPoint is to match the affine dimension.
    /// 
    /// @tparam ProjectedPoint a type of point that matches the affine dimension.
    ///
    /// @param[out] result the range of projected points.
    /// @param[in]  input  the range of input points
    ///
    /// @return the maximum dilation of projected rational coordinates
    /// that was used to create integer coordinates.
    template <typename ProjectedPoint>
    Scalar projectPoints( std::vector< ProjectedPoint >& result,
                          const Points& input )
    {
      Scalar lcm = 1;
      std::vector< Scalar > denoms ( input.size() );
      std::vector< Point >  lambdas( input.size() );
      Point  r;
      // get points rational coordinates.
      for ( std::size_t i = 0; i < input.size(); i++ )
        std::tie( denoms[ i ], lambdas[ i ], r ) = decompose( input[ i ] );
      // compute ppcm
      for ( std::size_t i = 0; i < denoms.size(); i++ )
        {
          const Scalar d = denoms[ i ];
          if ( d == 1 ) continue;
          lcm = Affine::ScalarOps::lcmPositive( lcm, d );
        }
      // project points
      result.resize( input.size() );
      for ( std::size_t i = 0; i < input.size(); i++ )
        dilatedTransform( result[ i ], lambdas[ i ], lcm / denoms[ i ] );
      return lcm;
    }

    /// Transforms the type of an input point into another one.
    ///
    /// @tparam OtherPoint a type of point of dimension at most Point::dimension.
    /// @param[out] pp the output restricted point.
    /// @param[in]  p  the input point.
    template <typename OtherPoint>
    static
    void transform( OtherPoint& pp, const Point& p )
    {
      BOOST_STATIC_ASSERT( OtherPoint::dimension <= Point::dimension );
      typedef typename OtherPoint::Coordinate Scalar;
      for ( std::size_t i = 0; i < pp.dimension; ++i )
        pp[ i ] = Scalar( p[ i ] );
    }
      
    /// Transforms the type of an input point into another one, while
    /// dilating it by a factor \a m.
    ///
    /// @tparam OtherPoint a type of point of dimension at most Point::dimension.
    /// @param[out] pp the output restricted point.
    /// @param[in]  p  the input point.
    /// @param[in]  m  the dilation factor.
    template <typename OtherPoint>
    static
    void dilatedTransform( OtherPoint& pp, const Point& p, Scalar m )
    {
      BOOST_STATIC_ASSERT( OtherPoint::dimension <= Point::dimension );
      for ( std::size_t i = 0; i < pp.dimension; ++i )
        pp[ i ] = m * Scalar( p[ i ] );
    }
    
    /// @}

    void selfDisplay( std::ostream& out ) const
    {
      out << "[ AffineBasis o=" << first << " B=";
      for ( auto b : second ) std::cout << "\n  " << b;
      out << " ]";
    }
    
    // ----------------------- public data --------------------------
  public:
    /// @name public data
    /// @{

    Point  first;  ///< the origin of the affine basis
    Points second; ///< the vector basis
    double epsilon;///< the accepted value below which a floating-point number is 0.
    
    /// @}
    
  }; // struct AffineBasis
    
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

template <typename TPoint>
std::ostream&
operator<<( std::ostream& out, const DGtal::AffineBasis<TPoint>& B )
{
  B.selfDisplay( out );
  return out;
}


#endif // !defined AffineBasis_h

#undef AffineBasis_RECURSES
#endif // else defined(AffineBasis_RECURSES)
