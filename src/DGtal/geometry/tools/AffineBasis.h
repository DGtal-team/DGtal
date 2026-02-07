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
#include <type_traits>
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
  ///
  /// @note Affine basis can be computed in several forms, which are
  /// more or less suited according to the targeted use:
  ///
  /// - ECHELON_REDUCED: the basis A is in row echelon form, useful
  ///   for solving systems of the form Ax=b. A drawback is that
  ///   component values may increase quickly if we ask for integers.
  ///
  /// - SHORTEST_ECHELON_REDUCED: the basis A is in row echelon form,
  ///   but the computation first sorts the input vector from the
  ///   shortest to the longest vectors before Gauss elimination. It
  ///   is useful for solving systems of the form Ax=b. A drawback is
  ///   also that component values may increase quickly if we ask for
  ///   integers.
  ///
  /// - LLL_REDUCED (specific to lattice vectors): the basis A is not
  /// - in row echelon form but is made of almost the shortest
  /// - possible lattice vectors. It is implemented with the LLL
  /// - algorithm (see <a
  /// - href="https://en.wikipedia.org/wiki/Lenstra–Lenstra–Lovász_lattice_basis_reduction_algorithm">
  /// - Wikipedia LLL algorithm </a>). It is not handy for solving
  /// - systems but the basis vectors are in general much shorter than
  /// - in row echelon form.
  template < typename TPoint >
  struct AffineBasis
  {
    typedef AffineBasis<TPoint>        Self;
    typedef TPoint                     Point;
    typedef typename Point::Coordinate Scalar;
    typedef std::vector< Point >       Points;
    typedef AffineGeometry< Point >    Affine;

    enum struct Type {
      INVALID = 0,     ///< invalid basis
      ECHELON_REDUCED, ///< echelon matrix
      SHORTEST_ECHELON_REDUCED, ///< echelon matrix starting from shortest vectors
      LLL_REDUCED      ///< delta-LLL reduced matrix
    };

    // ----------------------- standard services --------------------------
  public:
    /// @name standard services
    /// @{

    /// Default constructor.
    /// This create the identity basis of the space.
    ///
    /// @param[in] tolerance the accepted oo-norm below which the
    /// vector is null (used only for points with float/double
    /// coordinates).
    AffineBasis( const double tolerance = 1e-12)
      : epsilon( tolerance )
    {
      first  = Point::zero;
      second.resize( Point::dimension );
      for ( auto k = 0; k < Point::dimension; k++ )
        second[ k ] = Point::base( k );
      _type = Type::SHORTEST_ECHELON_REDUCED;
    }

    /// Constructor from points.
    ///
    /// @param[in] points the range of points belonging to the affine space.
    ///
    /// @param[in] type if Type::ECHELON_REDUCED or
    /// Type::SHORTEST_ECHELON_REDUCED, reduces the basis so that it
    /// forms a echelon matrix, otherwise computes its
    /// delta-LLL-lattice.
    ///
    /// @param[in] delta the parameter \f$ \delta \f$ of
    /// LLL-algorithm, which should be between 0.25 and 1 (value 0.99
    /// is default in sagemath).
    ///
    /// @param[in] tolerance the accepted oo-norm below which the
    /// vector is null (used only for points with float/double
    /// coordinates).
    template <typename TInputPoint>
    AffineBasis( const std::vector< TInputPoint >& points,
                 AffineBasis::Type type,
                 const double delta = 0.99,
                 const double tolerance = 1e-12 )
      : epsilon( tolerance )
    {
      if ( points.size() == 0 ) return;
      first  = Affine::transform( points[ 0 ] );
      std::vector< TInputPoint > basis( points.size() - 1 );
      for ( std::size_t i = 0; i < basis.size(); i++ )
        basis[ i ] = ( points[ i+1 ] - first );
      initBasis( basis );
      reduce( type, delta );
    }

    /// Constructor from origin and basis.
    ///
    /// @param[in] origin the origin of the affine basis
    /// @param[in] basis the range of vectors forming the basis
    ///
    /// @param[in] type if Type::ECHELON_REDUCED or
    /// Type::SHORTEST_ECHELON_REDUCED, reduces the basis so that it
    /// forms a echelon matrix, otherwise computes its
    /// delta-LLL-lattice.
    ///
    /// @param[in] is_reduced when 'true', assumes that the given
    /// basis is already reduced, otherwise it forces the reduction of
    /// the basis.
    ///
    /// @param[in] delta the parameter \f$ \delta \f$ of
    /// LLL-algorithm, which should be between 0.25 and 1 (value 0.99
    /// is default in sagemath).
    ///
    /// @param[in] tolerance the accepted oo-norm below which the
    /// vector is null (used only for points with float/double
    /// coordinates).
    template <typename TInputPoint>
    AffineBasis( const TInputPoint& origin,
                 const std::vector<TInputPoint>& basis,
                 AffineBasis::Type type,
                 bool is_reduced = false,
                 const double delta = 0.99,
                 const double tolerance = 1e-12 )
      : epsilon( tolerance )
    {
      first = Affine::transform( origin );
      initBasis( basis );
      if ( ! is_reduced ) reduce( type, delta );
      else _type = type;
    }

    /// Creates an affine basis going through \a origin and orthogonal
    /// to the lattice vector \a normal.
    ///
    /// @param[in] origin the origin of the affine basis
    /// @param[in] normal the lattice normal vector.
    ///
    /// @param[in] type if Type::ECHELON_REDUCED or
    /// Type::SHORTEST_ECHELON_REDUCED, then the basis will be in
    /// echelon form, otherwise it will make the vectors as short as
    /// possible, but the matrix won't be in echelon form.
    ///
    /// @param[in] tolerance the accepted oo-norm below which the
    /// vector is null (used only for points with float/double
    /// coordinates).
    template <typename TInputPoint>
    AffineBasis( const TInputPoint& origin,
                 const TInputPoint& normal,
                 AffineBasis::Type type = Type::ECHELON_REDUCED,
                 const double tolerance = 1e-12 )
      : epsilon( tolerance )
    {
      first = Affine::transform( origin );
      // basis is shortened is type is LLL.
      second = Affine::orthogonalLatticeBasis( normal, type == Type::LLL_REDUCED );
      _type = ( type == Type::LLL_REDUCED )
        ? Type::LLL_REDUCED : Type::ECHELON_REDUCED;
    }

    /// Reduces the basis into a set of a linearly independent
    /// vectors, and in the desired reduced form.
    ///
    /// @param[in] type the desired type of matrix reduction.
    ///
    /// @param[in] delta the parameter \f$ \delta \f$ of
    /// LLL-algorithm, which should be between 0.25 and 1 (value 0.99
    /// is default in sagemath).
    ///
    void reduce( AffineBasis::Type type, double delta )
    {
      if ( type == Type::SHORTEST_ECHELON_REDUCED )
        sortBasis();
      if ( type == Type::ECHELON_REDUCED || type == Type::SHORTEST_ECHELON_REDUCED )
        reduceAsEchelon( type );
      else if ( type == Type::LLL_REDUCED )
        {
          std::vector< Point > X( second.size()+1 );
          X[ 0 ] = first;
          for ( auto i = 0; i < second.size(); i++ )
            X[ i+1 ] = second[ i ] + first;
          std::tie( first, second ) = AffineGeometry<Point>::affineBasis( X, epsilon );
          reduceAsLLL( delta, (Scalar) 0 );
        }
    }

    /// @returns the affine dimension of the basis
    /// @pre the basis is reduced.
    Dimension dimension() const
    {
      return second.size();
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

    /// @param[in] other an other affine basis
    ///
    /// @return 'true' if and only if every vector of 'this' basis is
    /// parallel to every vector of the \a other given basis.
    bool isParallel( const Self& other ) const
    {
      if ( dimension() != other.dimension() ) return false;
      if ( ( _type != Type::ECHELON_REDUCED )
           && ( _type != Type::SHORTEST_ECHELON_REDUCED ) )
        trace.error() << "[AffineBasis::isParallel] Requires type=*_ECHELON_REDUCED\n"
                      << " type=" << reductionTypeName() << "\n" ;
      for ( const auto& b : other.second )
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
    /// coordinates of (p-o) in the basis, r is the remainder vector if p
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
    /// coordinates of w in the basis, r is the remainder vector if w
    /// does not belong to the affine space.
    std::tuple< Scalar, Point, Point > decomposeVector( Point w ) const
    {
      Point r;
      Scalar alphas = 1;
      for ( auto i = 0; i < second.size(); i++ )
        {
          std::pair< Scalar, Scalar > c
            = Affine::reduceVector( w, second[ i ], i, epsilon );
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

    // ----------------------- debug and I/O services --------------------------
  public:
    /// @name debug and I/O services
    /// @{

    /// Displays this object on the output stream.
    ///
    /// @param[in,out] out any output stream.
    void selfDisplay( std::ostream& out ) const
    {
      out << "[ AffineBasis o=" << first
          << " type=" << reductionTypeName()
          << " B=";
      for ( auto b : second ) std::cout << "\n  " << b;
      out << " ]";
    }

    /// @return 'true' if and only if this object is in a consistent
    /// state. Here it means that the matrix is echelon or LLL-reduced.
    bool isValid() const
    {
      return _type != Type::INVALID;
    }

    /// @return the type of matrix as a string
    std::string reductionTypeName() const
    {
      if ( _type == Type::INVALID )              return "INVALID";
      else if ( _type == Type::ECHELON_REDUCED ) return "ECHELON_REDUCED";
      else if ( _type == Type::SHORTEST_ECHELON_REDUCED ) return "SHORTEST_ECHELON_REDUCED";
      else if ( _type == Type::LLL_REDUCED )     return "LLL_REDUCED";
      else return "";
    }
    /// @}

    // ----------------------- public data --------------------------
  public:
    /// @name public data
    /// @{

    Point  first;  ///< the origin of the affine basis
    Points second; ///< the vector basis
    double epsilon {1e-12};///< the accepted value below which a floating-point number is 0.
    AffineBasis::Type _type; ///< the type of reduction of the basis.
    /// @}

    // ----------------------- protected services --------------------------
  protected:
    /// @name protected services
    /// @{

    /// If the basis is an integer lattice, reduces the basis vectors
    /// by their gcd, otherwise normalize vectors to have 1 L2-norm.
    void normalize()
    {
      for ( auto& v : second )
        v = Affine::simplifiedVector( v );
    }

    /// Reduces the basis so that each basis vector is normalized,
    /// removes linearly dependent vectors, and builds a echelon matrix.
    void reduceAsEchelon( Type type )
    {
      std::vector< bool > is_independent( second.size(), false );
      std::vector< std::vector< Scalar > > U( second.size() );
      Dimension k = 0; // the current column to put in echelon form.
      for ( std::size_t i = 0; i < second.size(); i++ )
        {
          std::size_t row = findIndexWithSmallestNonNullComponent( k, i, second );
          if ( row != i && row != second.size() )
            std::swap( second[ i ], second[ row ] );
          Point& w            = second[ i ];
          // check if this vector is independent from the previous ones
          is_independent[ i ] = true;
          for ( std::size_t j = 0; j < i; j++ )
            if ( is_independent[ j ] )
              Affine::reduceVector( w, second[ j ], epsilon );
          if ( ! Affine::ScalarOps::isNonZero( w.normInfinity(), epsilon ) )
            is_independent[ i ] = false; // not independent, forget it
          else
            { // independent, make sure it is reduced.
              w = Affine::simplifiedVector( w );
              k++;
            }
        }
      Points new_basis;
      for ( std::size_t i = 0; i < second.size(); i++ )
        if ( is_independent[ i ] )
          new_basis.push_back( second[ i ] );
      std::swap( second, new_basis );
      orderEchelonBasis();
      _type = type;
    }

    /// Guarantees that the basis is in echelon form.
    void orderEchelonBasis()
    {
      auto compare = [this] ( const Point& v, const Point& w ) -> bool
      {
        // Note: curiously std::sort sometimes test v against itself
        // and must return false in this case.
        for ( auto k = 0; k < Point::dimension; ++k )
          {
            bool v_non_null = Affine::ScalarOps::isNonZero( v[ k ], epsilon );
            bool w_non_null = Affine::ScalarOps::isNonZero( w[ k ], epsilon );
            if ( v_non_null && ! w_non_null )      return true;
            else if ( ! v_non_null && w_non_null ) return false;
            else if ( v_non_null && w_non_null )   return false; // ==
          }
        return false; // 0 == 0
      };
      std::sort( second.begin(), second.end(), compare );
    }

    /// Reduces the basis so that each basis vector is normalized,
    /// then computes its delta-LLL-reduction lattice, and removes
    /// linearly dependent vectors.
    ///
    /// @param[in] delta the parameter \f$ \delta \f$ of
    /// LLL-algorithm, which should be between 0.25 and 1 (value 0.99
    /// is default in sagemath).
    void reduceAsLLL( double delta, Scalar )
    {
      if constexpr( std::is_floating_point< Scalar >::value == true )
        {
          trace.error() << "[AffineBasis::reduceAsLLL]"
                        << " It has no meaning to use LLL algorithm on matrix with double coefficients\n";
          _type = Type::INVALID;
          return;
        }
      for ( auto& v : second )
        v = Affine::simplifiedVector( v );
      std::vector< std::vector< Scalar > > B( second.size() );
      for ( auto i = 0; i < second.size(); i++ )
        {
          B[ i ] = std::vector< Scalar >( Point::dimension );
          for ( auto j = 0; j < Point::dimension; j++ )
            B[ i ][ j ] = second[ i ][ j ];
        }
      functions::reduceBasisWithLLL( B, delta );
      /// keep only independent vectors in basis
      second.clear();
      for ( std::size_t i = 0; i < B.size(); i++ )
        {
          Point b;
          for ( auto j = 0; j < Point::dimension; j++ )
            b[ j ] = B[ i ][ j ];
          if ( b != Point::zero )
            second.push_back( Affine::simplifiedVector( b ) );
        }
      _type = Type::LLL_REDUCED;
    }

    /// Removes null vectors from input set of vectors and sets the
    /// initial basis.
    ///
    /// @tparam TInputPoint the type of input points
    /// @param[in] basis a set of arbitrary vectors
    template <typename TInputPoint>
    void initBasis( const std::vector<TInputPoint>& basis )
    {
      second.reserve( basis.size() );
      for ( auto i = 0; i < basis.size(); i++ )
        {
          Point b = Affine::transform( basis[ i ] );
          if ( b != Point::zero ) second.push_back( b );
        }
    }

    /// Simplifies vectors, removes duplicates and puts smallest
    /// candidate basis vectors before longest.
    void sortBasis()
    {
      // Reduces all vectors
      normalize();
      // Purge duplicates
      std::sort( second.begin(), second.end() );
      second.erase( std::unique( second.begin(), second.end() ), second.end() );
      // Sort according to size of components.
      auto compare = []( const Point& u, const Point& v ) -> bool
      {
        const auto n1_u = u.norm1();
        const auto n1_v = v.norm1();
        if ( n1_u < n1_v ) return true;
        else if ( n1_v < n1_u ) return false;
        const auto noo_u = u.normInfinity();
        const auto noo_v = v.normInfinity();
        if ( noo_u < noo_v ) return true;
        else if ( noo_v < noo_u ) return false;
        return u < v;
      };
      std::sort( second.begin(), second.end(), compare );
    }

    /// Given a range of points \a basis, starting from rank \a i,
    /// find the index of the point with lowest non null k-th
    /// coefficient in absolute value, or basis.size() if every point
    /// had its k-th component null.
    ///
    /// @param[in] k the component/coordinate of interest
    /// @param[in] i the starting index
    /// @param[in] basis a range of points/vectors
    ///
    /// @return starting from rank \a i, the index of the point with lowest non null k-th
    /// coefficient in absolute value, or basis.size() if every point
    /// had its k-th component null.
    std::size_t
    findIndexWithSmallestNonNullComponent( Dimension k,
                                           std::size_t i,
                                           const std::vector< Point >& basis )
    {
      ASSERT( ! basis.empty() );
      ASSERT( k < Point::dimension );
      std::size_t index = i;
      Scalar      v     = 0;
      for ( ; index < basis.size(); index++ )
        {
          v = abs( basis[ index ][ k ] );
          if ( Affine::ScalarOps::isNonZero( v, epsilon ) )
            break;
        }
      for ( auto j = index + 1; j < basis.size(); j++ )
        {
          Scalar vj = abs( basis[ j ][ k ] );
          if ( vj != 0 && vj < v )
            {
              index = j;
              v     = vj;
            }
        }
      return index;
    }

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
