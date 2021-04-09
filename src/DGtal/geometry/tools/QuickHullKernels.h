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
 * @file QuickHullKernels.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/07/11
 *
 * Header file for module QuickHullKernels.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(QuickHullKernels_RECURSES)
#error Recursive header files inclusion detected in QuickHullKernels.h
#else // defined(QuickHullKernels_RECURSES)
/** Prevents recursive inclusion of headers. */
#define QuickHullKernels_RECURSES

#if !defined QuickHullKernels_h
/** Prevents repeated inclusion of headers. */
#define QuickHullKernels_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <vector>
#include <array>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/math/linalg/SimpleMatrix.h"

namespace DGtal
{
  namespace detail {

#ifdef WITH_BIGINTEGER
    /// ------------- GMP SPECIALIZED SERVICES ----------------------------
    
    /// @param[inout] n the (initialized) big integer to set
    /// @param[in] ull a signed long long integer to assign to \a n.
    static void mpz_set_sll(mpz_t n, long long sll)
    {
      mpz_set_si(n, (int)(sll >> 32));     /* n = (int)sll >> 32 */
      mpz_mul_2exp(n, n, 32 );             /* n <<= 32 */
      mpz_add_ui(n, n, (unsigned int)sll); /* n += (unsigned int)sll */
    }
    
    /// @param[inout] n the (initialized) big integer to set
    /// @param[in] ull an unsigned long long integer to assign to \a n.
    static void mpz_set_ull(mpz_t n, unsigned long long ull)
    {
      mpz_set_ui(n, (unsigned int)(ull >> 32)); /* n = (unsigned int)(ull >> 32) */
      mpz_mul_2exp(n, n, 32);                   /* n <<= 32 */
      mpz_add_ui(n, n, (unsigned int)ull);      /* n += (unsigned int)ull */
    }      

    /// Conversion to uint64 is tricky and not native for GMP.
    /// @param n any number
    /// @return its uint64 representation.
    static unsigned long long mpz_get_ull(mpz_t n)
    {
      unsigned int lo, hi;
      mpz_t tmp;
      mpz_init( tmp );
      mpz_mod_2exp( tmp, n, 64 );   /* tmp = (lower 64 bits of n) */
      lo = mpz_get_ui( tmp );       /* lo = tmp & 0xffffffff */ 
      mpz_div_2exp( tmp, tmp, 32 ); /* tmp >>= 32 */
      hi = mpz_get_ui( tmp );       /* hi = tmp & 0xffffffff */
      mpz_clear( tmp );
      return (((unsigned long long)hi) << 32) + lo;
    }
    
    /// Conversion to int64 is tricky and not native for GMP.
    /// @param n any number
    /// @return its int64 representation.
    static long long mpz_get_sll(mpz_t n)
    {
      return (long long)mpz_get_ull(n); /* just use unsigned version */
    }
#endif
    
    /// ------------- INTEGER/POINT CONVERSION SERVICES --------------------
    
    /// Allows seamless conversion of integral types and lattice
    /// points, while checking for errors when going from a more
    /// precise to a less precise type.
    ///
    /// Generic version allowing only the identity cast.
    ///
    /// @tparam dim static constant of type DGtal::Dimension that
    /// specifies the static  dimension of the space and thus the number
    /// of elements  of the Point or Vector.
    ///
    /// @tparam TInteger an integral type, a model of concepts::CInteger
    template < DGtal::Dimension dim,
               typename TInteger >
    struct IntegerConverter {
      BOOST_CONCEPT_ASSERT(( concepts::CInteger<TInteger> ));
      typedef TInteger Integer;

      /// @param i any integer
      /// @return the same integer
      static Integer cast( Integer i ) 
      {
        return i;
      }

      /// Conversion of a lattice point.
      ///
      /// @param p any point
      /// @return the same point
      static
      PointVector< dim, Integer >
      cast( PointVector< dim, Integer > p )
      {
        return p;
      }
    };

    /// Allows seamless conversion of integral types and lattice
    /// points, while checking for errors when going from a more
    /// precise to a less precise type.
    ///
    /// Specialized version for int32_t.
    ///
    /// @tparam dim static constant of type DGtal::Dimension that
    /// specifies the static  dimension of the space and thus the number
    /// of elements  of the Point or Vector.
    template < DGtal::Dimension dim >
    struct IntegerConverter< dim, DGtal::int32_t > {
      typedef DGtal::int32_t Integer;

      /// @param i any integer
      /// @return the same integer
      static DGtal::int32_t cast( DGtal::int32_t i ) 
      {
        return i;
      }

      /// Conversion of a lattice point.
      ///
      /// @param p any point
      /// @return the same point
      static
      PointVector< dim, DGtal::int32_t >
      cast( PointVector< dim, DGtal::int32_t > p )
      {
        return p;
      }

      /// @param i any integer
      /// @return the same integer
      static DGtal::int32_t cast( DGtal::int64_t i ) 
      {
        DGtal::int32_t r = DGtal::int32_t( i );
        if ( DGtal::int64_t( r ) != i )
          trace.warning() << "Bad integer conversion: " << i << " -> " << r
                          << std::endl;
        return r;
      }

      /// Conversion of a lattice point.
      ///
      /// @param p any point
      /// @return the same point
      static
      PointVector< dim, DGtal::int32_t >
      cast( PointVector< dim, DGtal::int64_t > p )
      {
        PointVector< dim, DGtal::int32_t > q;
        for ( DGtal::Dimension i = 0; i < dim; i++ )
          q[ i ] = cast( p[ i ] );
        return q;
      }
      
#ifdef WITH_BIGINTEGER
      /// @param i any integer
      /// @return the same integer
      static DGtal::int32_t cast( DGtal::BigInteger i ) 
      {
        DGtal::int32_t r = i.get_si();
        if ( DGtal::BigInteger( r ) != i )
          trace.warning() << "Bad integer conversion: " << i << " -> " << r
                          << std::endl;
        return r;
      }

      /// Conversion of a lattice point.
      ///
      /// @param p any point
      /// @return the same point
      static
      PointVector< dim, DGtal::int32_t >
      cast( PointVector< dim, DGtal::BigInteger > p )
      {
        PointVector< dim, DGtal::int32_t > q;
        for ( DGtal::Dimension i = 0; i < dim; i++ )
          q[ i ] = cast( p[ i ] );
        return q;
      }

#endif
    };
    

    /// Allows seamless conversion of integral types and lattice
    /// points, while checking for errors when going from a more
    /// precise to a less precise type.
    ///
    /// Specialized version for int64_t.
    ///
    /// @tparam dim static constant of type DGtal::Dimension that
    /// specifies the static  dimension of the space and thus the number
    /// of elements  of the Point or Vector.
    template < DGtal::Dimension dim >
    struct IntegerConverter< dim, DGtal::int64_t > {
      typedef DGtal::int64_t Integer;

      /// @param i any integer
      /// @return the same integer
      static DGtal::int64_t cast( DGtal::int32_t i ) 
      {
        return i;
      }

      /// Conversion of a lattice point.
      ///
      /// @param p any point
      /// @return the same point
      static
      PointVector< dim, DGtal::int64_t >
      cast( PointVector< dim, DGtal::int32_t > p )
      {
        PointVector< dim, DGtal::int64_t > q;
        for ( DGtal::Dimension i = 0; i < dim; i++ )
          q[ i ] = cast( p[ i ] );
        return q;
      }

      /// @param i any integer
      /// @return the same integer
      static DGtal::int64_t cast( DGtal::int64_t i ) 
      {
        return i;
      }

      /// Conversion of a lattice point.
      ///
      /// @param p any point
      /// @return the same point
      static
      PointVector< dim, DGtal::int64_t >
      cast( PointVector< dim, DGtal::int64_t > p )
      {
        return p;
      }
      
#ifdef WITH_BIGINTEGER
      /// @param i any integer
      /// @return the same integer
      static DGtal::int64_t cast( DGtal::BigInteger i ) 
      {
        DGtal::int64_t r = mpz_get_sll( i.get_mpz_t() );
        DGtal::BigInteger tmp;
        mpz_set_sll( tmp.get_mpz_t(), r );
        if ( tmp != i )
          trace.warning() << "Bad integer conversion: " << i << " -> " << r
                          << std::endl;
        return r;
      }

      /// Conversion of a lattice point.
      ///
      /// @param p any point
      /// @return the same point
      static
      PointVector< dim, DGtal::int64_t >
      cast( PointVector< dim, DGtal::BigInteger > p )
      {
        PointVector< dim, DGtal::int64_t > q;
        for ( DGtal::Dimension i = 0; i < dim; i++ )
          q[ i ] = cast( p[ i ] );
        return q;
      }
      
#endif
    };
    

#ifdef WITH_BIGINTEGER
    /// Allows seamless conversion of integral types and lattice
    /// points, while checking for errors when going from a more
    /// precise to a less precise type.
    ///
    /// Specialized version for BigInteger
    ///
    /// @tparam dim static constant of type DGtal::Dimension that
    /// specifies the static  dimension of the space and thus the number
    /// of elements  of the Point or Vector.
    template < DGtal::Dimension dim >
    struct IntegerConverter< dim, DGtal::BigInteger > {
      typedef DGtal::BigInteger Integer;

      /// @param i any integer
      /// @return the same integer
      static DGtal::BigInteger cast( DGtal::int32_t i ) 
      {
        return DGtal::BigInteger( i );
      }

      /// Conversion of a lattice point.
      ///
      /// @param p any point
      /// @return the same point
      static
      PointVector< dim, DGtal::BigInteger >
      cast( PointVector< dim, DGtal::int32_t > p )
      {
        PointVector< dim, DGtal::BigInteger > q;
        for ( DGtal::Dimension i = 0; i < dim; i++ )
          q[ i ] = cast( p[ i ] );
        return q;
      }

      /// @param i any integer
      /// @return the same integer
      static DGtal::BigInteger cast( DGtal::int64_t i ) 
      {
        DGtal::BigInteger tmp;
        mpz_set_sll( tmp.get_mpz_t(), i );
        return tmp;
      }

      /// Conversion of a lattice point.
      ///
      /// @param p any point
      /// @return the same point
      static
      PointVector< dim, DGtal::BigInteger >
      cast( PointVector< dim, DGtal::int64_t > p )
      {
        PointVector< dim, DGtal::BigInteger > q;
        for ( DGtal::Dimension i = 0; i < dim; i++ )
          q[ i ] = cast( p[ i ] );
        return q;
      }
      
      /// @param i any integer
      /// @return the same integer
      static DGtal::BigInteger cast( DGtal::BigInteger i ) 
      {
        return i;
      }

      /// Conversion of a lattice point.
      ///
      /// @param p any point
      /// @return the same point
      static
      PointVector< dim, DGtal::BigInteger >
      cast( PointVector< dim, DGtal::BigInteger > p )
      {
        return p;
      }
      
    };
#endif
    
    // ------------------------ OTHER SERVICES -----------------------
    
    /// Compute the center of the bounding box containing the given points.
    /// @tparam Point any type of points
    /// @param[in] points a range of points
    /// @return the (approximate center of the given range.
    /// @note Complexity is O(n), if n is the size of \a points.
    template < typename Point >
    Point center( const std::vector< Point >& points )
    {
      if ( points.empty() ) return Point::zero;
      Point l = points[ 0 ];
      Point u = l;
      for ( const auto& p : points ) {
        l = l.inf( p );
        u = u.sup( p );
      }
      return Point( ( l + u ) / 2 );
    }

    /// Transform an input range of points into an output range with a
    /// conversion function, and possibly removes duplicates in the
    /// output range. Used as preprocessing of QuickHull algorithm.
    ///
    /// @tparam OutputValue any value that is LessThanComparable,
    /// Assignable, CopyConstructible, DefaultConstructible.
    ///
    /// @tparam ConversionFct is a functor from input values to OutputValue.
    ///
    /// @tparam ForwardIterator is a forward iterator on some input values.
    ///
    /// @param[out] output_values a range of points converted from the
    /// input range, with duplicates possibly removed according to
    /// \a remove_duplicates
    ///
    /// @param[out] input2output the surjective mapping between the
    /// input range and the output range used for computation.
    ///
    /// @param[out] output2input the injective mapping between the
    /// output range and the input range.
    ///
    /// @param[in] itb,ite a range of values.
    ///
    /// @param[in] F a function that may transform input values to output values
    ///
    /// @param[in] remove_duplicates when 'true' remove duplicate
    /// values, otherwise, `output_values.size()` is equal to the size
    /// of the range itb,ite.
    template < typename OutputValue,
               typename ForwardIterator,
               typename ConversionFct >
    void transform( std::vector< OutputValue >& output_values,
                    std::vector< std::size_t >& input2output,
                    std::vector< std::size_t >& output2input,
                    ForwardIterator itb, ForwardIterator ite,
                    const ConversionFct& F,
                    bool remove_duplicates )
    {
      typedef std::size_t Size;
      std::vector< OutputValue > input;
      while ( itb != ite ) {
        const auto ip = *itb++;
        input.push_back( F( ip ) );
      }
      if ( ! remove_duplicates ) {
        output_values.swap( input );
        input2output.resize( input.size() );
        output2input.resize( input.size() );
        for ( Size i = 0; i < input.size(); ++i )
          input2output[ i ] = output2input[ i ] = i;
      }
      else {
        output_values.clear();
        std::vector< std::size_t > i2c_sort( input.size() );
        input2output.resize( input.size() );
        for ( Size i = 0; i < input.size(); i++ ) i2c_sort[ i ] = i;
        // indirect sort
        std::sort( i2c_sort.begin(), i2c_sort.end(),
                   [&input] ( Size i, Size j ) { return input[ i ] < input[ j ]; } );
        output_values.resize( input.size() );
        output_values[ 0 ] = input[ i2c_sort[ 0 ] ];
        input2output[ i2c_sort[ 0 ] ] = 0;
        Size j = 0;
        for ( Size i = 1; i < input.size(); i++ ) {
          if ( input[ i2c_sort[ i-1 ] ] != input[ i2c_sort[ i ] ] )
            output_values[ ++j ] = input[ i2c_sort[ i ] ];
          input2output[ i2c_sort[ i ] ] = j;
        }
        output_values.resize( j+1 );
        output2input.resize( output_values.size() );
        for ( Size i = 0; i < input2output.size(); i++ )
          output2input[ input2output[ i ] ] = i;
      }      
    }

  } // namespace detail {


  /////////////////////////////////////////////////////////////////////////////
  // template class ConvexHullCommonKernel
  /**
     Description of template class 'ConvexHullCommonKernel' <p> \brief
     Aim: the common part of all geometric kernels for computing the
     convex hull or Delaunay triangulation of a range of points.

     @tparam dim the dimension of the space that is used for computing
     the convex hull (either the same as input points for convex hull,
     or one more than input points for Delaunay triangulation)

     @tparam TCoordinateInteger the integer type that represents
     coordinates of lattice points, a model of concepts::CInteger.

     @tparam TInternalInteger the integer type that is used for
     internal computations of above/below plane tests, a model of
     concepts::CInteger. Must be at least as precise as
     TCoordinateInteger.
  */
  template < Dimension dim,
             typename TCoordinateInteger  = DGtal::int64_t,
             typename TInternalInteger = DGtal::int64_t >
  struct ConvexHullCommonKernel {
    BOOST_CONCEPT_ASSERT(( concepts::CInteger<TCoordinateInteger> ));
    BOOST_CONCEPT_ASSERT(( concepts::CInteger<TInternalInteger> ));
    typedef TCoordinateInteger         CoordinateInteger;
    typedef TInternalInteger           InternalInteger;
    //typedef CoordinateInteger          Scalar;
    typedef CoordinateInteger          CoordinateScalar;
    typedef InternalInteger            InternalScalar;
    //typedef DGtal::PointVector< dim, CoordinateInteger > Point;
    //typedef DGtal::PointVector< dim, CoordinateInteger > Vector;
    typedef DGtal::PointVector< dim, CoordinateInteger > CoordinatePoint;
    typedef DGtal::PointVector< dim, CoordinateInteger > CoordinateVector;
    typedef DGtal::PointVector< dim, InternalInteger >   InternalPoint;
    typedef DGtal::PointVector< dim, InternalInteger >   InternalVector;
    typedef std::size_t                Size;
    typedef Size                       Index;
    typedef std::vector< Index >       IndexRange;
    typedef std::array< Index, dim >   CombinatorialPlaneSimplex;
    static const Dimension dimension = dim;

    /// Converter to outer coordinate integers or lattice points / vector
    typedef detail::IntegerConverter< dim, CoordinateInteger > Outer; 
    /// Converter to inner internal integers or lattice points / vector
    typedef detail::IntegerConverter< dim, InternalInteger >   Inner;
    
    class HalfSpace {
      friend class ConvexHullCommonKernel< dim >;
      InternalVector N; ///< the normal vector
      InternalScalar c; ///< the intercept
      HalfSpace( const InternalVector& aN, const InternalScalar aC )
        : N( aN ), c( aC ) {}
    public:
      HalfSpace() = default;
      const InternalVector& internalNormal() const    { return N; }
      InternalScalar internalIntercept() const { return c; }
    };
    
    /// Default constructor.
    ConvexHullCommonKernel() = default;

    /// Computes an halfspace from dimension points specified by \a
    /// simplex with vertices in a range \a vpoints of Point. It is
    /// oriented such that the point of index \a idx_below is included
    /// in the half-space (i.e. below).
    ///
    /// @param[in] vpoints a range of points over which the simplex is defined.
    /// @param[in] simplex a range of dimension indices of points defining an hyperplane.
    /// @param[in] idx_below the index of a p-oint that is below the hyperplane.
    ///
    /// @return the corresponding halfspace (has null normal vector if
    /// simplex was not full dimensional)
    HalfSpace
    compute( const std::vector< CoordinatePoint >& vpoints,
             const CombinatorialPlaneSimplex& simplex,
             Index idx_below )
    {
      HalfSpace hs = compute( vpoints, simplex );
      if ( hs.N != InternalVector::zero ) 
        {
          const InternalPoint  ip = Inner::cast( vpoints[ idx_below ] );
          const InternalScalar nu = hs.N.dot( ip );
          //const Scalar nu = hs.N.dot( vpoints[ idx_below ] );
          if ( nu > hs.c ) { hs.N = -hs.N; hs.c = -hs.c; }
        }
      return hs;
    }

    /// Computes an halfspace from dimension points specified by \a
    /// simplex with vertices in a range \a vpoints of
    /// Point. Orientation is induced by the order of the points. If
    /// the simplex is degenrated, the half-space is invalid and has
    /// null normal.
    ///
    /// @param[in] vpoints a range of points over which the simplex is defined.
    /// @param[in] simplex a range of dimension indices of points defining an hyperplane.
    ///
    /// @return the corresponding halfspace (has null normal vector if
    /// simplex was not full dimensional)
    HalfSpace
    compute( const std::vector< CoordinatePoint >& vpoints,
             const CombinatorialPlaneSimplex& simplex )
    {
      typedef DGtal::SimpleMatrix< InternalScalar, dimension, dimension > Matrix;
      Matrix A;
      InternalVector N;
      InternalScalar c;
      for ( Dimension i = 1; i < dimension; i++ )
        for ( Dimension j = 0; j < dimension; j++ )
          A.setComponent( i-1, j,
                          Inner::cast( vpoints[ simplex[ i ] ][ j ]
                                       - vpoints[ simplex[ 0 ] ][ j ] ) );
      for ( Dimension j = 0; j < dimension; j++ )
        N[ j ] = A.cofactor( dimension-1, j );
      const InternalPoint ip = Inner::cast( vpoints[ simplex[ 0 ] ] );
      // c = N.dot( vpoints[ simplex[ 0 ] ] );
      return HalfSpace { N, N.dot( ip ) };
    }
    
    /// @param H the half-space
    /// @return the normal to this facet.
    CoordinateVector normal( const HalfSpace& H ) const
    {
      return Outer::cast( H.N );
    }

    /// @param H the half-space
    /// @return the intercept of this facet.
    CoordinateScalar intercept( const HalfSpace& H ) const
    {
      return Outer::cast( H.c );
    }
    
    /// Equivalent of the dot product of the normals of the half-spaces.
    ///
    /// @param H1 an half-space
    /// @param H2 an half-space
    ///
    /// @return a positive scalar if both half-spaces points to to the
    /// same hemisphere, a negative scalar if they point to opposite
    /// hemispheres, and zero if they are orthogonal.
    InternalScalar dot( const HalfSpace& H1, const HalfSpace& H2 ) const
    {
      return H1.N.dot( H2.N );
    }

    /// @param H1 an half-space
    /// @param H2 an half-space
    ///
    /// @return 'true' if the half-spaces have the smae members.
    ///
    /// @note two half-spaces may be not equal but may represent the
    /// same set of points. For instance `H1={{1,0},3}` and
    /// `H1={{2,0},6}`.
    bool equal( const HalfSpace& H1, const HalfSpace& H2 ) const
    {
      return H1.c == H2.c && H1.N == H2.N;
    }
    
    /// @param H the half-space
    /// @param p any point
    /// @return the (signed) height of \a p wrt this plane.
    InternalScalar height( const HalfSpace& H, const CoordinatePoint& p ) const
    { return H.N.dot( Inner::cast( p ) ) - H.c; }

    /// @param H the half-space
    /// @param p any point
    /// @return the volume of the vectors spanned by the simplex and this point.
    InternalScalar volume( const HalfSpace& H, const CoordinatePoint& p ) const
    {
      InternalScalar v = height( H, p );
      return v < InternalScalar( 0 ) ? -v : v;
    }

    /// @param H the half-space
    /// @param p any point
    /// @return 'true' iff p is strictly above this plane (so in direction N ).
    bool above( const HalfSpace& H, const CoordinatePoint& p ) const
    { return height( H, p ) > 0; }

    /// @param H the half-space
    /// @param p any point
    /// @return 'true' iff p is above or lies on this plane (so in direction N ).
    bool aboveOrOn( const HalfSpace& H, const CoordinatePoint& p ) const
    { return height( H, p ) >= 0; }

    /// @param H the half-space
    /// @param p any point
    /// @return 'true' iff p lies on this plane.
    bool on( const HalfSpace& H, const CoordinatePoint& p ) const
    { return height( H, p ) == 0; } 
    
    
  }; //   template < Dimension dim >  struct ConvexHullIntegralKernel {


  
  /////////////////////////////////////////////////////////////////////////////
  // template class ConvexHullIntegralKernel
  /**
     Description of template class 'ConvexHullIntegralKernel' <p>
     \brief Aim: a geometric kernel to compute the convex hull of
     digital points with integer-only arithmetic.

     @see \ref moduleQuickHull

     @tparam dim the dimension of the space of processed points.
  */
  template < Dimension dim >
  struct ConvexHullIntegralKernel : public ConvexHullCommonKernel< dim >
  {
    typedef ConvexHullCommonKernel< dim > Base;
    // inheriting types
    // using typename Base::Point;
    // using typename Base::Vector;
    // using typename Base::Scalar;
    using typename Base::CoordinatePoint;
    using typename Base::CoordinateVector;
    using typename Base::CoordinateScalar;
    using typename Base::InternalPoint;
    using typename Base::InternalVector;
    using typename Base::InternalScalar;
    using typename Base::Size;
    using typename Base::Index;
    using typename Base::IndexRange;
    using typename Base::CombinatorialPlaneSimplex;
    using typename Base::HalfSpace;
    // inheriting constants
    using Base::dimension;
    // inheriting methods
    using Base::compute;
    using Base::normal;
    using Base::intercept;
    using Base::dot;
    using Base::equal;
    using Base::height;
    using Base::volume;
    using Base::above;
    using Base::aboveOrOn;
    using Base::on;
    
    /// Default constructor.
    ConvexHullIntegralKernel() = default;

    /// @return 'true' if a kernel may induce infinite facets. This is
    /// typically the case of a Delaunay computation kernel, which
    /// casts points in higher dimension.
    bool hasInfiniteFacets() const
    { return false; }

    /// @param[in] hs an half-space corresponding to a facet.
    ///
    /// @return 'true' if the facet associated to this half-space
    /// corresponds to an infinite facet.
    bool isHalfSpaceFacetInfinite( const HalfSpace& hs ) const
    {
      (void) hs; // unused parameter
      return false;
    }
    
    /// Transforms a range \a input_points of input points to a range
    /// \a processed_points of points adapted to a processing by
    /// QuickHull convex hull algorithm. Keep the mapping information
    /// between input and processed points.
    ///
    /// @tparam InputPoint any model of point whose components are
    /// convertible to Scalar.
    ///
    /// @param[out] processed_points the range of points prepared for
    /// a process by QuickHull.
    /// 
    /// @param[out] input2comp the surjective mapping between
    /// the \a input_points range and the \a processed_points range
    /// used for computation.
    ///
    /// @param[out] comp2input the injective mapping between the 
    /// \a processed_points range used for computation and the \a
    /// input_points range.
    ///
    /// @param[in] input_points the range of input points.
    ///
    /// @param[in] remove_duplicates when 'true', this method removes possible
    /// duplicates in \a input_points and \a processed_points may thus
    /// be of smaller size, otherwise, when 'false', it means that
    /// there are no duplicates in \a input_points.
    template < typename InputPoint>
    void makeInput( std::vector< CoordinatePoint >& processed_points,
                    IndexRange& input2comp, IndexRange& comp2input,
                    const std::vector< InputPoint >& input_points,
                    bool remove_duplicates )
    {
      const auto F = [&] ( InputPoint input ) -> CoordinatePoint
        {
          CoordinatePoint p;
          for ( Dimension i = 0; i < dimension; i++ )
            p[ i ] = CoordinateScalar( input[ i ] );
          return p;
        };
      DGtal::detail::transform( processed_points, input2comp, comp2input,
                                input_points.cbegin(), input_points.cend(),
                                F, remove_duplicates );
    }

    /// @tparam OutputPoint a model of point such that processing type
    /// Point is convertible to it.
    template < typename OutputPoint>
    void convertPointTo( const CoordinatePoint& p, OutputPoint& out_p ) const
    {
      for ( Dimension k = 0; k < dimension; k++ )
        out_p[ k ] = p[ k ];
    }

  }; //   template < Dimension dim >  struct ConvexHullIntegralKernel {


  /////////////////////////////////////////////////////////////////////////////
  // template class DelaunayIntegralKernel
  /**
     Description of template class 'DelaunayIntegralKernel' <p> \brief
     Aim: a geometric kernel to compute the Delaunay triangulation of
     digital points with integer-only arithmetic. It casts lattice
     point into a higher dimensional space and computes its convex
     hull. Facets pointing toward the bottom form the simplices of the
     Delaunay triangulation.

     @see \ref moduleQuickHull

     @tparam dim the dimension of the space of processed points.
  */
  template < Dimension dim >
  struct DelaunayIntegralKernel : public ConvexHullCommonKernel< dim+1 >
  {
    typedef ConvexHullCommonKernel< dim+1 > Base;
    // inheriting types
    // using typename Base::Point;
    // using typename Base::Vector;
    // using typename Base::Scalar;
    using typename Base::CoordinatePoint;
    using typename Base::CoordinateVector;
    using typename Base::CoordinateScalar;
    using typename Base::InternalPoint;
    using typename Base::InternalVector;
    using typename Base::InternalScalar;
    using typename Base::Size;
    using typename Base::Index;
    using typename Base::IndexRange;
    using typename Base::CombinatorialPlaneSimplex;
    using typename Base::HalfSpace;
    // inheriting constants
    using Base::dimension;
    // inheriting methods
    using Base::compute;
    using Base::normal;
    using Base::intercept;
    using Base::dot;
    using Base::equal;
    using Base::height;
    using Base::volume;
    using Base::above;
    using Base::aboveOrOn;
    using Base::on;

    /// Default constructor.
    DelaunayIntegralKernel() = default;

    /// @return 'true' if a kernel may induce infinite facets. This is
    /// typically the case of a Delaunay computation kernel, which
    /// casts points in higher dimension.
    bool hasInfiniteFacets() const
    { return true; }

    /// @param[in] hs an half-space corresponding to a facet.
    ///
    /// @return 'true' if the facet associated to this half-space
    /// correspond to an infinite facet.
    bool isHalfSpaceFacetInfinite( const HalfSpace& hs ) const
    {
      return hs.internalNormal()[ dimension - 1 ] >= InternalScalar( 0 );
    }
    
    /// Transforms a range \a input_points of input points to a range
    /// \a processed_points of points adapted to a processing by
    /// QuickHull convex hull algorithm. Keep the mapping information
    /// between input and processed points. This kernel transforms
    /// `dim`-dimensional into `dim+1`-dimensional points, where the
    /// last coordinate lies on a paraboloid. This is the classical
    /// way for computing a Delaunay triangulation as the convex hull
    /// of the points onto a paraboloid of higher dimension.
    ///
    /// @tparam InputPoint any model of point whose components are
    /// convertible to Scalar.
    ///
    /// @param[out] processed_points the range of points prepared for
    /// a process by QuickHull.
    /// 
    /// @param[out] input2comp the surjective mapping between
    /// the \a input_points range and the \a processed_points range
    /// used for computation.
    ///
    /// @param[out] comp2input the injective mapping between the 
    /// \a processed_points range used for computation and the \a
    /// input_points range.
    ///
    /// @param[in] input_points the range of input points.
    ///
    /// @param[in] remove_duplicates when 'true', this method removes possible
    /// duplicates in \a input_points and \a processed_points may thus
    /// be of smaller size, otherwise, when 'false', it means that
    /// there are no duplicates in \a input_points.
    template < typename InputPoint>
    void makeInput( std::vector< CoordinatePoint >& processed_points,
                    IndexRange& input2comp, IndexRange& comp2input,
                    const std::vector< InputPoint >& input_points,
                    bool remove_duplicates )
    {
      const auto F = [&] ( InputPoint input ) -> CoordinatePoint
        {
          CoordinatePoint p;
          CoordinateScalar z = 0;
          for ( Dimension i = 0; i < dimension-1; i++ ) {
            const CoordinateScalar x = CoordinateScalar( input[ i ] );
            p[ i ]   = x;
            z       += x*x;
          }
          p[ dimension-1 ] = z;
          return p;
        };
      DGtal::detail::transform( processed_points, input2comp, comp2input,
                                input_points.cbegin(), input_points.cend(),
                                F, remove_duplicates );
    }

    /// @tparam OutputPoint a model of point such that processing type
    /// Point is convertible to it.
    template < typename OutputPoint>
    void convertPointTo( const CoordinatePoint& p, OutputPoint& out_p ) const
    {
      for ( Dimension k = 0; k < dimension-1; k++ )
        out_p[ k ] = p[ k ];
    }
        
  }; //   template < Dimension dim >  struct DelaunayIntegralKernel {


  /////////////////////////////////////////////////////////////////////////////
  // template class ConvexHullRationalKernel
  /**
     Description of template class 'ConvexHullRationalKernel' <p>
     \brief Aim: a geometric kernel to compute the convex hull of
     floating points with integer-only arithmetic. Floating points are
     approximated with rational points with fixed precision (a given
     number of bits). All remaining computations are exact, as long as
     there is no overflow.
     
     Each floating point input coordinate `x` is converted to an integer
     through the following formula `(Integer) round( x * precision )`,
     where `precision` is the floating point value given at
     instanciation of the kernel.

     Each output floating point coordinate is built from integer
     coordinates `a` through the formula `( (double) a ) / precision`,
     where `precision` is the floating point value given at
     instanciation of the kernel.

     @see \ref moduleQuickHull

     @tparam dim the dimension of the space of processed points.
  */
  template < Dimension dim >
  struct ConvexHullRationalKernel : public ConvexHullCommonKernel< dim >
  {
    typedef ConvexHullCommonKernel< dim > Base;
    // inheriting types
    // using typename Base::Point;
    // using typename Base::Vector;
    // using typename Base::Scalar;
    using typename Base::CoordinatePoint;
    using typename Base::CoordinateVector;
    using typename Base::CoordinateScalar;
    using typename Base::InternalPoint;
    using typename Base::InternalVector;
    using typename Base::InternalScalar;
    using typename Base::Size;
    using typename Base::Index;
    using typename Base::IndexRange;
    using typename Base::CombinatorialPlaneSimplex;
    using typename Base::HalfSpace;
    // inheriting constants
    using Base::dimension;
    // inheriting methods
    using Base::compute;
    using Base::normal;
    using Base::intercept;
    using Base::dot;
    using Base::equal;
    using Base::height;
    using Base::volume;
    using Base::above;
    using Base::aboveOrOn;
    using Base::on;

    /// The precision as the common denominator for all rational points.
    double precision;
    
    /// Constructor with specified precision
    ///
    /// @param[in] aPrecision the chosen precision as the common
    /// denominator of all rationals (by defaut, 1024).
    ConvexHullRationalKernel( double aPrecision = 1024. )
      : precision( aPrecision ) {}

    /// @return 'true' if a kernel may induce infinite facets. This is
    /// typically the case of a Delaunay computation kernel, which
    /// casts points in higher dimension.
    bool hasInfiniteFacets() const
    { return false; }

    /// @param[in] hs an half-space corresponding to a facet.
    ///
    /// @return 'true' if the facet associated to this half-space
    /// corresponds to an infinite facet.
    bool isHalfSpaceFacetInfinite( const HalfSpace& hs ) const
    {
      (void) hs; // unused parameter      
      return false;
    }
    
    /// Transforms a range \a input_points of input points to a range
    /// \a processed_points of points adapted to a processing by
    /// QuickHull convex hull algorithm. Keep the mapping information
    /// between input and processed points.
    ///
    /// @tparam InputPoint any model of point whose components are
    /// convertible to Scalar.
    ///
    /// @param[out] processed_points the range of points prepared for
    /// a process by QuickHull.
    /// 
    /// @param[out] input2comp the surjective mapping between
    /// the \a input_points range and the \a processed_points range
    /// used for computation.
    ///
    /// @param[out] comp2input the injective mapping between the 
    /// \a processed_points range used for computation and the \a
    /// input_points range.
    ///
    /// @param[in] input_points the range of input points.
    ///
    /// @param[in] remove_duplicates when 'true', this method removes possible
    /// duplicates in \a input_points and \a processed_points may thus
    /// be of smaller size, otherwise, when 'false', it means that
    /// there are no duplicates in \a input_points.
    ///
    /// @note Each floating point input coordinate `x` is converted to an integer
    /// through the following formula `(Integer) round( x * precision )`,
    /// where `precision` is the floating point value given at
    /// instanciation of the kernel.
    template < typename InputPoint>
    void makeInput( std::vector< CoordinatePoint >& processed_points,
                    IndexRange& input2comp, IndexRange& comp2input,
                    const std::vector< InputPoint >& input_points,
                    bool remove_duplicates )
    {
      const auto F = [&] ( InputPoint input ) -> CoordinatePoint
        {
          CoordinatePoint p;
          for ( Dimension i = 0; i < dimension; i++ )
            p[ i ] = CoordinateScalar( round( input[ i ] * precision ) );
          return p;
        };
      DGtal::detail::transform( processed_points, input2comp, comp2input,
                                input_points.cbegin(), input_points.cend(),
                                F, remove_duplicates );
    }

    /// Converts an integral point (as represented internally for
    /// QuickHull computations) to its corresponding output point
    /// representation.
    ///
    /// @tparam OutputPoint a model of point such that processing type
    /// Point is convertible to it.
    ///
    /// @param[in] p an integral point (as represented internally for
    /// QuickHull computations)
    ///
    /// @param[out] out_p its corresponding output point representation.
    ///
    /// @note Each output floating point coordinate is built from integer
    /// coordinates `a` through the formula `( (double) a ) / precision`,
    /// where `precision` is the floating point value given at
    /// instanciation of the kernel.
    template < typename OutputPoint>
    void convertPointTo( const CoordinatePoint& p, OutputPoint& out_p ) const
    {
      for ( Dimension k = 0; k < dimension; k++ )
        out_p[ k ] = ( (double) p[ k ] ) / precision;
    }
    
    
  }; //   template < Dimension dim >  struct ConvexHullRationalKernel {


  /////////////////////////////////////////////////////////////////////////////
  // template class DelaunayRationalKernel
  /**
     Description of template class 'DelaunayRationalKernel' <p> \brief
     Aim: a geometric kernel to compute the Delaunay triangulation of
     a range of floating points with integer-only arithmetic. Floating
     points are approximated with rational points with fixed precision
     (a given number of bits), which are cast in a higher dimensional
     space and lifted onto the "norm" paraboloid, as classically done
     when computing a Delaunay triangulation from a convex hull. All
     remaining computations are exact, as long as there is no
     overflow.

     Each floating point input coordinate `x` is converted to an integer
     through the following formula `(Integer) round( x * precision )`,
     where `precision` is the floating point value given at
     instanciation of the kernel.

     Each output floating point coordinate is built from integer
     coordinates `a` through the formula `( (double) a ) / precision`,
     where `precision` is the floating point value given at
     instanciation of the kernel.

     @see \ref moduleQuickHull

     @tparam dim the dimension of the space of processed points.
  */
  template < Dimension dim >
  struct DelaunayRationalKernel : public ConvexHullCommonKernel< dim+1 >
  {
    typedef ConvexHullCommonKernel< dim+1 > Base;
    // inheriting types
    // using typename Base::Point;
    // using typename Base::Vector;
    // using typename Base::Scalar;
    using typename Base::CoordinatePoint;
    using typename Base::CoordinateVector;
    using typename Base::CoordinateScalar;
    using typename Base::InternalPoint;
    using typename Base::InternalVector;
    using typename Base::InternalScalar;
    using typename Base::Size;
    using typename Base::Index;
    using typename Base::IndexRange;
    using typename Base::CombinatorialPlaneSimplex;
    using typename Base::HalfSpace;
    // inheriting constants
    using Base::dimension;
    // inheriting methods
    using Base::compute;
    using Base::normal;
    using Base::intercept;
    using Base::dot;
    using Base::equal;
    using Base::height;
    using Base::volume;
    using Base::above;
    using Base::aboveOrOn;
    using Base::on;

    /// The precision as the common denominator for all rational points.
    double precision;
    
    /// Constructor with specified precision
    ///
    /// @param[in] aPrecision the chosen precision as the common
    /// denominator of all rationals (by defaut, 1024).
    DelaunayRationalKernel( double aPrecision = 1024. )
      : precision( aPrecision ) {}

    /// @return 'true' if a kernel may induce infinite facets. This is
    /// typically the case of a Delaunay computation kernel, which
    /// casts points in higher dimension.
    bool hasInfiniteFacets() const
    { return true; }

    /// @param[in] hs an half-space corresponding to a facet.
    ///
    /// @return 'true' if the facet associated to this half-space
    /// correspond to an infinite facet.
    bool isHalfSpaceFacetInfinite( const HalfSpace& hs ) const
    {
      return hs.internalNormal()[ dimension - 1 ] >= InternalScalar( 0 );
    }

    /// Transforms a range \a input_points of input points to a range
    /// \a processed_points of points adapted to a processing by
    /// QuickHull convex hull algorithm. Keep the mapping information
    /// between input and processed points. This kernel transforms
    /// `dim`-dimensional into `dim+1`-dimensional points, where the
    /// last coordinate lies on a paraboloid. This is the classical
    /// way for computing a Delaunay triangulation as the convex hull
    /// of the points onto a paraboloid of higher dimension.
    ///
    /// @tparam InputPoint any model of point whose components are
    /// convertible to Scalar.
    ///
    /// @param[out] processed_points the range of points prepared for
    /// a process by QuickHull.
    /// 
    /// @param[out] input2comp the surjective mapping between
    /// the \a input_points range and the \a processed_points range
    /// used for computation.
    ///
    /// @param[out] comp2input the injective mapping between the 
    /// \a processed_points range used for computation and the \a
    /// input_points range.
    ///
    /// @param[in] input_points the range of input points.
    ///
    /// @param[in] remove_duplicates when 'true', this method removes possible
    /// duplicates in \a input_points and \a processed_points may thus
    /// be of smaller size, otherwise, when 'false', it means that
    /// there are no duplicates in \a input_points.
    ///
    /// @note Each floating point input coordinate `x` is converted to
    /// an integer through the following formula `(Integer) round( x *
    /// precision )`, where `precision` is the floating point value
    /// given at instanciation of the kernel. A new coordinate is
    /// added so that the point is lifted onto the "norm" paraboloid.
    template < typename InputPoint>
    void makeInput( std::vector< CoordinatePoint >& processed_points,
                    IndexRange& input2comp, IndexRange& comp2input,
                    const std::vector< InputPoint >& input_points,
                    bool remove_duplicates )
    {
      const auto F = [&] ( InputPoint input ) -> CoordinatePoint
        {
          CoordinatePoint p;
          CoordinateScalar z = 0;
          for ( Dimension i = 0; i < dimension - 1; i++ ) {
            const CoordinateScalar x
              = CoordinateScalar( round( input[ i ] * precision ) );
            p[ i ] = x;
            z     += x*x;
          }
          p[ dimension-1 ] = z;
          return p;
        };
      DGtal::detail::transform( processed_points, input2comp, comp2input,
                                input_points.cbegin(), input_points.cend(),
                                F, remove_duplicates );
    }

    /// Converts an integral point (as represented internally for
    /// QuickHull computations) to its corresponding output point
    /// representation.
    ///
    /// @tparam OutputPoint a model of point such that processing type
    /// Point is convertible to it.
    ///
    /// @param[in] p an integral point (as represented internally for
    /// QuickHull computations)
    ///
    /// @param[out] out_p its corresponding output point representation.
    ///
    /// @note Each output floating point coordinate is built from
    /// integer coordinates `a` through the formula `( (double) a ) /
    /// precision`, where `precision` is the floating point value
    /// given at instanciation of the kernel. The last coordinate is
    /// not used since it was just related to the lifting of the point
    /// onto the "norm" paraboloid, as classically done when computing
    /// the Delaunay triangulation from a convex hull.
    template < typename OutputPoint>
    void convertPointTo( const CoordinatePoint& p, OutputPoint& out_p ) const
    {
      for ( Dimension k = 0; k < dimension - 1; k++ )
        out_p[ k ] = ( (double) p[ k ] ) / precision;
    }
    
  }; //   template < Dimension dim >  struct DelaunayRationalKernel {

  
  
} // namespace DGtal {

#endif // !defined QuickHullKernels_h

#undef QuickHullKernels_RECURSES
#endif // else defined(QuickHullKernels_RECURSES)
