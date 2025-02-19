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
 * @file IntegerConverter.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/04/10
 *
 * Header file for module IntegerConverter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(IntegerConverter_RECURSES)
#error Recursive header files inclusion detected in IntegerConverter.h
#else // defined(IntegerConverter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IntegerConverter_RECURSES

#if !defined IntegerConverter_h
/** Prevents repeated inclusion of headers. */
#define IntegerConverter_h

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

namespace DGtal
{
#ifdef WITH_BIGINTEGER
  namespace detail {
    /// ------------- GMP SPECIALIZED SERVICES ----------------------------
    
    /// @param[inout] n the (initialized) big integer to set
    /// @param[in] sll a signed long long integer to assign to \a n.
    static inline void mpz_set_sll(mpz_t n, long long sll)
    {
      mpz_set_si(n, (int)(sll >> 32));     /* n = (int)sll >> 32 */
      mpz_mul_2exp(n, n, 32 );             /* n <<= 32 */
      mpz_add_ui(n, n, (unsigned int)sll); /* n += (unsigned int)sll */
    }
    
    /// @param[inout] n the (initialized) big integer to set
    /// @param[in] ull an unsigned long long integer to assign to \a n.
    static inline void mpz_set_ull(mpz_t n, unsigned long long ull)
    {
      mpz_set_ui(n, (unsigned int)(ull >> 32)); /* n = (unsigned int)(ull >> 32) */
      mpz_mul_2exp(n, n, 32);                   /* n <<= 32 */
      mpz_add_ui(n, n, (unsigned int)ull);      /* n += (unsigned int)ull */
    }      

    /// Conversion to uint64 is tricky and not native for GMP.
    /// @param n any number
    /// @return its uint64 representation.
    static inline unsigned long long mpz_get_ull(mpz_t n)
    {
      mpz_t tmp;
      mpz_init( tmp );
      mpz_mod_2exp( tmp, n, 64 );   /* tmp = (lower 64 bits of n) */
      auto lo = mpz_get_ui( tmp );       /* lo = tmp & 0xffffffff */
      mpz_div_2exp( tmp, tmp, 32 ); /* tmp >>= 32 */
      auto hi = mpz_get_ui( tmp );       /* hi = tmp & 0xffffffff */
      mpz_clear( tmp );
      return (((unsigned long long)hi) << 32) + lo;
    }
    
    /// Conversion to int64 is tricky and not native for GMP.
    /// @param n any number
    /// @return its int64 representation.
    static inline long long mpz_get_sll(mpz_t n)
    {
      return (long long)mpz_get_ull(n); /* just use unsigned version */
    }
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
             concepts::CInteger TInteger >
  struct IntegerConverter {
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
      auto r = i.get_si();
      if ( DGtal::BigInteger( r ) != i )
        trace.warning() << "Bad integer conversion: " << i << " -> " << r
                        << std::endl;
      return (DGtal::int32_t)r;
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
      DGtal::int64_t r = detail::mpz_get_sll( i.get_mpz_t() );
      DGtal::BigInteger tmp;
      detail::mpz_set_sll( tmp.get_mpz_t(), r );
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
      detail::mpz_set_sll( tmp.get_mpz_t(), i );
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

      
} // namespace DGtal {

#endif // !defined IntegerConverter_h

#undef IntegerConverter_RECURSES
#endif // else defined(IntegerConverter_RECURSES)
