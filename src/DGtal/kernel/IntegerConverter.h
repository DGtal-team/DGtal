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
      
    /// @param i any integer
    /// @return the same integer
    static DGtal::int32_t cast( DGtal::BigInteger i ) 
    {
      auto r = NumberTraits<DGtal::BigInteger>::castToInt64_t(i);
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
      
    /// @param i any integer
    /// @return the same integer
    static DGtal::int64_t cast( DGtal::BigInteger i ) 
    {
      DGtal::int64_t r = NumberTraits<DGtal::BigInteger>::castToInt64_t(i);
      DGtal::BigInteger tmp(r);
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
  };
    
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
      DGtal::BigInteger tmp = i;
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
      
} // namespace DGtal {

#endif // !defined IntegerConverter_h

#undef IntegerConverter_RECURSES
#endif // else defined(IntegerConverter_RECURSES)
