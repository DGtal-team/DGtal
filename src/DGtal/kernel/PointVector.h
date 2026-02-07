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
 * @file PointVector.h
 * @author David Coeurjolly (@c david.coeurjolly@liris.cnrs.fr )
 * @author Guillaume Damiand
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/05/14
 *
 *
 * This file is part of the DGtal library.
 */

#if defined(PointVector_RECURSES)
#error Recursive header files inclusion detected in PointVector.h
#else // defined(PointVector_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PointVector_RECURSES

#if !defined PointVector_h
/** Prevents repeated inclusion of headers. */
#define PointVector_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <bitset>
#include <algorithm>
#include <array>
#include <vector>
#include <type_traits>
#include <utility>
#include <functional>

#include "DGtal/base/Common.h"
#include "DGtal/base/CBidirectionalRange.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/kernel/CEuclideanRing.h"
#include "DGtal/kernel/ArithmeticConversionTraits.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /// @return a bitset having true for each dimension presents in dimensions.
  template<Dimension dim, typename Container>
  std::bitset<dim> setDimensionsIn( const Container &dimensions );

  /// @return a bitset having true for each dimension not presents in dimensions.
  template<Dimension dim, typename Container>
  std::bitset<dim> setDimensionsNotIn( const Container &dimensions );

  /////////////////////////////////////////////////////////////////////////////
  // class PointVector pre-declaration
  /**
   * Description of class 'PointVector' <p>
   *
   * @brief Aim: Implements basic operations that will be used in
   * Point and  Vector classes.
   *
   * A PointVector may represent either a symbolic point or a symbolic
   * vector depending on the context. The coordinates of the point or
   * the components of the vector should be part of a ring. For
   * performance reasons, these two types are just aliases. The user
   * should take care how to use it depending on the context. For
   * instance, adding two points has no meaning, but will be
   * authorized by the compiler.
   *
   * @tparam dim static constant of type DGtal::Dimension that
   * specifies the static  dimension of the space and thus the number
   * of elements  of the Point or Vector.
   * @tparam TEuclideanRing specifies the number type associated to an
   * Euclidean domain (or Euclidean ring) algebraic structure
   * (commutative unitary ring with no zero divisors and with a division
   * operator but not necessarily an inverse for the multiplication
   * operator). This type is used to represent PointVector elements
   * (Coordinate for Point and Component for Vector) and define
   * operations on Point or Vectors.
   * @tparam TContainer specifies the container to be used to store
   * the point coordinates. At this point, such container must be a
   * random access bidirectionnal a-la STL containers (e.g. vector,
   * boost/array). If TContainer implements comparison operators == !=
   * < <= > <=, then PointVector will also implements it and with the
   * exact same behaviour.
   *
   *
   * All operations involving @ref PointVector, and some of its methods,
   * follow the classical arithmetic conversion rules.
   * More precisely, if an operation involves two component types
   * @a T and @a U, then the result will have the type of `T() + U()`
   * as component type, that is the last of @a T and @a U in the following
   * conversion chain:
   *
   * @code
   * int8_t -> uint8_t -> int16_t -> uint16_t -> int32_t -> uint32_t -> int64_t -> uint64_t -> float -> double -> long double.
   * @endcode
   *
   * A consequence is that if the result is stored in a @ref PointVector
   * whose component type has lower rank than the result type in the
   * conversion chain above, then it will result in a compilation error.
   * This behavior is designed to avoid unwanted conversions that may lead to loss of precision.
   *
   * This constraint can be dodged by using one of the conversion constructor
   * (explicitly or along with a conversion functor) or equivalent methods.
   *
   * @see ArithmeticConversionTraits
   * @see https://en.cppreference.com/w/cpp/language/operator_arithmetic#Conversions
   *
   *
   * For example, dividing a @ref PointVector with integer components by
   * an integer uses the Euclidean division and returns a @ref PointVector
   * with higher integer component type.
   * On the other hand, dividing a @ref PointVector with integer components by
   * a double will use classical division on real numbers and returns
   * a @ref PointVector with double component type.
   *
   *
   * PointVector also realizes the concept CLattice with an infimum
   * (meet, greatest lower bound) and a supremum (join, least upper
   * bound) operation.
   *
   * Usage example:
   * @code
   *
   * ...
   * typedef PointVector<5, int> VectorD5;
   * VectorD5 p, q, r;
   *
   * p[1] = 2;  // p = {0, 2, 0, 0, 0}
   * q[3] = -5   // q = {0, 0, 0, -5, 0}
   * r =  p + q ;   //  r = {0, 2, 0, -5, 0}
   *
   * ...
   * @endcode
   *
   * PointVector is a model of CBidirectionalRange.
   *
   * @see testPointVector.cpp
   *
   */
  template < DGtal::Dimension dim,
      typename TEuclideanRing,
      typename TContainer=std::array<TEuclideanRing,dim> >
  class PointVector;

  /////////////////////////////////////////////////////////////////////////////
  // Type traits

  /** @brief Type trait to check if a given type is a @ref PointVector
   *
   * Default definition inherits from `std::false_type`.
   */
  template <typename T>
  struct IsAPointVector : std::false_type {};

  /** @brief Specialization of @ref IsAPointVector for a @ref PointVector
   *
   * Inherits from `std::true_type`
   */
  template <DGtal::Dimension dim, typename TEuclideanRing, typename TContainer>
  struct IsAPointVector<PointVector<dim, TEuclideanRing, TContainer>> : std::true_type {};

  /** @brief Specialization of @ref ArithmeticConversionTraits when both operands are @ref PointVector
   *
   * @see ArithmeticConversionTraits
   */
  template < DGtal::Dimension dim,
    typename LeftEuclideanRing, typename LeftContainer,
    typename RightEuclideanRing, typename RightContainer >
  struct ArithmeticConversionTraits< PointVector<dim, LeftEuclideanRing, LeftContainer>, PointVector<dim, RightEuclideanRing, RightContainer>,
        typename std::enable_if< IsArithmeticConversionValid<LeftEuclideanRing, RightEuclideanRing>::value >::type >
    {
      using type = typename std::conditional<
        std::is_same< LeftEuclideanRing, ArithmeticConversionType<LeftEuclideanRing, RightEuclideanRing> >::value,
        PointVector<dim, LeftEuclideanRing, LeftContainer>,
        PointVector<dim, RightEuclideanRing, RightContainer> >::type;
    };

  /** @brief Specialization of @ref ArithmeticConversionTraits when left operand is a @ref PointVector
   *
   * @see ArithmeticConversionTraits
   */
  template < DGtal::Dimension dim,
    typename LeftEuclideanRing, typename LeftContainer,
    typename RightEuclideanRing >
  struct ArithmeticConversionTraits< PointVector<dim, LeftEuclideanRing, LeftContainer>, RightEuclideanRing,
        typename std::enable_if<
               IsArithmeticConversionValid<LeftEuclideanRing, RightEuclideanRing>::value
            && ! IsAPointVector<RightEuclideanRing>::value >::type >
    {
      using type = typename std::conditional<
        std::is_same< LeftEuclideanRing, ArithmeticConversionType<LeftEuclideanRing, RightEuclideanRing> >::value,
        PointVector<dim, LeftEuclideanRing, LeftContainer>,
        PointVector<dim, RightEuclideanRing> >::type;
    };

  /** @brief Specialization of @ref ArithmeticConversionTraits when right operand is a @ref PointVector
   *
   * @see ArithmeticConversionTraits
   */
  template < DGtal::Dimension dim,
    typename LeftEuclideanRing,
    typename RightEuclideanRing, typename RightContainer >
  struct ArithmeticConversionTraits< LeftEuclideanRing, PointVector<dim, RightEuclideanRing, RightContainer>,
        typename std::enable_if<
                 IsArithmeticConversionValid<LeftEuclideanRing, RightEuclideanRing>::value
            && ! IsAPointVector<LeftEuclideanRing>::value  >::type >
    {
      using type = typename std::conditional<
        std::is_same< LeftEuclideanRing, ArithmeticConversionType<LeftEuclideanRing, RightEuclideanRing> >::value,
        PointVector<dim, LeftEuclideanRing>,
        PointVector<dim, RightEuclideanRing, RightContainer> >::type;
    };


  /////////////////////////////////////////////////////////////////////////////
  // PointVector comparison operators

  /** @brief Equality operator between two Points/Vectors.
   *
   * @return true iff the two points are equal.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline bool
  operator== ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
               PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

  /** @brief Difference operator on Points/Vectors.
   *
   * @return true iff the two points differ, false otherwise.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline bool
  operator!= ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
               PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

  /** @brief Comparison operator on Points/Vectors (LesserThan).
   *
   * @return true iff lhs < rhs, false otherwise.
   *
   * @note It uses the lexicographical order.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline bool
  operator< ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
              PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

  /** @brief Comparison operator on Points/Vectors (GreaterThan).
   *
   * @return true iff lhs > rhs, false otherwise.
   *
   * @note It uses the lexicographical order.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline bool
  operator> ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
              PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

  /** @brief Comparison operator on Points/Vectors (LesserOrEqualThan).
   *
   * @return true iff lhs <= rhs, false otherwise.
   *
   * @note It uses the lexicographical order.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline bool
  operator<= ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
               PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

  /** @brief Comparison operator on Points/Vectors (GreaterOrEqualThan).
   *
   * @return true iff lhs >= rhs, false otherwise.
   *
   * @note It uses the lexicographical order.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline bool
  operator>= ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
               PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

  /////////////////////////////////////////////////////////////////////////////
  // Binary arithmetic operators between two PointVectors

  /** @brief Addition operator between two Points/Vectors.
   *
   * @return a point/vector with best component type accordingly to
   *   the C++ conversion rules in arithmetic operations context.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline auto
  operator+ ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
              PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
      -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

  /** @brief Subtraction operator between two Points/Vectors.
   *
   * @return a point/vector with best component type accordingly to
   *   the C++ conversion rules in arithmetic operations context.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline auto
  operator- ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
              PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
      -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

  /** @brief Multiplication operator between two Points/Vectors.
   *
   * @return a point/vector with best component type accordingly to
   *   the C++ conversion rules in arithmetic operations context.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline auto
  operator* ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
              PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
      -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

  /** @brief Division operator between two Points/Vectors.
   *
   * @return a point/vector with best component type accordingly to
   *   the C++ conversion rules in arithmetic operations context.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline auto
  operator/ ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
              PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
      -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

  /////////////////////////////////////////////////////////////////////////////
  // Binary arithmetic operators between a PointVector and a scalar

  /** @brief Addition operator between a Point/Vector and a scalar.
   *
   * @return a point/vector with best component type accordingly to
   *   the C++ conversion rules in arithmetic operations context.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightScalar >
  inline auto
  operator+ ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
              RightScalar const& rhs )
      -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

  /** @brief Addition operator between a scalar and a Point/Vector.
   *
   * @return a point/vector with best component type accordingly to
   *   the C++ conversion rules in arithmetic operations context.
   */
  template < Dimension ptDim,
      typename LeftScalar,
      typename RightEuclideanRing, typename RightContainer >
  inline auto
  operator+ ( LeftScalar const& lhs,
             PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
      -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

  /** @brief Subtraction operator between a Point/Vector and a scalar.
   *
   * @return a point/vector with best component type accordingly to
   *   the C++ conversion rules in arithmetic operations context.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightScalar >
  inline auto
  operator- ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
              RightScalar const& rhs )
      -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

  /** @brief Subtraction operator between a scalar and a Point/Vector.
   *
   * @return a point/vector with best component type accordingly to
   *   the C++ conversion rules in arithmetic operations context.
   */
  template < Dimension ptDim,
      typename LeftScalar,
      typename RightEuclideanRing, typename RightContainer >
  inline auto
  operator- ( LeftScalar const& lhs,
             PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
      -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

  /** @brief Multiplication operator between a Point/Vector and a scalar.
   *
   * @return a point/vector with best component type accordingly to
   *   the C++ conversion rules in arithmetic operations context.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightScalar >
  inline auto
  operator* ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
              RightScalar const& rhs )
      -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

  /** @brief Multiplication operator between a scalar and a Point/Vector.
   *
   * @return a point/vector with best component type accordingly to
   *   the C++ conversion rules in arithmetic operations context.
   */
  template < Dimension ptDim,
      typename LeftScalar,
      typename RightEuclideanRing, typename RightContainer >
  inline auto
  operator* ( LeftScalar const& lhs,
             PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
      -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

  /** @brief Division operator between a Point/Vector and a scalar.
   *
   * @return a point/vector with best component type accordingly to
   *   the C++ conversion rules in arithmetic operations context.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightScalar >
  inline auto
  operator/ ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
              RightScalar const& rhs )
      -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

  /** @brief Division operator between a scalar and a Point/Vector.
   *
   * @return a point/vector with best component type accordingly to
   *   the C++ conversion rules in arithmetic operations context.
   */
  template < Dimension ptDim,
      typename LeftScalar,
      typename RightEuclideanRing, typename RightContainer >
  inline auto
  operator/ ( LeftScalar const& lhs,
             PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
      -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

  /////////////////////////////////////////////////////////////////////////////
  // Operations between Points/Vectors

  /** @brief Dot product between two points/vectors.
   *
   * @return the dot product in the best Euclidean ring accordingly to
   *   the C++ conversion rules in arithmetic operations context.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline
  DGtal::ArithmeticConversionType<LeftEuclideanRing, RightEuclideanRing>
  dotProduct ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
               PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

  /** @brief Cross product of two 3D Points/Vectors.
   *
   * @return a 3D point/vector with best component type accordingly to
   *   the C++ conversion rules in arithmetic operations context.
   */
  template <
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline auto
  crossProduct( PointVector<3, LeftEuclideanRing, LeftContainer> const& lhs,
                PointVector<3, RightEuclideanRing, RightContainer> const& rhs )
      -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

  /** @brief Cross product of two 2D Points/Vectors.
   *
   * @return the 3th component of the cross product of the two points/vectors
   *    embedded in 3D.
   */
  template <
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline
  PointVector<3, DGtal::ArithmeticConversionType<LeftEuclideanRing, RightEuclideanRing>>
  crossProduct( PointVector<2, LeftEuclideanRing, LeftContainer> const& lhs,
                PointVector<2, RightEuclideanRing, RightContainer> const& rhs );

  /** @brief Positive angle between two vectors, deduced from their scalar product.
   *
   * @return the angle between @a lhs and @a rhs in [0,pi].
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline double
  cosineSimilarity( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
                    PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

  /** @brief Implements the infimum (or greatest lower bound).
   *
   * It means the point whose coordinates are exactly the minimum of the two
   * points coordinate by coordinate.
   *
   * @return a new point (with best Euclidean ring type accordingly to
   *    the C++ conversion rules) being the inf between @a lhs and @a rhs;
   * @see isLower
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline auto
  inf( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
       PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
      -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

  /** @brief Implements the supremum (or least upper bound).
   *
   * It means the point whose coordinates are exactly the maximum of the two
   * points coordinate by coordinate.
   *
   * @return a new point (with best Euclidean ring type accordingly to
   *    the C++ conversion rules) being the sup between *this and apoint.
   * @see isUpper
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline auto
  sup( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
       PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
      -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

  /** @brief Return true if the first point is below the second point.
   *
   * @return true if @a lhs is below @a rhs (ie. lhs == inf(lhs,rhs))
   * @note faster than computing the infimum and compare it afterwards.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline bool
  isLower( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
           PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

  /** @brief Return true if the first point is upper the second point.
   *
   * @return true if @a lhs is upper @a rhs (ie. lhs == sup(lhs,rhs))
   * @note faster than computing the supremum and compare it afterwards.
   */
  template < Dimension ptDim,
      typename LeftEuclideanRing, typename LeftContainer,
      typename RightEuclideanRing, typename RightContainer >
  inline bool
  isUpper( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
           PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

  /////////////////////////////////////////////////////////////////////////////
  // class PointVector full declaration
  template < DGtal::Dimension dim,
      typename TEuclideanRing,
      typename TContainer >
  class PointVector
  {

    // Friend with all PointVectors
    template <
      DGtal::Dimension otherDim,
      typename TOtherEuclideanRing,
      typename TOtherContainer >
    friend class PointVector;

    // ----------------------- Standard services ------------------------------
  public:

    BOOST_CONCEPT_ASSERT(( concepts::CEuclideanRing<TEuclideanRing> ) );

    ///We cannot check the TContainer since boost::array is not a
    ///model of boost::RandomAccessContainer

    ///Self type
    typedef PointVector<dim, TEuclideanRing, TContainer> Self;

    ///Type for Vector elements
    typedef TEuclideanRing Component;

    ///Type for Point elements
    typedef Component Coordinate;

    ///Unsigned version of the components.
    typedef typename NumberTraits<Component>::UnsignedVersion UnsignedComponent;

    ///Copy of the dimension type
    typedef DGtal::Dimension Dimension;

    ///Copy of the static dimension of the Point/Vector.
    static const Dimension dimension = dim;

    ///Types needed by CLinearAlgebraContainer
    typedef Component Scalar;
    typedef Dimension Index;

    ///Copy of the container type
    typedef TContainer Container;


    /**
     *  Copy of the Container iterator types
     *
     **/
    typedef typename Container::iterator Iterator; ///< Mutable iterator type.
    typedef typename Container::const_iterator ConstIterator; ///< Constant iterator type.
    typedef typename Container::reverse_iterator ReverseIterator; ///< Mutable reverse iterator type.
    typedef typename Container::const_reverse_iterator ConstReverseIterator; ///< Constant reverse iterator type.

    /** @brief
     * Constructor.
     */
    PointVector();

    /** @brief
     * Constructor from array of values.
     *
     * @param ptrValues the array of values.
     * @note this constructor is explicit to avoid unwanted conversion from pointers.
     */
    explicit PointVector( const Component* ptrValues );

    /** @brief
     * Constructor from two values (the Dimension of the vector should
     * be at least 2). Other components are set to 0.
     *
     * @param x the first value.
     * @param y the second value.
     */
    PointVector( const Component & x, const Component & y );

    /** @brief
     * Constructor from three values (the Dimension of the vector should
     * be at least 3). Other components are set to 0.
     *
     * @param x the first value.
     * @param y the second value.
     * @param z the third value.
     */
    PointVector( const Component & x, const Component & y, const Component & z );

    /** @brief
     * Constructor from four values (the Dimension of the vector should
     * be at least 4). Other components are set to 0.
     *
     * @param x the first value.
     * @param y the second value.
     * @param z the third value.
     * @param t the fourth value.
     */
    PointVector( const Component & x, const Component & y,
                 const Component & z, const Component & t );

    /** @brief Constructor from initializer list.
     *
     * @param init the initializer list.
     */
    PointVector( std::initializer_list<Component> init );

    /** @brief Constructor taking two points and a functor as parameters.
     *
     *  The new point is initialized by the result of functor f
     *  applied for each pair of coordinates of apoint1 and apoint2
     */
    template <
      typename LeftComponent, typename LeftStorage,
      typename RightComponent, typename RightStorage,
      typename BinaryFunctor >
    PointVector( const PointVector<dim, LeftComponent, LeftStorage> & apoint1,
                 const PointVector<dim, RightComponent, RightStorage> & apoint2,
                 const BinaryFunctor& f );

    /** @brief Constructor taking a point and a unary functor as parameters.
     *
     *  The new point is initialized by the result of functor f for
     *  each coordinate of apoint1
     */
    template <
      typename OtherComponent, typename OtherStorage,
      typename UnaryFunctor >
    PointVector( const PointVector<dim, OtherComponent, OtherStorage> & apoint1,
                 const UnaryFunctor & f );

    /** @brief
     * Destructor.
     */
    ~PointVector() {}

    // ----------------------- Standard services ------------------------------

  public:
    /** @brief Copy constructor.
     *
     * @param other the object to clone.
     */
    PointVector( const Self & other );

    /** @brief Copy constructor from another component PointVector.
     *
     * @param other the object to clone.
     *
     * @warning This constructor is implicitly available only if the
     * conversion from @a OtherComponent to @a Component follows the
     * classical arithmetic rules in arithmetic context
     * (see the doc of @ref PointVector).
     */
    template <
      typename OtherComponent, typename OtherCont,
      typename std::enable_if< std::is_same< Component, ArithmeticConversionType<Component, OtherComponent> >::value, int >::type = 0 >
    PointVector( const PointVector<dim,OtherComponent,OtherCont> & other );

    /** @brief Copy constructor from another component PointVector.
     *
     * @param other the object to clone.
     *
     * @warning This constructor must be explicitly specified if the
     * conversion from @a OtherComponent to @a Component breaks the
     * classical arithmetic rules in arithmetic context
     * (see the doc of @ref PointVector).
     */
    template <
      typename OtherComponent, typename OtherCont,
      typename std::enable_if< ! std::is_same< Component, ArithmeticConversionType<Component, OtherComponent> >::value, int >::type = 0 >
    explicit
    PointVector( const PointVector<dim,OtherComponent,OtherCont> & other );

    /** @brief Assignment Operator
     *
     * @param pv the object to copy.
     * @return a reference on 'this'.
     */
    Self & operator= ( const Self & pv );

    /** @brief Assignment operator from PointVector with different component
     * type.
     *
     * @param v is the Point that gets copied to @a *this.
     * @return a reference on 'this'.
     *
     * @warning Available only if the conversion from @a OtherComponent
     * to @a Component follows the classical arithmetic rules in arithmetic
     * context (see the doc of @ref PointVector).
     * Otherwise, consider converting the source point before assignment.
     */
    template <
      typename OtherComponent,
      typename OtherContainer,
      typename std::enable_if< std::is_same< Component, ArithmeticConversionType<Component, OtherComponent> >::value, int >::type = 0 >
    Self & operator= ( const PointVector<dim, OtherComponent, OtherContainer> & v );

    /** @brief Partial copy of a given PointVector.
     *
     * Only coordinates in @a dimensions are copied.
     *
     * @tparam OtherComponent Component type of the point to copy from.
     * @tparam OtherContainer Storage type of the point to copy from.
     *
     * @param pv the object to copy.
     * @param dimensions the dimensions of v to copy
     *        (Size between 0 and N, all differents).
     * @return a reference on 'this'.
     *
     * @warning Available only if the conversion from @a OtherComponent
     * to @a Component follows the classical arithmetic rules in arithmetic
     * context (see the doc of @ref PointVector).
     * Otherwise, consider using the version that accepts a functor.
     */
    template <
      typename OtherComponent,
      typename OtherContainer,
      typename std::enable_if< std::is_same< Component, ArithmeticConversionType<Component, OtherComponent> >::value, int >::type = 0 >
    Self& partialCopy ( const PointVector<dim, OtherComponent, OtherContainer> & pv,
                        const std::vector<Dimension> &dimensions);

    /** @brief Partial copy of a given PointVector.
     *
     * Only coordinates not in @a dimensions are copied.
     *
     * @tparam OtherComponent Component type of the point to copy from.
     * @tparam OtherContainer Storage type of the point to copy from.
     *
     * @param pv the object to copy.
     * @param dimensions the dimensions of v to copy
     *        (Size between 0 and N, all differents).
     * @return a reference on 'this'.
     *
     * @warning Available only if the conversion from @a OtherComponent
     * to @a Component follows the classical arithmetic rules in arithmetic
     * context (see the doc of @ref PointVector).
     * Otherwise, consider using the version that accepts a functor.
     */
    template <
      typename OtherComponent,
      typename OtherContainer,
      typename std::enable_if< std::is_same< Component, ArithmeticConversionType<Component, OtherComponent> >::value, int >::type = 0 >
    Self& partialCopyInv ( const PointVector<dim, OtherComponent, OtherContainer> & pv,
                           const std::vector<Dimension> &dimensions);

    /** @brief Partial copy of a given PointVector using a functor.
     *
     * Only coordinates in @a dimensions are copied.
     *
     * @tparam OtherComponent Component type of the point to copy from.
     * @tparam OtherContainer Storage type of the point to copy from.
     * @tparam UnaryFunctor   Type of the functor applied to copied values.
     *
     * @param pv the object to copy.
     * @param dimensions the dimensions of v to copy
     *        (Size between 0 and N, all differents).
     * @param f  the functor applied to copied values.
     *
     * @return a reference on 'this'.
     */
    template <
      typename OtherComponent,
      typename OtherContainer,
      typename UnaryFunctor >
    Self& partialCopy ( const PointVector<dim, OtherComponent, OtherContainer> & pv,
                        const std::vector<Dimension> & dimensions,
                        const UnaryFunctor & f);

    /** @brief Partial copy of a given PointVector using a functor.
     *
     * Only coordinates not in @a dimensions are copied.
     *
     * @tparam OtherComponent Component type of the point to copy from.
     * @tparam OtherContainer Storage type of the point to copy from.
     * @tparam UnaryFunctor   Type of the functor applied to copied values.
     *
     * @param pv the object to copy.
     * @param dimensions the dimensions of v to copy
     *        (Size between 0 and N, all differents).
     * @param f  the functor applied to copied values.
     *
     * @return a reference on 'this'.
     */
    template <
      typename OtherComponent,
      typename OtherContainer,
      typename UnaryFunctor >
    Self& partialCopyInv ( const PointVector<dim, OtherComponent, OtherContainer> & pv,
                           const std::vector<Dimension> & dimensions,
                           const UnaryFunctor & f);

    /** @brief Partial equality.
     *
     * Only coordinates in @a dimensions are compared.
     *
     * @tparam OtherComponent Component type of the point to compare with.
     * @tparam OtherContainer Storage type of the point to compare with.
     *
     * @param pv Point/Vector to compare to this.
     * @param dimensions  Dimensions along which to compare the points.
     *
     * @return true iff points are equal for given dimensions .
     */
    template <
      typename OtherComponent,
      typename OtherContainer >
    bool partialEqual ( const PointVector<dim, OtherComponent, OtherContainer> & pv,
                        const std::vector<Dimension> &dimensions )  const;

    /** @brief Partial inverse equality.
     *
     * Only coordinates not in @a dimensions are compared.
     *
     * @tparam OtherComponent Component type of the point to compare with.
     * @tparam OtherContainer Storage type of the point to compare with.
     *
     * @param pv Point/Vector to compare to this.
     * @param dimensions  Dimensions along which to compare the points.
     *
     * @return true iff points are equal for dimensions not in dimensions.
     */
    template <
      typename OtherComponent,
      typename OtherContainer >
    bool partialEqualInv ( const PointVector<dim, OtherComponent, OtherContainer> & pv,
                           const std::vector<Dimension> &dimensions )  const;

    // ----------------------- Iterator services ------------------------------
  public:
    /**
     * PointVector begin() iterator.
     *
     * @return an Iterator on the first element of a Point/Vector.
     **/
    Iterator begin();

    /**
     * PointVector end() iterator.
     *
     * @return an Iterator on the last element of a Point/Vector.
     **/
    Iterator end();

    /**
     * PointVector begin() const iterator.
     *
     * @return an ConstIterator on the first element of a Point/Vector.
     **/
    ConstIterator begin() const;

    /**
     * PointVector end() const iterator.
     *
     * @return a ConstIterator on the last element of a Point/Vector.
     **/
    ConstIterator end() const;

    /**
     * PointVector rbegin() reverse iterator.
     *
     * @return a ReverseIterator on the first element of a Point/Vector.
     **/
    ReverseIterator rbegin();

    /**
     * PointVector rend() reverse iterator.
     *
     * @return a ReverseIterator on the last element of a Point/Vector.
     **/
    ReverseIterator rend();

    /**
     * PointVector rbegin() const reverse iterator.
     *
     * @return an ConstReverseIterator on the first element of a Point/Vector.
     **/
    ConstReverseIterator rbegin() const;

    /**
     * PointVector rend() const reverse iterator.
     *
     * @return a ConstReverseIterator on the last element of a Point/Vector.
     **/
    ConstReverseIterator rend() const;

    /**
     * PointVector data() (const and non-const) access to raw data of a std container
     *
     * @return container.data()
     */
    inline const Component* data() const noexcept;
    /**
     * PointVector data() (const and non-const) access to raw data of a std container
     *
     * @return container.data()
     */
    inline Component* data() noexcept;

    // ----------------------- Array services ------------------------------
  public:
    /**
     * Returns the size of the vector (i.e. the number of its
     * coefficients).
     */
    static Dimension size();
    inline Dimension rows() const { return dim; }

    /**
     * Returns the  @a i-th coefficient of the vector.
     *
     * @pre The @a i index must lie between @a 0 and @a size() .
     *
     * @param i is the index of the retrieved coefficient.
     */
    const Component& operator[]( Dimension i ) const;
    inline const Component& operator()( Dimension i ) const { return (*this)[i]; }

    /**
     * Returns a non-const reference to the @a i-th element of the
     * vector.
     *
     * @pre The @a i index must lie between @a 0 and @a size() .
     *
     * @param i is the index of the retrieved coefficient.
     */
    Component& operator[](Dimension i );
    Component& operator()(Dimension i ) { return (*this)[i]; }

    // ----------------------- Comparison operations --------------------------
    // Friend declarations in order to have them documented on the same page
    // as PointVector.
  public:

    template < Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline bool
    operator== ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
                 PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

    template < Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline bool
    operator!= ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
                 PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

    template < Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline bool
    operator< ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
                PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

    template < Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline bool
    operator<= ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
                 PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

    template < Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline bool
    operator>= ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
                 PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

    // ------------- Binary arithmetic operations between points --------------
    // Friend declarations in order to have them documented on the same page
    // as PointVector.
  public:

    template < Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline auto
    operator+ ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
                PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
        -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

    template < Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline auto
    operator- ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
                PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
        -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

    template < Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline auto
    operator* ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
                PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
        -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

    template < Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline auto
    operator/ ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
                PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
        -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

    // ------ Binary arithmetic operations between a point and a vector -------
    // Friend declarations in order to have them documented on the same page
    // as PointVector.
  public:

    template < DGtal::Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightScalar >
    friend inline auto
    operator+ ( DGtal::PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
                RightScalar const& rhs )
        -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

    template < Dimension ptDim,
        typename LeftScalar,
        typename RightEuclideanRing, typename RightContainer >
    friend inline auto
    operator+ ( LeftScalar const& lhs,
               PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
        -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

    template < DGtal::Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightScalar >
    friend inline auto
    operator- ( DGtal::PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
                RightScalar const& rhs )
        -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

    template < Dimension ptDim,
        typename LeftScalar,
        typename RightEuclideanRing, typename RightContainer >
    friend inline auto
    operator- ( LeftScalar const& lhs,
               PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
        -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

    template < DGtal::Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightScalar >
    friend inline auto
    operator* ( DGtal::PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
                RightScalar const& rhs )
        -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

    template < Dimension ptDim,
        typename LeftScalar,
        typename RightEuclideanRing, typename RightContainer >
    friend inline auto
    operator* ( LeftScalar const& lhs,
               PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
        -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

    template < DGtal::Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightScalar >
    friend inline auto
    operator/ ( DGtal::PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
                RightScalar const& rhs )
        -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

    template < Dimension ptDim,
        typename LeftScalar,
        typename RightEuclideanRing, typename RightContainer >
    friend inline auto
    operator/ ( LeftScalar const& lhs,
               PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
        -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

    // -------- Unary arithmetic operations on a point or a vector ------------
  public:

    /** @brief Adds the @a coeff scalar number to @a *this.
     *
     * @return a reference on 'this'.
     *
     * @warning This operator is available only if the
     * conversion from @a OtherComponent to @a Component follows the
     * classical arithmetic rules in arithmetic context
     * (see the doc of @ref PointVector).
     * Otherwise, consider converting the right-hand-side before.
     */
    template <
      typename OtherComponent,
      typename std::enable_if< std::is_same< Component, ArithmeticConversionType<Component, OtherComponent> >::value, int >::type = 0 >
    inline
    PointVector & operator+= ( OtherComponent coeff );

    /** @brief Adds the @a v vector/point to @a *this, componentwise.
     *
     * @return a reference on 'this'.
     *
     * @warning This operator is available only if the
     * conversion from @a OtherComponent to @a Component follows the
     * classical arithmetic rules in arithmetic context
     * (see the doc of @ref PointVector).
     * Otherwise, consider converting the right-hand-side before.
     */
    template <
      typename OtherComponent, typename OtherStorage,
      typename std::enable_if< std::is_same< Component, ArithmeticConversionType<Component, OtherComponent> >::value, int >::type = 0 >
    inline
    PointVector & operator+= ( PointVector<dim, OtherComponent, OtherStorage> const& v );

    /** @brief Subtracts the @a coeff scalar number to @a *this.
     *
     * @return a reference on 'this'.
     *
     * @warning This operator is available only if the
     * conversion from @a OtherComponent to @a Component follows the
     * classical arithmetic rules in arithmetic context
     * (see the doc of @ref PointVector).
     * Otherwise, consider converting the right-hand-side before.
     */
    template <
      typename OtherComponent,
      typename std::enable_if< std::is_same< Component, ArithmeticConversionType<Component, OtherComponent> >::value, int >::type = 0 >
    inline
    PointVector & operator-= ( OtherComponent coeff );

    /** @brief Subtracts the @a v vector/point to @a *this, componentwise.
     *
     * @return a reference on 'this'.
     *
     * @warning This operator is available only if the
     * conversion from @a OtherComponent to @a Component follows the
     * classical arithmetic rules in arithmetic context
     * (see the doc of @ref PointVector).
     * Otherwise, consider converting the right-hand-side before.
     */
    template <
      typename OtherComponent, typename OtherStorage,
      typename std::enable_if< std::is_same< Component, ArithmeticConversionType<Component, OtherComponent> >::value, int >::type = 0 >
    inline
    PointVector & operator-= ( PointVector<dim, OtherComponent, OtherStorage> const& v );

    /** @brief Multiplies @a *this by the @a coeff scalar number.
     *
     * @return a reference on 'this'.
     *
     * @warning This operator is available only if the
     * conversion from @a OtherComponent to @a Component follows the
     * classical arithmetic rules in arithmetic context
     * (see the doc of @ref PointVector).
     * Otherwise, consider converting the right-hand-side before.
     */
    template <
      typename OtherComponent,
      typename std::enable_if< std::is_same< Component, ArithmeticConversionType<Component, OtherComponent> >::value, int >::type = 0 >
    inline
    PointVector & operator*= ( OtherComponent coeff );

    /** @brief Multiplies @a *this by the @a v vector/point, componentwise.
     *
     * @return a reference on 'this'.
     *
     * @warning This operator is available only if the
     * conversion from @a OtherComponent to @a Component follows the
     * classical arithmetic rules in arithmetic context
     * (see the doc of @ref PointVector).
     * Otherwise, consider converting the right-hand-side before.
     */
    template <
      typename OtherComponent, typename OtherStorage,
      typename std::enable_if< std::is_same< Component, ArithmeticConversionType<Component, OtherComponent> >::value, int >::type = 0 >
    inline
    PointVector & operator*= ( PointVector<dim, OtherComponent, OtherStorage> const& v );

    /** @brief Divides @a *this by the @a coeff scalar number.
     *
     * @return a reference on 'this'.
     *
     * @warning This operator is available only if the
     * conversion from @a OtherComponent to @a Component follows the
     * classical arithmetic rules in arithmetic context
     * (see the doc of @ref PointVector).
     * Otherwise, consider converting the right-hand-side before.
     */
    template <
      typename OtherComponent,
      typename std::enable_if< std::is_same< Component, ArithmeticConversionType<Component, OtherComponent> >::value, int >::type = 0 >
    inline
    PointVector & operator/= ( OtherComponent coeff );

    /** @brief Divides @a *this by the @a v vector/point, componentwise.
     *
     * @return a reference on 'this'.
     *
     * @warning This operator is available only if the
     * conversion from @a OtherComponent to @a Component follows the
     * classical arithmetic rules in arithmetic context
     * (see the doc of @ref PointVector).
     * Otherwise, consider converting the right-hand-side before.
     */
    template <
      typename OtherComponent, typename OtherStorage,
      typename std::enable_if< std::is_same< Component, ArithmeticConversionType<Component, OtherComponent> >::value, int >::type = 0 >
    inline
    PointVector & operator/= ( PointVector<dim, OtherComponent, OtherStorage> const& v );

    /** @brief Unary minus operator.
     *
     * -Vector => Vector
     *
     * @return a new Vector that is the opposite of 'this', i.e. -'this'.
     */
    inline
    Self operator-() const;


    // ------------------ Other operators between vectors ---------------------
  public:
    /** @brief Dot product with a PointVector
     *
     * @param v a vector that is dot-producted to *this.
     * @return the dot product in the best Euclidean ring accordingly to
     *   the C++ conversion rules in arithmetic operations context.
     *
     * @see DGtal::dotProduct
     */
    template < typename OtherComponent, typename OtherStorage >
    inline auto
    dot( const PointVector<dim, OtherComponent, OtherStorage> &v) const
        -> decltype( DGtal::dotProduct(*this, v) );

    template < Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline
    DGtal::ArithmeticConversionType<LeftEuclideanRing, RightEuclideanRing>
    dotProduct ( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
                 PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

    /** @brief Cross product with a PointVector
     *
     * @param v a vector that is cross-producted to *this.
     * @return the cross product (3D vector for 3D input, scalar for 2D input)
     *    in the best Euclidean ring accordingly to the C++ conversion rules
     *    in arithmetic operations context.
     *
     * @warning Only available in 3D and 2D (return the 3th component of the
     *    corresponding cross produt in 3D).
     * @see DGtal::crossProduct
     */
    template < typename OtherComponent, typename OtherStorage >
    inline auto
    crossProduct( const PointVector<dim, OtherComponent, OtherStorage> &v) const
        -> decltype( DGtal::crossProduct(*this, v) );

    template <
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline auto
    crossProduct( PointVector<3, LeftEuclideanRing, LeftContainer> const& lhs,
                  PointVector<3, RightEuclideanRing, RightContainer> const& rhs )
        -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

    template <
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline
    PointVector<3, DGtal::ArithmeticConversionType<LeftEuclideanRing, RightEuclideanRing>>
    crossProduct( PointVector<2, LeftEuclideanRing, LeftContainer> const& lhs,
                  PointVector<2, RightEuclideanRing, RightContainer> const& rhs );

    /** @brief
     * Positive angle between two vectors, deduced from their scalar product.
     *
     * @param v any vector
     * @return the angle between *this and v in [0,pi].
     */
    template < typename OtherComponent, typename OtherStorage >
    inline double
    cosineSimilarity ( const PointVector<dim, OtherComponent, OtherStorage> & v ) const;

    template < Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline double
    cosineSimilarity( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
                      PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

    /** @brief Implements the infimum (or greatest lower bound).
     *
     * It means the point whose coordinates are exactly the minimum of the two
     * points coordinate by coordinate.
     *
     * @param aPoint any point.
     * @return a new point (with best Euclidean ring type accordingly to
     *    the C++ conversion rules) being the inf between *this and apoint.
     * @see isLower
     */
    template < typename OtherComponent, typename OtherStorage >
    inline auto
    inf( const PointVector<dim, OtherComponent, OtherStorage> & aPoint ) const
        -> decltype( DGtal::inf(*this, aPoint) );

    template < Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline auto
    inf( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
         PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
        -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

    /** @brief Implements the supremum (or least upper bound).
     *
     * It means the point whose coordinates are exactly the maximum of the two
     * points coordinate by coordinate.
     *
     * @param aPoint any point.
     * @return a new point (with best Euclidean ring type accordingly to
     *    the C++ conversion rules) being the sup between *this and apoint.
     * @see isUpper
     */
    template < typename OtherComponent, typename OtherStorage >
    inline auto
    sup( const PointVector<dim, OtherComponent, OtherStorage> & aPoint ) const
        -> decltype( DGtal::sup(*this, aPoint) );

    template < Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline auto
    sup( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
         PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs )
        -> decltype( DGtal::constructFromArithmeticConversion(lhs, rhs) );

    /** @brief Return true if this point is below a given point.
     *
     * @param p any point.
     * @return true if this is below @a p (ie. this == inf(this,p))
     * @note faster than computing the infimum and compare it afterwards.
     */
    template < typename OtherComponent, typename OtherStorage >
    bool isLower( const PointVector<dim, OtherComponent, OtherStorage>& p ) const;

    template < Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline bool
    isLower( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
             PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

    /** @brief Return true if this point is upper a given point.
     *
     * @param p any point.
     * @return true if this is upper @a p (ie. this == sup(this,p))
     * @note faster than computing the supremum and compare it afterwards.
     */
    template < typename OtherComponent, typename OtherStorage >
    bool isUpper( const PointVector<dim, OtherComponent, OtherStorage>& p ) const;

    template < Dimension ptDim,
        typename LeftEuclideanRing, typename LeftContainer,
        typename RightEuclideanRing, typename RightContainer >
    friend inline bool
    isUpper( PointVector<ptDim, LeftEuclideanRing, LeftContainer> const& lhs,
             PointVector<ptDim, RightEuclideanRing, RightContainer> const& rhs );

    /**
     * Return the maximum component value of a point/vector.
     *
     * @return the maximum value.
     */
    Component max() const;

    /**
     * Return the minimum component value of a point/vector.
     *
     * @return the minimum value.
     */
    Component min() const;

    /**
     * Return the iterator on the component with maximum value of a
     * point/vector.
     *
     * @return an iterator.
     */
    Iterator maxElement() ;

    /**
     * Return the iterator on the component with minimum value of a
     * point/vector.
     *
     * @return an iterator.
     */
    Iterator minElement() ;

    /**
       Negates this vector.
    */
    void negate();

    /**
     * Specify the set of norm types
     *
     */
    enum NormType { L_2, L_1, L_infty };

    /**
     * Computes the norm of a point/vector.
     * \warning This method performs a conversion
     * from the type T to double for each components to compute the
     * norms. For exact norms (restricted to L_1 and L_infinity
     * norms), please refer to PointVector::norm1 and PointVector::normInfinity.
     *

     * @param type specifies the type of norm to consider (see @ref NormType).
     * @return the norm of the point/vector as a double.
     */
    double norm( const NormType type = L_2 ) const;

    /**
     * Computes the square L2 norm of a point/vector.
     * \warning This method performs a conversion
     * from the type T to double for each components to compute the
     * norms.
     *
     * @return the squared norm of the point/vector as a double.
     */
    double squaredNorm( ) const;


    /**
     * Computes the 1-norm of a vector.
     *
     * @return the absolute sum of the components of this vector.
     */
    UnsignedComponent norm1() const;

    /**
     * Computes the infinity-norm of a vector.
     *
     * @return the maximum absolute value of the components of this vector.
     */
    UnsignedComponent normInfinity() const;


    /**
     * Compute the normalization of a given vector (*this) and return
     * a unitary vector on double.
     *
     * @return a unitary vector with double as coordinate type.
     * @advanced the point container is forced to std::array<double,dim>
     */
    PointVector<dim, double, std::array<double,dim> > getNormalized() const;

    /** @brief Resets all the values to zero.
     */
    void reset();

    /** @brief Resets all the values to zero.
     *
     * @note Needed by CLinearAlgebraContainer.
     */
    inline void clear() { reset(); }



    // ------------------------- Standard vectors ------------------------------
  public:

    /**
     * @param val any value.
     * @return the diagonal vector (val,val, .. val).
     */
    static Self diagonal( Component val = 1 );

    /**
     * @param k any number between 0 and Dimension-1.
     * @param val any value.
     * @return the [k]-th base vector (0,0, ..., 0, val, 0, ..., 0).
     */
    static Self base( Dimension k, Component val = 1 );

    // --------------- CDrawableWithBoard2D realization -------------------
  public:

    /**
     * @return the style name used for drawing this object.
     */
    std::string className() const;

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    /// Static const for zero PointVector.
    static Self zero;

    // ------------------------- Hidden services ------------------------------
  protected:

    ///Internal data-structure: std::array with constant size.
    Container myArray;

  }; // end of class PointVector

  /// Operator <<
  template<Dimension dim, typename Component, typename TC>
  std::ostream&
  operator<<( std::ostream & out, const PointVector<dim, Component, TC> & object );

  ///Static const for zero definition
  template< Dimension dim, typename Component, typename TC>
  PointVector<dim, Component,TC>  PointVector<dim, Component,TC>::zero;

} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions
#include "DGtal/kernel/PointVector.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PointVector_h

#undef PointVector_RECURSES
#endif // else defined(PointVector_RECURSES)
