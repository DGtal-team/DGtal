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
 * @file
 * @author Roland Denis (\c denis@math.univ-lyon1.fr )
 * CNRS, Institut Camille Jordan (UMR 5208), Université Lyon 1, France
 *
 * @date 2018/07/09
 *
 * This file is part of the DGtal library.
 */

#if defined(FunctorConstImage_RECURSES)
#error Recursive header files inclusion detected in FunctorConstImage.h
#else // defined(FunctorConstImage_RECURSES)
/** Prevents recursive inclusion of headers. */
#define FunctorConstImage_RECURSES

#if !defined FunctorConstImage_h
/** Prevents repeated inclusion of headers. */
#define FunctorConstImage_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <type_traits>
#include <utility>
#include <iterator>

#include "boost/iterator/transform_iterator.hpp"
#include "boost/concept/assert.hpp"
#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/base/FunctorHolder.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/**
 * @brief Transform a point-dependant functor into a constant image.
 *
 * @tparam TDomain  Domain type.
 * @tparam TValue   Value type returned by the functor.
 * @tparam TFunctor Type of the functor.
 *
 * The functor must accept a point, and eventually a domain, and return
 *    a value whose type is \a Value.
 *
 * @warning This class is not meant to be directly constructed by the user.
 * As illustrated below, use instead the makeFunctorConstImage helper that
 * will choose the more appropriate storage type for the functor depending
 * on the given callable object.
 *
 *
 * @see makeFunctorConstImage, FunctorHolder
 *
 * @todo domain storage ?
 */
template <
  typename TDomain,
  typename TValue,
  typename TFunctor
>
class FunctorConstImage
{
  BOOST_CONCEPT_ASSERT(( DGtal::concepts::CDomain<TDomain> ));

  // ----------------------- Interface --------------------------------------
public:

  // DGtal types
  using Self      = FunctorConstImage<TDomain, TValue, TFunctor>;
  using Domain    = TDomain;
  using Point     = typename Domain::Point;
  using Vector    = typename Domain::Vector;
  using Integer   = typename Domain::Integer;
  using Size      = typename Domain::Size;
  using Dimension = typename Domain::Dimension;
  using Vertex    = Point;
  using Value     = TValue;
  using Functor   = TFunctor;

  using ConstIterator = boost::transform_iterator< std::reference_wrapper<const Self>, typename Domain::ConstIterator >;
  using ConstReverseIterator = std::reverse_iterator< ConstIterator >;
  class ConstRange;

  BOOST_STATIC_CONSTANT( Dimension, dimension = Domain::Space::dimension );

  // ------------------------- Private Datas --------------------------------
  // Private members moved to the beginning of the class because of
  //  the SFINAE trick in the operator() methods.
private:
  Domain  myDomain;   ///< The image domain.
  Functor myFunctor;  ///< The functor that generates the image.

  // ----------------------- Standard services ------------------------------
public:

  /** Constructor
   * @param aDomain   The domain of the image.
   * @param aFunctor  The functor taking point as parameter.
   */
  template < class TGivenFunctor >
  FunctorConstImage( Domain const& aDomain, TGivenFunctor && aFunctor )
    : myDomain( aDomain )
    , myFunctor( std::forward<TGivenFunctor>(aFunctor) )
  {
  }

  // ----------------------- Interface --------------------------------------
public:

  /**
   * @return the associated domain.
   */
  inline
  Domain const& domain() const
    {
      return myDomain;
    }

  //@{

  /** Gets the value of the functor for the given point.
   * @param aPoint the point.
   * @return the value at \a aPoint.
   *
   * @todo merge doc of the two versions of operator()
   */
  template <typename TPoint>
  inline
  auto operator() ( TPoint const& aPoint ) const
      -> decltype( myFunctor( aPoint ) )
    {
      ASSERT_MSG(
          myDomain.isInside(aPoint),
          "The point is outside the domain."
      );
      return myFunctor( aPoint );
    }

  template <typename TPoint>
  inline
  auto operator() ( TPoint const& aPoint ) const
      -> decltype( myFunctor( aPoint, myDomain ) )
    {
      ASSERT_MSG(
          myDomain.isInside(aPoint),
          "The point is outside the domain."
      );
      return myFunctor( aPoint, myDomain );
    }

  //@}
  /**
   * @return a constant range over this image.
   */
  inline
  ConstRange constRange() const
    {
      return ConstRange( *this );
    }

  /**
   * Writes/Displays the object on an output stream.
   * @param out the output stream where the object is written.
   */
  inline
  void selfDisplay ( std::ostream & out ) const
    {
      out << "[FunctorConstImage] holding a " << myFunctor << " on domain " << myDomain;
    }

  /**
   * Checks the validity/consistency of the object.
   * @return 'true' if the object is valid, 'false' otherwise.
   */
  inline constexpr
  bool isValid() const
    {
      return true;
    }

public:
  /// Constant range
  class ConstRange
    {
  public:
      ConstRange( Self const& aFunctorConstImage )
        : myFunctorConstImage( aFunctorConstImage )
      {}

      using ConstIterator = Self::ConstIterator;
      using ConstReverseIterator = Self::ConstReverseIterator;
      using Point = Self::Point;

      inline ConstIterator begin()  const { return { myFunctorConstImage.myDomain.begin(), myFunctorConstImage }; }
      inline ConstIterator begin( Point const& aPoint ) const { return { myFunctorConstImage.myDomain.begin(aPoint), myFunctorConstImage }; }
      inline ConstIterator end()    const { return { myFunctorConstImage.myDomain.end(), myFunctorConstImage }; }

      inline ConstReverseIterator rbegin()  const { return ConstReverseIterator( end() ); }
      inline ConstReverseIterator rbegin( Point const& aPoint ) const { return ConstReverseIterator( ++begin(aPoint) ); }
      inline ConstReverseIterator rend()    const { return ConstReverseIterator( begin() ); }

  private:
      Self const& myFunctorConstImage;
    }; // End of class ConstRange
}; // End of class FunctorConstImage

/**
 * Overloads 'operator<<' for displaying objects of class 'XXX'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'XXX' to write.
 * @return the output stream after the writing.
 */
template <typename TDomain, typename TValue, typename TFunctor>
std::ostream&
operator<< ( std::ostream & out, const FunctorConstImage<TDomain, TValue, TFunctor> & object )
{
  object.selfDisplay(out);
  return out;
}

/** FunctorConstImage construction helper with specification of the return type.
 *
 * @tparam  TValue    The image value type.
 * @tparam  TDomain   The domain type (auto-deduced).
 * @tparam  TFunctor  The functor type (auto-deduced).
 * @param   aDomain   The image domain.
 * @param   aFunctor  The functor that generates the image.
 * @return an instance of the appropriate FunctorConstImage type.
 *
 * @see FunctorConstImage
 */
template <
  typename TValue,
  typename TDomain,
  typename TFunctor
>
auto
makeFunctorConstImage( TDomain const& aDomain, TFunctor && aFunctor )
    -> FunctorConstImage<TDomain, TValue, decltype(holdFunctor(std::forward<TFunctor>(aFunctor)))>
  {
    return { aDomain, holdFunctor(std::forward<TFunctor>(aFunctor)) };
  }

//@{

/** FunctorConstImage construction helper with auto-deduction of the return type.
 *
 * @tparam  TDomain   The domain type (auto-deduced).
 * @tparam  TFunctor  The functor type (auto-deduced).
 * @param   aDomain   The image domain.
 * @param   aFunctor  The functor that generates the image.
 * @return an instance of the appropriate FunctorConstImage type.
 *
 * @see FunctorConstImage
 */
template <
  typename TDomain,
  typename TFunctor
>
auto
makeFunctorConstImage( TDomain const& aDomain, TFunctor && aFunctor )
    -> FunctorConstImage<
          TDomain,
          typename std::decay<decltype(aFunctor(std::declval<typename TDomain::Point>()))>::type,
          decltype(holdFunctor(std::forward<TFunctor>(aFunctor)))
        >
  {
    return { aDomain, holdFunctor(std::forward<TFunctor>(aFunctor)) };
  }

template <
  typename TDomain,
  typename TFunctor
>
auto
makeFunctorConstImage( TDomain const& aDomain, TFunctor && aFunctor )
    -> FunctorConstImage<
          TDomain,
          typename std::decay<decltype(aFunctor(std::declval<typename TDomain::Point>(), aDomain))>::type,
          decltype(holdFunctor(std::forward<TFunctor>(aFunctor)))
        >
  {
    return { aDomain, holdFunctor(std::forward<TFunctor>(aFunctor)) };
  }

//@}

} // namespace DGtal

#endif // !defined FunctorConstImage_h

#undef FunctorConstImage_RECURSES
#endif // else defined(FunctorConstImage_RECURSES)
