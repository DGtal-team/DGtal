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

#if defined(ConstImageFunctorHolder_RECURSES)
#error Recursive header files inclusion detected in ConstImageFunctorHolder.h
#else // defined(ConstImageFunctorHolder_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ConstImageFunctorHolder_RECURSES

#if !defined ConstImageFunctorHolder_h
/** Prevents repeated inclusion of headers. */
#define ConstImageFunctorHolder_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <type_traits>
#include <utility>
#include <iterator>

#include <boost/iterator/transform_iterator.hpp>
#include <boost/concept/assert.hpp>
#include <boost/iterator/reverse_iterator.hpp>

#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/base/FunctorHolder.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
namespace functors
{

/**
 * @brief Transform a point-dependent (and possibly domain-dependent) functor into a constant image.
 *
 * @tparam TDomain  Domain type.
 * @tparam TValue   Value type returned by the functor.
 * @tparam TFunctor Type of the functor.
 *
 * The functor must accept a point, and eventually a domain, and return
 *    a value whose type is \a Value.
 *
 * @warning This class is not meant to be directly constructed by the user.
 * As illustrated below, use instead the holdConstImageFunctor helper that
 * will choose the more appropriate storage type for the functor depending
 * on the given callable object.
 *
 * A typical usage would be:
 * @snippet exampleConstImageFunctorHolder.cpp example1
 * resulting in:
 * \image html ConstImageFunctorHolder_example1.png "Image generated from a point-dependent lambda."
 * \image latex ConstImageFunctorHolder_example1.png "Image generated from a point-dependent lambda." width=5cm
 *
 * In you want to use a function instead of a functor or lambda, consider
 * wrapping it into a lambda to avoid a performance penalty due to the fact
 * that a pointer to a function cannot be inlined:
@code
auto image = DGtal::holdConstImageFunctor(
    domain,
    [] (Point const& pt) { return my_function(pt); }
);
@endcode
 *
 * This example is illustrated in @ref moduleImages_functorconstimage
 * and you can find more informations about how to use this class appropriately
 * in the module about @ref moduleFunctors .
 *
 * @see holdConstImageFunctor, FunctorHolder, @ref moduleFunctors
 *
 * @see exampleConstImageFunctorHolder.cpp
 *
 */
template <
  DGtal::concepts::CDomain TDomain,
  typename TValue,
  typename TFunctor
>
class ConstImageFunctorHolder
{
  // ----------------------- Interface --------------------------------------
public:

  // DGtal types
  using Self      = ConstImageFunctorHolder<TDomain, TValue, TFunctor>;
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
  using ConstReverseIterator = boost::reverse_iterator< ConstIterator >;
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

  /** @brief Constructor
   * @param aDomain   The domain of the image.
   * @param aFunctor  The functor taking point as parameter.
   */
  template < class TGivenFunctor >
  explicit ConstImageFunctorHolder( Domain const& aDomain, TGivenFunctor && aFunctor )
    : myDomain( aDomain )
    , myFunctor( std::forward<TGivenFunctor>(aFunctor) )
  {
  }

  // ----------------------- Interface --------------------------------------
public:

  /// @brief Returns the associated domain.
  inline
  Domain const& domain() const
    {
      return myDomain;
    }

  //@{

  /** @brief Evaluates the functor at the given point.
   * @tparam  TPoint  point type (auto-deduced).
   * @param   aPoint  the point.
   * @return the value at \a aPoint.
   *
   * @note There are two overloads of this operator, automatically dispatched depending on the arity of the functor.
   */
  template <typename TPoint> // Needed template parameter to enable SFINAE trick
  inline
  auto operator() ( TPoint const& aPoint ) const
      -> decltype( myFunctor( aPoint ) ) // Using SFINAE to enable this overload for unary functor
    {
      ASSERT_MSG(
          myDomain.isInside(aPoint),
          "The point is outside the domain."
      );
      return myFunctor( aPoint );
    }

  template <typename TPoint> // Needed template parameter to enable SFINAE trick
  inline
  auto operator() ( TPoint const& aPoint ) const
      -> decltype( myFunctor( aPoint, myDomain ) ) // Using SFINAE to enable this overload for binary functor
    {
      ASSERT_MSG(
          myDomain.isInside(aPoint),
          "The point is outside the domain."
      );
      return myFunctor( aPoint, myDomain );
    }

  //@}

  /** @brief Returns a constant range over this image.
   */
  inline
  ConstRange constRange() const
    {
      return ConstRange( *this );
    }

  /** @brief Writes/Displays the object on an output stream.
   * @param out the output stream where the object is written.
   */
  inline
  void selfDisplay ( std::ostream & out ) const
    {
      out << "[ConstImageFunctorHolder] holding a " << myFunctor << " on domain " << myDomain;
    }

  /** @brief Checks the validity/consistency of the object.
   * @return 'true' if the object is valid, 'false' otherwise.
   */
  inline constexpr
  bool isValid() const
    {
      return true;
    }

public:
  /// Constant range on a @ref ConstImageFunctorHolder
  class ConstRange
    {
  public:
      ConstRange( Self const& aConstImageFunctorHolder )
        : myConstImageFunctorHolder( aConstImageFunctorHolder )
      {}

      using ConstIterator = typename Self::ConstIterator;
      using ConstReverseIterator = typename Self::ConstReverseIterator;
      using Point = typename Self::Point;

      inline ConstIterator begin()  const { return { myConstImageFunctorHolder.myDomain.begin(), myConstImageFunctorHolder }; }
      inline ConstIterator begin( Point const& aPoint ) const { return { myConstImageFunctorHolder.myDomain.begin(aPoint), myConstImageFunctorHolder }; }
      inline ConstIterator end()    const { return { myConstImageFunctorHolder.myDomain.end(), myConstImageFunctorHolder }; }

      inline ConstReverseIterator rbegin()  const { return ConstReverseIterator( end() ); }
      inline ConstReverseIterator rbegin( Point const& aPoint ) const { return ConstReverseIterator( ++begin(aPoint) ); }
      inline ConstReverseIterator rend()    const { return ConstReverseIterator( begin() ); }

  private:
      Self const& myConstImageFunctorHolder;
    }; // End of class ConstRange
}; // End of class ConstImageFunctorHolder

/** @brief Overloads 'operator<<' for displaying objects of class @ref ConstImageFunctorHolder.
 *
 * @param out the output stream where the object is written.
 * @param object the object of class @ref ConstImageFunctorHolder to write.
 * @return the output stream after the writing.
 */
template <typename TDomain, typename TValue, typename TFunctor>
std::ostream&
operator<< ( std::ostream & out, const ConstImageFunctorHolder<TDomain, TValue, TFunctor> & object )
{
  object.selfDisplay(out);
  return out;
}

/** @brief ConstImageFunctorHolder construction helper with specification of the return type.
 *
 * @tparam  TValue    The image value type.
 * @tparam  TDomain   The domain type (auto-deduced).
 * @tparam  TFunctor  The functor type (auto-deduced).
 * @param   aDomain   The image domain.
 * @param   aFunctor  The functor that generates the image.
 * @return an instance of the appropriate ConstImageFunctorHolder type.
 *
 * @see ConstImageFunctorHolder
 */
template <
  typename TValue,
  typename TDomain,
  typename TFunctor
>
inline auto
holdConstImageFunctor( TDomain const& aDomain, TFunctor && aFunctor )
    -> ConstImageFunctorHolder<TDomain, TValue, decltype(holdFunctor(std::forward<TFunctor>(aFunctor)))>
  {
    return ConstImageFunctorHolder<TDomain, TValue, decltype(holdFunctor(std::forward<TFunctor>(aFunctor)))>{ aDomain, holdFunctor(std::forward<TFunctor>(aFunctor)) };
  }

//@{

/** @brief ConstImageFunctorHolder construction helper with auto-deduction of the return type.
 *
 * @tparam  TDomain   The domain type (auto-deduced).
 * @tparam  TFunctor  The functor type (auto-deduced).
 * @param   aDomain   The image domain.
 * @param   aFunctor  The functor (unary or binary) that generates the image.
 * @return an instance of the appropriate ConstImageFunctorHolder type.
 *
 * @note You don't have to choose between the version for unary or binary functor:
 *  this choice is automatically done using SFINAE technique.
 *
 * @see ConstImageFunctorHolder and @ref moduleImages for more informations.
 */

// Auto-deduction of the return type in case of an unary functor.
template <
  typename TDomain,
  typename TFunctor
>
inline auto
holdConstImageFunctor( TDomain const& aDomain, TFunctor && aFunctor )
    -> ConstImageFunctorHolder<
          TDomain,
          typename std::decay<decltype(aFunctor(std::declval<typename TDomain::Point>()))>::type,
          decltype(holdFunctor(std::forward<TFunctor>(aFunctor)))
        >
  {
    return ConstImageFunctorHolder<
        TDomain,
        typename std::decay<decltype(aFunctor(std::declval<typename TDomain::Point>()))>::type,
        decltype(holdFunctor(std::forward<TFunctor>(aFunctor)))
      >{ aDomain, holdFunctor(std::forward<TFunctor>(aFunctor)) };
  }

// Auto-deduction of the return type in case of a binary functor.
template <
  typename TDomain,
  typename TFunctor
>
inline auto
holdConstImageFunctor( TDomain const& aDomain, TFunctor && aFunctor )
    -> ConstImageFunctorHolder<
          TDomain,
          typename std::decay<decltype(aFunctor(std::declval<typename TDomain::Point>(), aDomain))>::type,
          decltype(holdFunctor(std::forward<TFunctor>(aFunctor)))
        >
  {
    return ConstImageFunctorHolder<
        TDomain,
        typename std::decay<decltype(aFunctor(std::declval<typename TDomain::Point>(), aDomain))>::type,
        decltype(holdFunctor(std::forward<TFunctor>(aFunctor)))
      >{ aDomain, holdFunctor(std::forward<TFunctor>(aFunctor)) };
  }

//@}

} // namespace functors
} // namespace DGtal

#endif // !defined ConstImageFunctorHolder_h

#undef ConstImageFunctorHolder_RECURSES
#endif // else defined(ConstImageFunctorHolder_RECURSES)
