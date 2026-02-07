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
 * @date 2018/07/17
 *
 * This file is part of the DGtal library.
 */

#if defined(PointFunctorHolder_RECURSES)
#error Recursive header files inclusion detected in PointFunctorHolder.h
#else // defined(PointFunctorHolder_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PointFunctorHolder_RECURSES

#if !defined PointFunctorHolder_h
/** Prevents repeated inclusion of headers. */
#define PointFunctorHolder_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <type_traits>
#include <utility>

#include "DGtal/base/FunctorHolder.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
namespace functors
{

/**
 * @brief Aim: hold any object callable on points as a @ref DGtal::concepts::CPointFunctor model.
 *
 * @tparam  TPoint    Point type.
 * @tparam  TValue    Value type returned by the functor.
 * @tparam  TFunctor  Type of the functor.
 *
 * The functor must accept a point and return a value whose type is @a Value.
 *
 * @warning This class is not meant to be directly constructed by the user.
 * Use instead the @ref holdPointFunctor helper that will choose the more
 * appropriate storage type for the functor depending on the given callable
 * object.
 *
 * A typical usage would be:
@code
using Point = PointVector<2, int>;
auto fn = holdPointFunctor<Point>( // auto-deduced Value template
  [] (Point const& pt) { return pt.norm(); }
);
@endcode
 *
 * In you want to use a function instead of a functor or lambda, consider
 * wrapping it into a lambda to avoid a performance penalty due to the fact
 * that a pointer to a function cannot be inlined:
@code
using Point = PointVector<2, int>;
auto fn = holdPointFunctor<Point>( // auto-deduced Value template
  [] (Point const& pt) { return my_function(pt); }
);
@endcode
 *
 * You can find more information about how to use this class appropriately
 * in the module about @ref moduleFunctors.
 *
 * @see holdPointFunctor, FunctorHolder, @ref moduleFunctors
 */
template <
  typename TPoint,
  typename TValue,
  typename TFunctor
>
class PointFunctorHolder
{
  // ----------------------- Interface --------------------------------------
public:

  // DGtal types
  using Self    = PointFunctorHolder<TPoint, TValue, TFunctor>;
  using Point   = TPoint;
  using Value   = TValue;
  using Functor = TFunctor;

  // ----------------------- Standard services ------------------------------
public:

  /** @brief Constructor
   *
   * @tparam  Function  The type of the callable object (auto-deduced).
   * @param   fn        The callable object.
   */
  template <
    typename Function,
    // SFINAE trick to disable this constructor in a copy/move construction context.
    typename std::enable_if<!std::is_base_of<PointFunctorHolder, typename std::decay<Function>::type>::value, int>::type = 0
  >
  explicit PointFunctorHolder(Function && fn)
      : myFunctor(std::forward<Function>(fn))
  {
  }

  // ----------------------- Interface --------------------------------------
public:

  /** @brief Evaluates the functor at the given point.
   *
   * @param aPoint  The point.
   */
  inline
  Value operator() ( Point const& aPoint ) const
    {
      return myFunctor( aPoint );
    }

  /** @brief Writes/Displays the object on an output stream.
   *
   * @param out the output stream where the object is written.
   */
  inline
  void selfDisplay ( std::ostream & out ) const
    {
      out << "[PointFunctorHolder] holding a " << myFunctor;
    }

  /** @brief Checks the validity/consistency of the object.
   *
   * @return 'true' if the object is valid, 'false' otherwise.
   */
  inline constexpr
  bool isValid() const
    {
      return true;
    }


  // ------------------------- Private Data --------------------------------
private:
  Functor myFunctor; ///< The held functor.

}; // End of class PointFunctorHolder


/** @brief Overloads 'operator<<' for displaying objects of class @ref PointFunctorHolder.
 *
 * @param out the output stream where the object is written.
 * @param object the object of class @ref PointFunctorHolder to write.
 * @return the output stream after the writing.
 */
template <typename TPoint, typename TValue, typename TFunctor>
std::ostream&
operator<< ( std::ostream & out, const PointFunctorHolder<TPoint, TValue, TFunctor> & object )
{
  object.selfDisplay(out);
  return out;
}


/** @brief PointFunctorHolder construction helper with specification of the return type.
 *
 * @tparam  TPoint    The point type.
 * @tparam  TValue    The functor value type.
 * @tparam  TFunctor  The functor type (auto-deduced).
 * @param   aFunctor  The functor to hold.
 * @return an instance of the appropriate PointFunctorHolder type.
 *
 * @see PointFunctorHolder
 */
template <
  typename TPoint,
  typename TValue,
  typename TFunctor
>
inline auto
holdPointFunctor( TFunctor && aFunctor )
    -> PointFunctorHolder<TPoint, TValue, decltype(holdFunctor(std::forward<TFunctor>(aFunctor)))>
  {
    return PointFunctorHolder<TPoint, TValue, decltype(holdFunctor(std::forward<TFunctor>(aFunctor)))>{
        holdFunctor(std::forward<TFunctor>(aFunctor))
    };
  }

/** @brief PointFunctorHolder construction helper with auto-deduction of the return type.
 *
 * @tparam  TPoint    The point type.
 * @tparam  TFunctor  The functor type (auto-deduced).
 * @param   aFunctor  The functor to hold.
 * @return an instance of the appropriate PointFunctorHolder type.
 *
 * @see PointFunctorHolder and @ref moduleFunctors for more information.
 */
template <
  typename TPoint,
  typename TFunctor
>
inline auto
holdPointFunctor( TFunctor && aFunctor )
    -> PointFunctorHolder<
          TPoint,
          typename std::decay<decltype(aFunctor(std::declval<TPoint>()))>::type,
          decltype(holdFunctor(std::forward<TFunctor>(aFunctor)))
        >
  {
    return PointFunctorHolder<
        TPoint,
        typename std::decay<decltype(aFunctor(std::declval<TPoint>()))>::type,
        decltype(holdFunctor(std::forward<TFunctor>(aFunctor)))
      >{ holdFunctor(std::forward<TFunctor>(aFunctor)) };
  }

} // namespace functors
} // namespace DGtal

#endif // !defined PointFunctorHolder_h

#undef PointFunctorHolder_RECURSES
#endif // else defined(PointFunctorHolder_RECURSES)
