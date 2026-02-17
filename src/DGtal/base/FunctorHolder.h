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
 * @date 2018/06/22
 *
 * This file is part of the DGtal library.
 */

#if defined(FunctorHolder_RECURSES)
#error Recursive header files inclusion detected in FunctorHolder.h
#else // defined(FunctorHolder_RECURSES)
/** Prevents recursive inclusion of headers. */
#define FunctorHolder_RECURSES

#if !defined FunctorHolder_h
/** Prevents repeated inclusion of headers. */
#define FunctorHolder_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <type_traits>
#include <functional>
#include <memory>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
namespace functors
{

//////////////////////////////////////////////////////////////////////////////
// Implementation details
namespace
{

  /// Invoke a callable object without dereferencing it before.
  template <bool>
  struct Invoker
  {
    template <typename F, typename... T>
    static inline
    auto apply(F && fn, T &&... args)
        -> decltype((std::forward<F>(fn))(std::forward<T>(args)...))
      {
        return (std::forward<F>(fn))(std::forward<T>(args)...);
      }
  };

  /// Invoke a callable object by dereferencing it before.
  template <>
  struct Invoker<true>
  {
    template <typename F, typename... T>
    static inline
    auto apply(F && fn, T &&... args)
        -> decltype((*std::forward<F>(fn))(std::forward<T>(args)...))
      {
        return (*std::forward<F>(fn))(std::forward<T>(args)...);
      }
  };

  /// Type traits to detect storage based on std::shared_ptr (for debug purpose)
  struct SharedPtrTrait
    {
      template <typename T>
      static constexpr std::true_type   apply(std::shared_ptr<T> const&) { return {}; }

      template <typename T>
      static constexpr std::false_type  apply(T const&) { return {}; }
    };

  /// Type traits to detect storage based on std::reference_wrapper (for debug purpose)
  struct ReferenceWrapperTrait
    {
      template <typename T>
      static constexpr std::true_type   apply(std::reference_wrapper<T> const&) { return {}; }

      template <typename T>
      static constexpr std::false_type  apply(T const&) { return {}; }
    };

} // anonymous namespace


/////////////////////////////////////////////////////////////////////////////
// class FunctorHolder
/**
 * @brief Aim: hold any callable object (function, functor, lambda, ...) as a C(Unary)Functor model.
 *
 * @tparam FunctorStorage  Type used to store the given callable object.
 * @tparam NeedDereference  true if the object must be dereferenced before calls.
 *
 * @warning This class is not meant to be directly constructed by the user.
 * As explained below, you should use instead the @ref holdFunctor helper that
 * will choose the more appropriate storage type depending on the given
 * callable object.
 *
 * Given any type of callable object (function, functor and lambda) passed
 * by value, reference or pointer, stores it in the most appropriate way so
 * that the returned object is compatible with DGtal functor concept
 * (@ref concepts::CUnaryFunctor so far), and especially the @ref boost::Assignable concept.
 *
 * More precisely, the storage mode depends on the lifetime of the given
 * object, deduced from its passing mode :
 * - if the given object is passed by left-value reference (long lifetime),
 *   it will be stored as a reference (using `std::reference_wrapper`).
 * - otherwise (short lifetime, like right-value references),
 *   it will be moved to a dynamically allocated memory place (`std::shared_ptr`)
 *   so that the object lifetime matches the FunctorHolder lifetime.
 *
 * In both case, the returned object is copy/move constructible and
 * copy/move assignable so that it can be used as a functor in all DGtal
 * library.
 *
 * @remark It is important to not explicitly specify the given object type
 * so that it's real lifetime can be deduced using universal reference.
 * However, you can transfer ownership of the object to FunctorHolder by moving
 * it using `std::move`.
 * As a consequence, the type of the returned FunctorHolder cannot be guessed
 * easily and the use of the `auto` keyword is thus mandatory.
 *
 * More information can be found in the dedicate module page about
 * @ref moduleFunctors .
 *
 * @see holdFunctor
 */
template <
  typename FunctorStorage,
  bool NeedDereference
>
class FunctorHolder
{
  // ------------------------- Private Data --------------------------------
private:

  FunctorStorage myFunctor; ///< The callable object.

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
    typename std::enable_if<!std::is_base_of<FunctorHolder, typename std::decay<Function>::type>::value, int>::type = 0
  >
  explicit FunctorHolder(Function && fn)
      : myFunctor(std::forward<Function>(fn))
  {
  }

  // ----------------------- Interface --------------------------------------
public:

  /** @brief Invokes the stored callable object in a constant context.
   *
   * @tparam  T     Arguments's types.
   * @param   args  The arguments.
   */
  template <typename... T>
  inline auto operator() (T &&... args) const
      -> decltype(Invoker<NeedDereference>::apply(this->myFunctor, std::forward<T>(args)...))
    {
      return Invoker<NeedDereference>::apply(myFunctor, std::forward<T>(args)...);
    }

  /** @brief Invokes the stored callable object in a mutable context.
   *
   * @tparam  T     Arguments's types.
   * @param   args  The arguments.
   */
  template <typename... T>
  inline auto operator() (T &&... args)
      -> decltype(Invoker<NeedDereference>::apply(this->myFunctor, std::forward<T>(args)...))
    {
      return Invoker<NeedDereference>::apply(myFunctor, std::forward<T>(args)...);
    }

  /** @brief Writes/Displays the object on an output stream.
   *
   * @param out the output stream where the object is written.
   */
  inline
  void selfDisplay ( std::ostream & out ) const
    {
      out << "[FunctorHolder]";
      if ( SharedPtrTrait::apply(myFunctor) )
        out << " using std::shared_ptr storage (rvalue)";
      else if ( ReferenceWrapperTrait::apply(myFunctor) )
        out << " using std::reference_wrapper storage (lvalue)";
      else
        out << " using custom storage";
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
}; // End of class FunctorHolder

/** @brief Overloads 'operator<<' for displaying objects of class @ref FunctorHolder.
 *
 * @param out the output stream where the object is written.
 * @param object the object of class @ref FunctorHolder to write.
 * @return the output stream after the writing.
 */
template <typename FunctorStorage, bool NeedDereference>
std::ostream&
operator<< ( std::ostream & out, const FunctorHolder<FunctorStorage, NeedDereference> & object )
{
  object.selfDisplay( out );
  return out;
}

//////////////////////////////////////////////////////////////////////////////
// Implementation details
namespace {

  /// Overload that stores the object in a std::reference_wrapper when given object is a lvalue reference.
  template <typename Function>
  inline auto holdFunctorImpl(Function && fn, std::true_type)
      -> FunctorHolder<decltype(std::ref(std::forward<Function>(fn))), false>
    {
      return FunctorHolder<decltype(std::ref(std::forward<Function>(fn))), false>{ std::forward<Function>(fn) };
    }

  /// Overload that stores the object in a std::shared_ptr when given object isn't a lvalue reference.
  template <typename Function>
  inline auto holdFunctorImpl(Function && fn, std::false_type)
      -> FunctorHolder<std::shared_ptr<Function>, true>
  {
      return FunctorHolder<std::shared_ptr<Function>, true>{ std::make_shared<Function>(std::forward<Function>(fn)) };
  }

} // anonymous namespace


//////////////////////////////////////////////////////////////////////////////
// FunctorHolder construction helper
/** @brief Hold any callable object (function, functor, lambda, ...) as a C(Unary)Functor model.
 *
 * @tparam  Function  The callable object type (auto-deduced, do not specify it)
 * @param   fn        The callable object to be holded.
 * @return  a FunctorHolder instance with appropriate storage type depending on the given object.
 *
 * @see FunctorHolder and @ref moduleFunctors for more information
 */
template <typename Function>
inline auto holdFunctor(Function && fn)
  -> decltype(holdFunctorImpl(std::forward<Function>(fn), typename std::is_lvalue_reference<Function>{})) // Can we please pass to C++14 ?
{
  return holdFunctorImpl(std::forward<Function>(fn), typename std::is_lvalue_reference<Function>{});
}

} // namespace functors
} // namespace DGtal

#endif // !defined FunctorHolder_h

#undef FunctorHolder_RECURSES
#endif // else defined(FunctorHolder_RECURSES)
