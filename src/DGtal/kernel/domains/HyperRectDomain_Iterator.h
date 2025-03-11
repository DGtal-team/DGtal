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
 * @file HyperRectDomain_Iterator.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * @author Guillaume Damiand
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/31
 *
 * Header file for module HyperRectDomain_Iterator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(HyperRectDomain_Iterator_RECURSES)
#error Recursive header files inclusion detected in HyperRectDomain_Iterator.h
#else // defined(HyperRectDomain_Iterator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define HyperRectDomain_Iterator_RECURSES

#if !defined HyperRectDomain_Iterator_h
/** Prevents repeated inclusion of headers. */
#define HyperRectDomain_Iterator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include <iterator>
#include <type_traits>

#include <boost/iterator/iterator_facade.hpp>

#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /** @brief Reverse iterator for HyperRectDomain
   *
   * @tparam TIterator  Iterator type on HyperRectDomain
   *
   * @note We need this specific implementation of a reverse iterator instead
   *  of a boost::reverse_iterator because the latter works only on non-stashing
   *  iterators (i.e. iterators that don't own its pointed data).
   *  Otherwise, it will lead to dangling reference!
   */
  template <typename TIterator>
  class HyperRectDomain_ReverseIterator
    : public boost::iterator_facade <
        HyperRectDomain_ReverseIterator<TIterator>,
        typename TIterator::Point const,
        std::random_access_iterator_tag,
        typename TIterator::Point const&,
        typename std::iterator_traits<TIterator>::difference_type
      >
  {
  public:
    using Iterator = TIterator;
    using Self  = HyperRectDomain_ReverseIterator<Iterator>;
    using Point = typename Iterator::Point;
    using Dimension = typename Point::Dimension;
    using DifferenceType = typename std::iterator_traits<Self>::difference_type; ///< Type of the difference between two iterators (usually std::ptrdiff_t except for BigInteger).

  public:
    /// @brief Constructor from a HyperRectDomain iterator
    explicit HyperRectDomain_ReverseIterator(Iterator it)
      : current(it)
      , prev(it)
    {
      --prev;
    }

    /// @brief Dereference
    const Point& dereference() const
      {
        return *prev;
      }

    /// @brief Compare iterators
    bool equal( const Self &other ) const
      {
        return current == other.current;
      }

    /// @brief Increment iterator
    void increment()
      {
        --current;
        --prev;
      }

    /// @brief Decrement iterator
    void decrement()
      {
        ++current;
        ++prev;
      }

    /// @brief Advance iterator by given steps
    void advance( DifferenceType const& n )
      {
        current -= n;
        prev -= n;
      }

    /// @brief Distance between two iterators on the same domain
    DifferenceType distance_to( const Self& other ) const
      {
        return std::distance(other.current, current);
      }

  private:
    Iterator current, prev;

  };

  /////////////////////////////////////////////////////////////////////////////
  // class HyperRectDomain_Iterator
  /** @brief Iterator for HyperRectDomain
   *
   * @tparam TPoint Point type.
   */
  template <typename TPoint>
  class HyperRectDomain_Iterator
    : public boost::iterator_facade <
        HyperRectDomain_Iterator<TPoint>,
        TPoint const,
        std::random_access_iterator_tag,
        TPoint const&,
#ifdef WITH_BIGINTEGER
        typename std::conditional<std::is_same<typename TPoint::Component, BigInteger>::value, BigInteger, std::ptrdiff_t>::type
#else
        std::ptrdiff_t
#endif
      >
  {
  public:
    using Point = TPoint;
    using Self  = HyperRectDomain_Iterator<TPoint>;
    using Dimension = typename Point::Dimension;
    using DifferenceType = typename std::iterator_traits<Self>::difference_type; ///< Type of the difference between two iterators (usually std::ptrdiff_t except for BigInteger).


    /** @brief HyperRectDomain iterator constructor
     *
     * @param p     The point pointed by this iterator
     * @param lower Lower bound of the iterated domain
     * @param upper Upper bound of the iterated domain
     *
     * @pre @a p must lie inside the given bounds or be equal to one of its bound
     * @pre the bounds must describe a valid (possibly empty) domain
     */
    HyperRectDomain_Iterator( const Point & p, const Point& lower, const Point &upper )
      : myPoint( p ), mylower( lower ), myupper( upper )
      {
        ASSERT_MSG( // For an empty domain, lower = upper + diag(1) so that begin() == end().
            lower.isLower(upper) || lower == upper + TPoint::diagonal(1),
            "The lower bound must be lower than the upper bound or, for an empty domain, be equal to the upper bound + diagonal(1)."
        );

        ASSERT_MSG(
            ( lower.isLower(p) && p.isLower(upper) ) || p == lower || p == upper,
            "The point must be inside the domain or be equal to one of his bound."
        );

        // Calculating iterator position in the sequence
        pos = 0;
        DifferenceType delta = 1;
        for ( Dimension i = 0; i < Point::dimension; ++i )
          {
            pos += delta * (myPoint[i] - mylower[i]);
            delta *= myupper[i] - mylower[i] + 1;
          }
      }

  private:
    friend class boost::iterator_core_access;

    /// @brief Dereference
    const Point& dereference() const
      {
        ASSERT_MSG( // we must be between [begin,end]
            mylower.isLower(myPoint) && myPoint.isLower(myupper),
            "The iterator points outside the domain."
        );

        return myPoint;

      }

    /** @brief Compare iterators
     *
     * @note compare only the pointed point, not the iterated domain.
     */
    bool equal( const Self &other ) const
      {
        ASSERT_MSG( // we should only compare iterators on the same domain
            mylower == other.mylower && myupper == other.myupper,
            "The compared iterators iterate on different domains."
        );

        return pos == other.pos;
      }

    /** @brief
     * Increments the iterator in order to scan the domain points dimension by dimension
     * (lexicographic order).
     */
    void increment()
      {
        ++pos;
        ++myPoint[0];
        for ( Dimension i = 0; myPoint[i] > myupper[i] && i < Point::dimension - 1; ++i )
          {
            ++myPoint[i+1];
            myPoint[i] = mylower[i];
          }
      }

    /** @brief
     * Decrements the iterator in order to scan the domain points dimension by dimension
     * (lexicographic order).
     **/
    void decrement()
      {
        --pos;
        --myPoint[0];
        for ( Dimension i = 0; myPoint[i] < mylower[i] && i < Point::dimension - 1; ++i )
          {
            --myPoint[i+1];
            myPoint[i] = myupper[i];
          }
      }

    /** @brief
     * Advances the iterator in order to scan the domain points dimension by dimension
     * (lexicographic order).
     */
    void advance( DifferenceType const& n )
      {
        pos += n;
        if (n > 0)
          {
            myPoint[0] += n;
            for ( Dimension i = 0; myPoint[i] > myupper[i] && i < Point::dimension - 1; ++i )
              {
                typename Point::Component const shift = myPoint[i] - mylower[i];
                typename Point::Component const length = myupper[i] - mylower[i] + 1;
                myPoint[i+1] += shift / length;
                myPoint[i] = mylower[i] + (shift % length);
              }
          }
        else if (n < 0)
          {
            myPoint[0] += n;
            for ( Dimension i = 0; myPoint[i] < mylower[i] && i < Point::dimension - 1; ++i )
              {
                typename Point::Component const shift = myupper[i] - myPoint[i];
                typename Point::Component const length = myupper[i] - mylower[i] + 1;
                myPoint[i+1] -= shift / length;
                myPoint[i] = myupper[i] - (shift % length);
              }
          }
      }

    /** @brief
     * Distance between two iterators on the same domain (lexicographic order).
     */
    DifferenceType distance_to( const Self& other ) const
      {
        ASSERT_MSG( // we should only compare iterators on the same domain
            mylower == other.mylower && myupper == other.myupper,
            "The compared iterators iterate on different domains."
        );

        return other.pos - pos;
      }

  private:
    ///Current Point in the domain
    TPoint myPoint;

    ///Copies of the Domain limits
    TPoint mylower, myupper;

    /// Iterator position in the current sequence
    DifferenceType pos;

  }; // End of class HyperRectDomain_Iterator

  /////////////////////////////////////////////////////////////////////////////
  // class HyperRectDomain_subIterator
  /**
   * Description of class 'HyperRectDomain_subIterator' <p>
   * Aim:
   */
  template<typename TPoint>
  class HyperRectDomain_subIterator
    : public boost::iterator_facade <
        HyperRectDomain_subIterator<TPoint>,
        const TPoint,
        std::random_access_iterator_tag,
        TPoint const&,
#ifdef WITH_BIGINTEGER
        typename std::conditional<std::is_same<typename TPoint::Component, BigInteger>::value, BigInteger, std::ptrdiff_t>::type
#else
        std::ptrdiff_t
#endif
      >
  {
  public:
    using Point = TPoint;
    using Self  = HyperRectDomain_subIterator<TPoint>;
    using Dimension = typename Point::Dimension;
    using DifferenceType = typename std::iterator_traits<Self>::difference_type; ///< Type of the difference between two iterators (usually std::ptrdiff_t except for BigInteger).

    HyperRectDomain_subIterator(const TPoint & p, const TPoint& lower,
        const TPoint &upper,
        const std::vector<Dimension> &subDomain)
      : myPoint( p ), mylower( lower ), myupper( upper )
      {
        ASSERT_MSG( // For an empty domain, lower = upper + diag(1) so that begin() == end().
            lower.isLower(upper) || lower == upper + TPoint::diagonal(0).partialCopy( TPoint::diagonal(1), subDomain ),
            "The lower bound must be lower than the upper bound or, for an empty domain, be equal to the upper bound + diagonal(1)."
        );

        ASSERT_MSG(
            ( lower.isLower(p) && p.isLower(upper) ) || p == lower || p == upper,
            "The point must be inside the domain or be equal to one of his bound."
        );

        ASSERT_MSG(
            subDomain.size() <= TPoint::dimension,
            "The sub-range cannot have more dimensions than the ambiant space."
        );

        mySubDomain.reserve( subDomain.size() );
        for ( typename std::vector<Dimension>::const_iterator it = subDomain.begin();
            it != subDomain.end(); ++it )
          {
            ASSERT_MSG(
                *it <= TPoint::dimension,
                "Invalid dimension in the sub-range."
            );
            mySubDomain.push_back( *it );
          }

        // Calculating iterator position in the sequence
        pos = 0;
        DifferenceType delta = 1;
        for ( Dimension i = 0; i < mySubDomain.size(); ++i )
          {
            auto const ii = mySubDomain[i];
            pos += delta * (myPoint[ii] - mylower[ii]);
            delta *= myupper[ii] - mylower[ii] + 1;
          }
      }

  private:
    friend class boost::iterator_core_access;

    /// @brief Dereference
    const Point& dereference() const
      {
        ASSERT_MSG( // we must be between [begin,end]
            mylower.isLower(myPoint) && myPoint.isLower(myupper),
            "The iterator points outside the domain."
        );

        return myPoint;

      }

    /** @brief Compare iterators
     *
     * @note compare only the pointed point, not the iterated domain.
     */
    bool equal( const Self &other ) const
      {
        ASSERT_MSG( // we should only compare iterators on the same domain and same dimensions
            mylower == other.mylower && myupper == other.myupper && mySubDomain == other.mySubDomain,
            "The compared iterators iterate on different domains or different dimensions."
        );

        return pos == other.pos;
      }


    /** @brief
     * Increments the iterator in order to scan the domain points dimension by dimension
     * (by using the subDomain order given by the user).
     */
    void increment()
      {
        ++pos;
        ++myPoint[mySubDomain[0]];
        for ( Dimension i = 0; myPoint[mySubDomain[i]] > myupper[mySubDomain[i]] && i < mySubDomain.size() - 1; ++i )
          {
            ++myPoint[mySubDomain[i+1]];
            myPoint[mySubDomain[i]] = mylower[mySubDomain[i]];
          }
      }

    /** @brief
     * Decrements the iterator in order to scan the domain points dimension by dimension
     * (by using the subDomain order given by the user).
     **/
    void decrement()
      {
        --pos;
        --myPoint[mySubDomain[0]];
        for ( Dimension i = 0; myPoint[mySubDomain[i]] < mylower[mySubDomain[i]] && i < mySubDomain.size() - 1; ++i )
          {
            --myPoint[mySubDomain[i+1]];
            myPoint[mySubDomain[i]] = myupper[mySubDomain[i]];
          }
      }

    /** @brief
     * Advances the iterator in order to scan the domain points dimension by dimension
     * (by using the subDomain order given by the user).
     */
    void advance( DifferenceType const& n )
      {
        pos += n;
        if (n > 0)
          {
            myPoint[mySubDomain[0]] += n;
            for ( Dimension i = 0; myPoint[mySubDomain[i]] > myupper[mySubDomain[i]] && i < mySubDomain.size() - 1; ++i )
              {
                auto const ii = mySubDomain[i];
                typename Point::Component const shift = myPoint[ii] - mylower[ii];
                typename Point::Component const length = myupper[ii] - mylower[ii] + 1;
                myPoint[mySubDomain[i+1]] += shift / length;
                myPoint[ii] = mylower[ii] + (shift % length);
              }
          }
        else if (n < 0)
          {
            myPoint[mySubDomain[0]] += n;
            for ( Dimension i = 0; myPoint[mySubDomain[i]] < mylower[mySubDomain[i]] && i < mySubDomain.size() - 1; ++i )
              {
                auto const ii = mySubDomain[i];
                typename Point::Component const shift = myupper[ii] - myPoint[ii];
                typename Point::Component const length = myupper[ii] - mylower[ii] + 1;
                myPoint[mySubDomain[i+1]] -= shift / length;
                myPoint[ii] = myupper[ii] - (shift % length);
              }
          }
      }

    /** @brief
     * Distance between two iterators on the same domain
     * (by using the subDomain order given by the user).
     */
    DifferenceType distance_to( const Self& other ) const
      {
        ASSERT_MSG( // we should only compare iterators on the same domain and same dimensions
            mylower == other.mylower && myupper == other.myupper && mySubDomain == other.mySubDomain,
            "The compared iterators iterate on different domains or different dimensions."
        );

        return other.pos - pos;
      }

  private:
    ///Current Point in the domain
    TPoint myPoint;

    ///Copies of the Domain limits
    TPoint mylower, myupper;

    /** Vector of subDomain on dimension, to fix the order in which dimensions
     * are considered.
     */
    std::vector<Dimension> mySubDomain;

    /// Iterator position in the current sequence
    DifferenceType pos;

  }; // End of class HyperRectDomain_subIterator

} //namespace
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined HyperRectDomain_Iterator_h

#undef HyperRectDomain_Iterator_RECURSES
#endif // else defined(HyperRectDomain_Iterator_RECURSES)
