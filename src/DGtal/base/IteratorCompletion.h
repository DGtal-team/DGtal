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
 * @file IteratorCompletion.h
 * @author Roland Denis (\c roland.denis@univ-smb.fr )
 * LAboratory of MAthematics - LAMA (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2015/06/23
 *
 * This file is part of the DGtal library.
 */

#if defined(IteratorCompletion_RECURSES)
#error Recursive header files inclusion detected in IteratorCompletion.h
#else // defined(IteratorCompletion_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IteratorCompletion_RECURSES

#if !defined IteratorCompletion_h
/** Prevents repeated inclusion of headers. */
#define IteratorCompletion_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <boost/iterator/reverse_iterator.hpp>
#include <DGtal/base/SimpleRandomAccessRangeFromPoint.h>
#include <DGtal/base/SimpleRandomAccessConstRangeFromPoint.h>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  /** 
   * @brief Aim: Traits that must be specialized for each IteratorCompletion derived class.
   *
   * This traits must shown:
   * - a typedef Iterator corresponding to the derived class mutable iterator.
   * - a typedef ConstIterator corresponding to the derived class constant iterator.
   * - a class DistanceFunctor, constructible from a pointer to the derived class and 
   *   that behaves like a distance functor from the begin() iterator to a given point.
   *   (\see SimpleRandomAccessRangeFromPoint and SimpleRandomAccessConstRangeFromPoint)
   */
  template < typename TDerived >
  class IteratorCompletionTraits;

  /**
   * @brief Aim: Class that uses CRTP to add reverse iterators and ranges to a derived class.
   *
   * Description of template class 'IteratorCompletionTraits' <p>
   * This class adds new iterators to a given class if it provides a minimal set of iterators.
   *
   * More precisely, it provides:
   * - reverse iterators if the class provides a bidirectional iterator,
   * - ranges if the class provides a random access iterator and a distance functor from a point.
   *
   * Each derived class of IteratorCompletion must specialized IteratorCompletionTraits in order to provide
   * enough informations on his iterators.
   *
   * \see ArrayImageView.h for usage example.
   */
  template < 
    typename TDerived
  >
  class IteratorCompletion
    {
    public:

      typedef typename IteratorCompletionTraits<TDerived>::Iterator           Iterator;
      typedef typename IteratorCompletionTraits<TDerived>::ConstIterator      ConstIterator;
      typedef typename IteratorCompletionTraits<TDerived>::DistanceFunctor    DistanceFunctor;

      typedef boost::reverse_iterator<Iterator>       ReverseIterator;
      typedef boost::reverse_iterator<ConstIterator>  ConstReverseIterator;
      typedef SimpleRandomAccessRangeFromPoint< ConstIterator, Iterator, DistanceFunctor >  Range;
      typedef SimpleRandomAccessConstRangeFromPoint< ConstIterator, DistanceFunctor >       ConstRange;
      typedef std::ptrdiff_t    Difference;

      /**
       * @return  a mutable reverse-iterator pointing to the last value.
       * @warning the derived class must have a end() method that return a mutable bidirectional iterator.
       */
      ReverseIterator rbegin()
        {
          return ReverseIterator{ static_cast<TDerived*>(this)->end() };
        }

      /**
       * @return  a constant reverse-iterator pointing to the last value.
       * @warning the derived class must have a end() method that return a constant bidirectional iterator.
       */
      inline
      ConstReverseIterator rbegin() const
        {
          return ConstReverseIterator{ static_cast<TDerived*>(this)->end() };
        }
      
      /**
       * @return  a constant reverse-iterator pointing to the last value (C++11).
       * @warning the derived class must have a cend() method that return a constant bidirectional iterator.
       */
      inline
      ConstReverseIterator crbegin() const
        {
          return ConstReverseIterator{ static_cast<TDerived*>(this)->cend() };
        }
      
      /**
       * @return  a mutable reverse-iterator pointing before the first value.
       * @warning the derived class must have a begin() method that return a mutable bidirectional iterator.
       */
      inline
      ReverseIterator rend()
        {
          return ReverseIterator{ static_cast<TDerived*>(this)->begin() };
        }

      /**
       * @return  a constant reverse-iterator pointing before the first value.
       * @warning the derived class must have a begin() method that return a constant bidirectional iterator.
       */
      inline
      ConstReverseIterator rend() const
        {
          return ConstReverseIterator{ static_cast<TDerived*>(this)->begin() };
        }
      
      /**
       * @return  a constant reverse-iterator pointing before the first value (C++11).
       * @warning the derived class must have a cbegin() method that return a constant bidirectional iterator.
       */
      inline
      ConstReverseIterator crend() const
        {
          return ConstReverseIterator{ static_cast<TDerived*>(this)->cbegin() };
        }

      /**
       * @return  a mutable range over the derived class values.
       * @warning the derived class must have begin() and end() methods that returns mutable random-access iterators.
       * @warning In addition, the class must provide a distance functor to a point.
       */
      inline
      Range range()
        {
          TDerived* const derived = static_cast<TDerived*>(this);
          return { 
              derived->begin(), 
              derived->end(),
              typename IteratorCompletionTraits<TDerived>::DistanceFunctor( derived )
          };
        }

      /**
       * @return  a constant range over the derived class values.
       * @warning the derived class must have begin() and end() methods that returns constant random-access iterators.
       * @warning In addition, the class must provide a distance functor to a point.
       */
      inline
      ConstRange constRange() const
        {
          TDerived const* const derived = static_cast<TDerived const*>(this);
          return { 
              derived->begin(), 
              derived->end(), 
              typename IteratorCompletionTraits<TDerived>::DistanceFunctor( derived )
          };
        }

    protected:

      /// Protected destructor to avoid memory leak.
      ~IteratorCompletion()
        {}
    };

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IteratorCompletion_h

#undef IteratorCompletion_RECURSES
#endif // else defined(IteratorCompletion_RECURSES)

/* GNU coding style */
/* vim: set ts=2 sw=2 expandtab cindent cinoptions=>4,n-2,{2,^-2,:2,=2,g0,h2,p5,t0,+2,(0,u0,w1,m1 : */
