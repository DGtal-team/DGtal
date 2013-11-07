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
 * @file SimpleBidirectionalRangeFromPoint.h
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2013/11/07
 *
 *
 * Header file for module SimpleBidirectionalRangeFromPoint.h
 *
 * This file contains the definition of basic functors.
 *
 * This file is part of the DGtal library.
 */

#if defined(SimpleBidirectionalRangeFromPoint_RECURSES)
#error Recursive header files inclusion detected in SimpleBidirectionalRangeFromPoint.h
#else // defined(SimpleBidirectionalRangeFromPoint_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SimpleBidirectionalRangeFromPoint_RECURSES

#if !defined SimpleBidirectionalRangeFromPoint_h
/** Prevents repeated inclusion of headers. */
#define SimpleBidirectionalRangeFromPoint_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/base/Circulator.h"
#include "DGtal/base/IteratorAdapter.h"
#include "DGtal/base/CBidirectionalRangeFromPoint.h"
#include "boost/concept_check.hpp"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{


  ///////////////////////////////////////////////////////////////////////////////
  // class SimpleBidirectionalRangeFromPoint
  ///////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Aim: model of CBidirectionalRangeFromPoint that adapts any range of elements
   * bounded by two iterators [itb, ite) and provides services to
   * (circularly)iterate over it (in a read-only manner).
   *
   * @tparam TIterator the type of the iterator to adapt (at least bidirectional)
   *
   * @see RangeAdapter
   */
  template <typename TConstIterator, typename TIterator, typename TPoint>

  class SimpleBidirectionalRangeFromPoint
  {

      BOOST_CONCEPT_ASSERT ( ( boost::BidirectionalIterator<TIterator> ) );
      BOOST_CONCEPT_ASSERT ( ( boost::BidirectionalIterator<TConstIterator> ) );

      // ------------------------- inner types --------------------------------

    public:

      typedef TPoint Point;

      typedef TIterator Iterator;
      typedef TConstIterator ConstIterator;

      typedef std::reverse_iterator<Iterator> ReverseIterator;
      typedef std::reverse_iterator<ConstIterator> ConstReverseIterator;

      typedef TIterator OutputIterator;
      typedef std::reverse_iterator<Iterator> ReverseOutputIterator;

      // typedef Circulator<Iterator> Circulator;
      // typedef std::reverse_iterator<Circulator> ReverseCirculator;

      // ------------------------- standard services --------------------------------

      /**
       * Standard constructor from two iterators
       * and one functor.
       * @param itb begin iterator.
       * @param ite end iterator.
       * @param aDistance functor used to adapt on-the-fly the elements of the range
       *
       */
      SimpleBidirectionalRangeFromPoint ( const TIterator& itb,
                                         const TIterator& ite )
          : myBegin ( itb ), myEnd ( ite ) {}

      /**
       * Copy constructor.
       * @param other the iterator to clone.
       */
      SimpleBidirectionalRangeFromPoint ( const SimpleBidirectionalRangeFromPoint & other )
          : myBegin ( other.myBegin ), myEnd ( other.myEnd ) {}

      /**
       * Assignment.
       * @param other the iterator to copy.
       * @return a reference on 'this'.
       */
      SimpleBidirectionalRangeFromPoint& operator= ( const SimpleBidirectionalRangeFromPoint & other )
      {
        if ( this != &other )
        {
          myBegin = other.myBegin;
          myEnd = other.myEnd;
        }

        return *this;
      }

      /**
       * Destructor. Does nothing.
       */
      ~SimpleBidirectionalRangeFromPoint() {}

      /**
       * Checks the validity/consistency of the object.
       * @return 'true' if the object is valid, 'false' otherwise.
       */
      bool isValid() const
      {
        return true;
      }

      // ------------------------- display --------------------------------
      /**
       * Writes/Displays the object on an output stream.
       * @param out the output stream where the object is written.
       */
      void selfDisplay ( std::ostream & out ) const
      {
        typedef typename IteratorCirculatorTraits<Iterator>::Value Value;
        out << "[SimpleBidirectionalRangeFromPoint]" << std::endl;
        out << "\t";
        std::copy ( myBegin, myEnd, std::ostream_iterator<Value> ( out, ", " ) );
        out << std::endl;
      }

      /**
       * @return the style name used for drawing this object.
       */
      std::string className() const
      {
        return "SimpleBidirectionalRangeFromPoint";
      }


      // ------------------------- private data --------------------------------

    private:
      /**
       * Begin underlying iterator
       */
      TIterator myBegin;
      /**
       * End underlying iterator
       */
      TIterator myEnd;

      // ------------------------- iterator services --------------------------------

    public:

      /**
       * Iterator service.
       * @return begin iterator
       */
      Iterator begin()
      {
        return Iterator ( myBegin );
      }


      /**
       * Iterator service.
       * @param aPoint a Point
       * @return begin iterator at aPoint
       */
      Iterator begin ( const Point &aPoint )
      {
        // TODO
      }

       /**
       * Iterator service.
       * @return begin iterator
       */
      ConstIterator begin() const
      {
        return ConstIterator ( myBegin );
      }


      /**
       * Iterator service.
       * @param aPoint a Point
       * @return begin iterator at aPoint
       */
      ConstIterator begin ( const Point &aPoint ) const
      {
        // TODO
      }

      /**
       * Iterator service.
       * @return end iterator
       */
      Iterator end()
      {
        return Iterator ( myEnd );
      }

     /**
        * Iterator service.
       * @return end iterator
       */
      ConstIterator end()  const
      {
        return ConstIterator ( myEnd );
      }

      /**
       * OutputIterator service.
       * @return an output itertor on the first elements
       */
      OutputIterator outputIterator()
      {
        return OutputIterator ( myBegin );
      }

      /**
       * OutputIterator service.
       * @param aPoint a point
       * @return an output itertor on the point
       */
      OutputIterator outputIterator ( const Point &aPoint )
      {
        // TODO
      }

      /**
      * ReverseOutputIterator service.
      * @return an output itertor on the first elements
      */
      ReverseOutputIterator routputIterator()
      {
        return ReverseOutputIterator ( myBegin );
      }

      /**
       * ReverseOutputIterator service.
       * @param aPoint a point
       * @return an output itertor on the point
       */
      ReverseOutputIterator routputIterator ( const Point &aPoint )
      {
        // TODO
      }



      /**
       * Iterator service.
       * @return rbegin iterator
       */
      ReverseIterator rbegin()
      {
        return ReverseIterator ( this->end() );
      }

      /**
       * Iterator service.
      * @param aPoint a Point
       * @return rbegin iterator at aPoint
       */
      ReverseIterator rbegin ( const Point &aPoint )
      {
        // TODO
      }


      /**
       * Iterator service.
       * @return rend iterator
       */
      ReverseIterator rend()
      {
        return ReverseIterator ( this->begin() );
      }


      /**
       * Iterator service.
       * @return rbegin iterator
       */
      ConstReverseIterator rbegin() const
      {
        return ConstReverseIterator ( this->end() );
      }

      /**
       * Iterator service.
      * @param aPoint a Point
       * @return rbegin iterator at aPoint
       */
      ConstReverseIterator rbegin ( const Point &aPoint ) const
      {
        // TODO
      }


      /**
       * Iterator service.
       * @return rend iterator
       */
      ConstReverseIterator rend() const
      {
        return ConstReverseIterator ( this->begin() );
      }

      // /**
      //  * Circulator service.
      //  * @return a circulator
      //  */
      // Circulator c() const
      // {
      //   return Circulator ( this->begin(), this->begin(), this->end() );
      // }

      // /**
      //  * Circulator service.
      //  * @return a reverse circulator
      //  */
      // ReverseCirculator rc() const
      // {
      //   return ReverseCirculator ( this->c() );
      // }

  }; //end class SimpleBidirectionalRangeFromPoint

} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////


#endif // !defined SimpleBidirectionalRangeFromPoint_h

#undef SimpleBidirectionalRangeFromPoint_RECURSES
#endif // else defined(SimpleBidirectionalRangeFromPoint_RECURSES)
