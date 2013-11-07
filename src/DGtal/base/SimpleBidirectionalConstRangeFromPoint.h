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
 * @file SimpleBidirectionalConstRangeFromPoint.h
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2013/11/06
 *
 *
 * Header file for module SimpleBidirectionalConstRangeFromPoint.h
 *
 * This file contains the definition of basic functors.
 *
 * This file is part of the DGtal library.
 */

#if defined(SimpleBidirectionalConstRangeFromPoint_RECURSES)
#error Recursive header files inclusion detected in SimpleBidirectionalConstRangeFromPoint.h
#else // defined(SimpleBidirectionalConstRangeFromPoint_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SimpleBidirectionalConstRangeFromPoint_RECURSES

#if !defined SimpleBidirectionalConstRangeFromPoint_h
/** Prevents repeated inclusion of headers. */
#define SimpleBidirectionalConstRangeFromPoint_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/base/Circulator.h"
#include "DGtal/base/ConstIteratorAdapter.h"
#include "DGtal/base/CConstBidirectionalRangeFromPoint.h"
#include "boost/concept_check.hpp"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{


  ///////////////////////////////////////////////////////////////////////////////
  // class SimpleBidirectionalConstRangeFromPoint
  ///////////////////////////////////////////////////////////////////////////////

  /**
   * @brief Aim: model of CConstBidirectionalRangeFromPoint that adapts any range of elements
   * bounded by two iterators [itb, ite) and provides services to
   * (circularly)iterate over it (in a read-only manner).
   *
   * @tparam TConstIterator the type of the iterator to adapt (at least bidirectional)
   *
   * @see ConstRangeAdapter
   */
  template <typename TConstIterator, typename TPoint>

  class SimpleBidirectionalConstRangeFromPoint
  {

      BOOST_CONCEPT_ASSERT ( ( boost::BidirectionalIterator<TConstIterator> ) );

      // ------------------------- inner types --------------------------------

    public:

      typedef TPoint Point;

      typedef TConstIterator ConstIterator;
      typedef std::reverse_iterator<ConstIterator> ConstReverseIterator;

      typedef Circulator<ConstIterator> ConstCirculator;
      typedef std::reverse_iterator<ConstCirculator> ConstReverseCirculator;

      // ------------------------- standard services --------------------------------

      /**
       * Standard constructor from two iterators
       * and one functor.
       * @param itb begin iterator.
       * @param ite end iterator.
       *
       */
      SimpleBidirectionalConstRangeFromPoint ( const TConstIterator& itb, const TConstIterator& ite )
        : myBegin ( itb ), myEnd ( ite ) {}

      /**
       * Copy constructor.
       * @param other the iterator to clone.
       */
      SimpleBidirectionalConstRangeFromPoint ( const SimpleBidirectionalConstRangeFromPoint & other )
          : myBegin ( other.myBegin ), myEnd ( other.myEnd ) {}

      /**
       * Assignment.
       * @param other the iterator to copy.
       * @return a reference on 'this'.
       */
      SimpleBidirectionalConstRangeFromPoint& operator= ( const SimpleBidirectionalConstRangeFromPoint & other )
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
      ~SimpleBidirectionalConstRangeFromPoint() {}

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
        typedef typename IteratorCirculatorTraits<ConstIterator>::Value Value;
        out << "[SimpleBidirectionalConstRangeFromPoint]" << std::endl;
        out << "\t";
        std::copy ( myBegin, myEnd, std::ostream_iterator<Value> ( out, ", " ) );
        out << std::endl;
      }

      /**
       * @return the style name used for drawing this object.
       */
      std::string className() const
      {
        return "SimpleBidirectionalConstRangeFromPoint";
      }


      // ------------------------- private data --------------------------------

    private:
      /**
       * Begin underlying iterator
       */
      TConstIterator myBegin;
      /**
       * End underlying iterator
       */
      TConstIterator myEnd;

    // ------------------------- iterator services --------------------------------

    public:

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
       * @return begin iterator
       */
      ConstIterator begin ( const Point &aPoint ) const
      {
        // TODO
        return ConstIterator ( myBegin );
      }

      /**
       * Iterator service.
       * @return end iterator
       */
      ConstIterator end() const
      {
        return ConstIterator ( myEnd );
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
       * @return rbegin iterator
       */
      ConstReverseIterator rbegin ( const Point &aPoint ) const
      {
        // TODO
        return ConstReverseIterator ( this->end() );
      }


      /**
       * Iterator service.
       * @return rend iterator
       */
      ConstReverseIterator rend() const
      {
        return ConstReverseIterator ( this->begin() );
      }

      /**
       * Circulator service.
       * @return a circulator
       */
      ConstCirculator c() const
      {
        return ConstCirculator ( this->begin(), this->begin(), this->end() );
      }

      /**
       * Circulator service.
       * @return a reverse circulator
       */
      ConstReverseCirculator rc() const
      {
        return ConstReverseCirculator ( this->c() );
      }

  }; //end class SimpleBidirectionalConstRangeFromPoint

} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////


#endif // !defined SimpleBidirectionalConstRangeFromPoint_h

#undef SimpleBidirectionalConstRangeFromPoint_RECURSES
#endif // else defined(SimpleBidirectionalConstRangeFromPoint_RECURSES)
