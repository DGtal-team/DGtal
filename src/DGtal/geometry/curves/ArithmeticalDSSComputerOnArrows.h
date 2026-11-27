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
 * @file ArithmeticalDSSComputerOnArrows.h
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 * @date 2016/03/25
 *
 * @brief Header file for module ArithmeticalDSSComputerOnArrows.ih
 *
 * This file is part of the DGtal library.
 *
 * @see testArithmeticalDSSComputerOnArrows.cpp
 */

#if defined(ArithmeticalDSSComputerOnArrows_RECURSES)
#error Recursive header files inclusion detected in ArithmeticalDSSComputerOnArrows.h
#else // defined(ArithmeticalDSSComputerOnArrows_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ArithmeticalDSSComputerOnArrows_RECURSES

#if !defined ArithmeticalDSSComputerOnArrows_h
/** Prevents repeated inclusion of headers. */
#define ArithmeticalDSSComputerOnArrows_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>
#include "DGtal/base/Exceptions.h"
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/base/ReverseIterator.h"
#include "DGtal/base/IteratorCirculatorTraits.h"
#include "DGtal/geometry/curves/ArithmeticalDSS.h"
#include "DGtal/geometry/curves/SegmentComputerHelpers.h"
//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // class ArithmeticalDSSComputerOnArrows
  /**
   * \brief Aim: This class is a wrapper around ArithmeticalDSS that is devoted
   * to the dynamic recognition of standard digital straight segments (DSS)
   * along any sequence of arrows, i.e. pairs point-vector, such as the
   * arrows of a GridCurve (see GridCurve::getArrowsRange()), each arrow
   * coding a unit step of a 4-connected digital curve.
   *
   * A segment of n arrows corresponds to a DSS of n+1 points: the starting
   * points of the arrows and the ending point of the last one (for a
   * direct orientation). Contrary to ArithmeticalDSSComputer, whose
   * elements are the points of a curve, the elements of this segment
   * computer are thus its steps, like those of StabbingLineComputer.
   *
   * @tparam TIterator type of iterator on arrows,
   * at least readable and forward.
   * @tparam TInteger type of integers used for the computation of remainders,
   * which is a model of concepts::CInteger.
   * @tparam isDirect boolean set to 'true' for a direct orientation (the starting point of
   * an arrow is visited before its ending point), 'false' otherwise.
   * @tparam TCategory category of segment computer, i.e. ForwardSegmentComputer,
   * BidirectionalSegmentComputer, DynamicSegmentComputer or
   * DynamicBidirectionalSegmentComputer (default), used by the functions
   * of SegmentComputerHelpers.h (and thus by the segmentation algorithms)
   * to select the algorithm.
   *
   * This class is a model of concepts::CDynamicBidirectionalSegmentComputer.
   * It is also default constructible, copy constructible, assignable and equality comparable.
   *
   * @see ArithmeticalDSSComputer
   */
  template <typename TIterator,
	    typename TInteger = typename IteratorCirculatorTraits<TIterator>::Value::first_type::Coordinate,
	    bool isDirect = true,
	    typename TCategory = DynamicBidirectionalSegmentComputer>
  class ArithmeticalDSSComputerOnArrows
  {
    static_assert( isSegmentComputerCategoryCompatible<TCategory, DynamicBidirectionalSegmentComputer>,
                   "TCategory must be a segment computer category" );

    // ----------------------- inner types ------------------------------
  public:

    /**
     * Type of iterator, at least readable and forward
     */
    typedef TIterator ConstIterator;
    BOOST_CONCEPT_ASSERT(( boost_concepts::ReadableIteratorConcept<ConstIterator> ));
    BOOST_CONCEPT_ASSERT(( boost_concepts::ForwardTraversalConcept<ConstIterator> ));

    /**
     * Type of 2d digital point
     */
    typedef typename IteratorCirculatorTraits<ConstIterator>::Value::first_type Point;
    BOOST_STATIC_ASSERT(( Point::dimension == 2 ));
    /**
     * Type of 2d digital vector
     */
    typedef typename IteratorCirculatorTraits<ConstIterator>::Value::second_type Vector;
    BOOST_STATIC_ASSERT(( Vector::dimension == 2 ));

    /**
     * Type of coordinate
     */
    typedef typename Point::Coordinate Coordinate;
    BOOST_CONCEPT_ASSERT(( concepts::CInteger<Coordinate> ));

    /**
     * Type of integer, devoted to remainders (and intercepts)
     */
    typedef TInteger Integer;
    BOOST_CONCEPT_ASSERT(( concepts::CInteger<Integer> ));

    /**
     * Type of objects that represents DSSs
     */
    typedef ArithmeticalDSS<Coordinate, Integer, 4> DSS;

    /**
     * Type of primitive representation, defined as an alias of DSS
     */
    typedef DSS Primitive;


    typedef ArithmeticalDSSComputerOnArrows<ConstIterator,Integer,isDirect,TCategory> Self;
    typedef ArithmeticalDSSComputerOnArrows<ReverseIterator<ConstIterator>,Integer,!isDirect,TCategory> Reverse;

    /**
     * Category of segment computer, see SegmentComputerTraits
     */
    typedef TCategory SegmentComputerCategory;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Default constructor.
     */
    ArithmeticalDSSComputerOnArrows();

    /**
     * Initialisation.
     * @param it an iterator on arrows
     */
    void init(const ConstIterator& it);

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    ArithmeticalDSSComputerOnArrows ( const ArithmeticalDSSComputerOnArrows & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    ArithmeticalDSSComputerOnArrows& operator= ( const ArithmeticalDSSComputerOnArrows & other );

    /**
     * @return a default-constructed instance of Self
     */
    Self getSelf() const;

    /**
     * @return a default-constructed instance of Reverse
     */
    Reverse getReverse() const;

    /**
     * Equality operator.
     * @param other the object to compare with.
     * @return 'true' if the DSS representations
     * and the ranges of the two objects match,
     * 'false' otherwise
     */
    bool operator==( const ArithmeticalDSSComputerOnArrows & other ) const;

    /**
     * Difference operator.
     * @param other the object to compare with.
     * @return 'false' if equal
     * 'true' otherwise
     */
    bool operator!=( const ArithmeticalDSSComputerOnArrows & other ) const;

    /**
     * Destructor.
     */
    ~ArithmeticalDSSComputerOnArrows(){};

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Tests whether the current DSS can be extended at the front.
     *
     * @return 'true' if yes, 'false' otherwise.
     */
    bool isExtendableFront();

    /**
     * Tests whether the current DSS can be extended at the back.
     *
     * @return 'true' if yes, 'false' otherwise.
     */
    bool isExtendableBack();

    /**
     * Tests whether the current DSS can be extended at the front.
     * Computes the parameters of the extended DSS if yes.
     * @return 'true' if yes, 'false' otherwise.
     */
    bool extendFront();

    /**
     * Tests whether the current DSS can be extended at the back.
     * Computes the parameters of the extended DSS if yes.
     * @return 'true' if yes, 'false' otherwise.
     */
    bool extendBack();

    /**
     * Removes the front arrow of the segment
     * if it has more than one arrow (i.e. more than two points)
     * @return 'true' if the front arrow is removed, 'false' otherwise.
     */
    bool retractFront();

    /**
     * Removes the back arrow of the segment
     * if it has more than one arrow (i.e. more than two points)
     * @return 'true' if the back arrow is removed, 'false' otherwise.
     */
    bool retractBack();


    // ------------------------- Accessors ------------------------------

    /**
     * @return begin iterator of the range.
     */
    ConstIterator begin() const;
    /**
     * @return end iterator of the range.
     */
    ConstIterator end() const;

    /**
     * @return the current DSS representation.
     * NB: since we return a const reference, you must
     * copy the result, if you want to keep it beyond
     * the object's existence.
     */
    const Primitive& primitive() const;
    /**
     * @return a-parameter of the DSS
     */
    Integer a() const;
    /**
     * @return b-parameter of the DSS
     */
    Integer b() const;
    /**
     * @return mu-parameter of the DSS
     */
    Integer mu() const;
    /**
     * @return omega-parameter of the DSS
     */
    Integer omega() const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;


    // ------------------------- Protected Data ------------------------------
  protected:

    /**
    * DSS representation
    */
    DSS myDSS;
    /**
    * begin iterator
    */
    ConstIterator myBegin;
    /**
    * end iterator
    */
    ConstIterator myEnd;

    // ------------------ Display ------------------------------------------

  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;


  }; // end of class ArithmeticalDSSComputerOnArrows


/**
 * Overloads 'operator<<' for displaying objects of class 'ArithmeticalDSSComputerOnArrows'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'ArithmeticalDSSComputerOnArrows' to write.
 * @return the output stream after the writing.
 */
  template <typename TIterator, typename TInteger, bool isDirect, typename TCategory>
std::ostream&
operator<< ( std::ostream & out,  const ArithmeticalDSSComputerOnArrows<TIterator,TInteger,isDirect,TCategory> & object )
{
  object.selfDisplay( out);
  return out;
}


  namespace detail {

    /**
     * Gives the first and second points of an arrow, according
     * to its orientation.
     * @tparam isDirect 'true' if the starting point of an arrow is
     * visited before its ending point, 'false' otherwise.
     */
    template <bool isDirect = true>
    struct ArrowHelper {

      template <typename Arrow>
      static typename Arrow::first_type
      firstPoint(const Arrow& anArrow) {
	return anArrow.first;
      }

      template <typename Arrow>
      static typename Arrow::first_type
      secondPoint(const Arrow& anArrow) {
	return anArrow.first+anArrow.second;
      }
    };

    //specialization for indirect orientation
    template <>
    struct ArrowHelper<false> {

      template <typename Arrow>
      static typename Arrow::first_type
      firstPoint(const Arrow& anArrow) {
	return anArrow.first+anArrow.second;
      }

      template <typename Arrow>
      static typename Arrow::first_type
      secondPoint(const Arrow& anArrow) {
	return anArrow.first;
      }
    };

  } //namespace detail

} // namespace DGtal



///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods.
#include "DGtal/geometry/curves/ArithmeticalDSSComputerOnArrows.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ArithmeticalDSSComputerOnArrows_h

#undef ArithmeticalDSSComputerOnArrows_RECURSES
#endif // else defined(ArithmeticalDSSComputerOnArrows_RECURSES)
