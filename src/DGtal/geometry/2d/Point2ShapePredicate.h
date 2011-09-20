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
 * @file Point2ShapePredicate.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/10/27
 *
 * @brief Header file for module Point2ShapePredicate.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Point2ShapePredicate_RECURSES)
#error Recursive header files inclusion detected in Point2ShapePredicate.h
#else // defined(Point2ShapePredicate_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Point2ShapePredicate_RECURSES

#if !defined Point2ShapePredicate_h
/** Prevents repeated inclusion of headers. */
#define Point2ShapePredicate_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <functional>

#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{


  /////////////////////////////////////////////////////////////////////////////
  // template class Point2ShapePredicate
  /**
   * \brief Aim: Predicate returning 'true' iff a given point is in 
   * the 'interior' of a given shape, 'false' otherwise
   *
   * This class is a model of CPointPredicate.
   *
   * @tparam TSurface  a type that is a model of COrientableHypersurface.
   * Must separate the space in two disjoints parts (the 'interior' and the 'exterior')
   * and must be able to return for any given point (of type TSurface::Point) 
   * the signed distance to itself (of type TSurface::Distance) by a method called signedDistance() 
   * @tparam isUpward  a bool for the orientation ('true' means that the interior 
   * is the set of points of positive distance)
   * @tparam isClosed  a bool for the closure ('true' means that the surface is included)
   *  
   * @see testHalfPlane.cpp
   */
  template <typename TSurface, bool isUpward, bool isClosed>
  class Point2ShapePredicate
  {
    // ----------------------- Standard services ------------------------------
  public:

    typedef typename TSurface::Point Point;
    typedef typename TSurface::Distance Distance; 
  
  /**
     * Constructor.
     * @param aSurface any Surface
     */
    Point2ShapePredicate(const Surface& aSurface);

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    Point2ShapePredicate ( const Point2ShapePredicate & other );

   /**
     * @param p any point.
     * @return true iff @a p is in the interior of the shape.
     */
    bool operator()( const Point & p ) const;

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;


    /**
     * Destructor. Does nothing
     */
    ~Point2ShapePredicate();


    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:
    /**
     * The surface with respect to the points have to be located
     */
    TSurface myS;
    // ------------------------- Hidden services ------------------------------
  protected:


  private:

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class Point2ShapePredicate

//////////////////////////////////////////////////////////////////////////////
// policy classes for Point2ShapePredicate

    template <typename T, bool b1, bool b2>
    struct Point2ShapePredicateComparator {
      bool operator()(const T& q, 
                       const T& t) const {
        std::less<T> c;
        return c(q,t);
      }
    };

    template <typename T>
    struct Point2ShapePredicateComparator<T,false,false> {
      bool operator()(const T& q, 
                       const T& t) const {
        std::less<T> c;
        return c(q,t);
      }
    };

    template <typename T>
    struct Point2ShapePredicateComparator<T,false,true> {
      bool operator()(const T& q, 
                       const T& t) const {
        std::less_equal<T> c;
        return c(q,t);
      }
    };

    template <typename T>
    struct Point2ShapePredicateComparator<T,true,false> {
      bool operator()(const T& q, 
                       const T& t) const {
        std::greater<T> c;
        return c(q,t);
      }
    };

    template <typename T>
    struct Point2ShapePredicateComparator<T,true,true> {
      bool operator()(const T& q, 
                       const T& t) const {
        std::greater_equal<T> c;
        return c(q,t);
      }
    };

/**
 * Overloads 'operator<<' for displaying objects of class 'Point2ShapePredicate'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'Point2ShapePredicate' to write.
 * @return the output stream after the writing.
 */
template <typename TSurface, bool isUpward, bool isClosed>
inline
std::ostream&
operator<< ( std::ostream & out, 
      const Point2ShapePredicate<TSurface,isUpward,isClosed> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/2d/Point2ShapePredicate.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Point2ShapePredicate_h

#undef Point2ShapePredicate_RECURSES
#endif // else defined(Point2ShapePredicate_RECURSES)
