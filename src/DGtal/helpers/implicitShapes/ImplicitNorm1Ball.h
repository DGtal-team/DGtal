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
 * @file ImplicitNorm1Ball.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/03/22
 *
 * Header file for module ImplicitNorm1Ball.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImplicitNorm1Ball_RECURSES)
#error Recursive header files inclusion detected in ImplicitNorm1Ball.h
#else // defined(ImplicitNorm1Ball_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImplicitNorm1Ball_RECURSES

#if !defined ImplicitNorm1Ball_h
/** Prevents repeated inclusion of headers. */
#define ImplicitNorm1Ball_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  template <typename TSpace>
  class ImplicitNorm1Ball
  {

  public:
    typedef TSpace Space;
    typedef typename Space::Point Point;
    typedef typename Space::Integer Integer;
    
   
    /** 
     * Constructor. Contructs a ball with center aCenter and width
     * aWidth.
     * 
     * @param aCenter the cube center. 
     * @param aHalfWidth the cube half-width.
     */
    ImplicitNorm1Ball(const Point &aCenter, const Integer &aHalfWidth): myCenter(aCenter),
								myHalfWidth(aHalfWidth)
    {};
    
    /** 
     * Destructor.
     * 
     */    
    ~ImplicitNorm1Ball();


    // ----------------------- Interface --------------------------------------
  public:
    
    /** 
     * Operator() of the implicit function. Given a point, it returns
     * the function value at p. In Shapes, positive values are used to
     * construct a set.
     * 
     * @param aPoint the point to evalute the function at.
     * @return the distance of aPoint to the ball center.
     */
    inline
    double operator()(const Point &aPoint) const
    {
      return IntegerTraits<Integer>::castToDouble(myHalfWidth) - 
	(aPoint - myCenter ).norm(Point::L_1);
    }

    /** 
     * Returns the lower bound of the Shape bounding box.
     * 
     * 
     * @return the lower bound point.
     */
    inline
    Point getLowerBound() const
    {
      return (myCenter - Point::diagonal(myHalfWidth));
    }
    
    /** 
     * Returns the upper bound of the Shape bounding box.
     * 
     * 
     * @return the upper bound point.
     */
    inline
    Point getUpperBound() const
    {
      return (myCenter + Point::diagonal(myHalfWidth)); 
    }
    
    // ----------------------- Interface --------------------------------------
  public:
    
    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:
   
    ///Ball center
    Point myCenter;

    ///Ball HalfWidth
    Integer myHalfWidth;
   
    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    ImplicitNorm1Ball();

  private:

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    ImplicitNorm1Ball & operator= ( const ImplicitNorm1Ball & other );
    
    
  }; // end of class ImplicitNorm1Ball


  /**
   * Overloads 'operator<<' for displaying objects of class 'ImplicitNorm1Ball'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ImplicitNorm1Ball' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const ImplicitNorm1Ball<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/helpers/implicitShapes/ImplicitNorm1Ball.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImplicitNorm1Ball_h

#undef ImplicitNorm1Ball_RECURSES
#endif // else defined(ImplicitNorm1Ball_RECURSES)
