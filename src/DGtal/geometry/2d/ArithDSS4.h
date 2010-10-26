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
//LICENSE-END
#pragma once

/**
 * @file ArithDSS4.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/02
 *
 * Header file for module ArithDSS4.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ArithDSS4_RECURSES)
#error Recursive header files inclusion detected in ArithDSS4.h
#else // defined(ArithDSS4_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ArithDSS4_RECURSES

#if !defined ArithDSS4_h
/** Prevents repeated inclusion of headers. */
#define ArithDSS4_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/2d/ArithDSS.h"
#include <cmath>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ArithDSS4
  /**
   * Description of template class 'ArithDSS4' <p>
   * \brief Aim:
   */
  template <typename Domain2D>
  class ArithDSS4: public ArithDSS<Domain2D>
  {
    // ----------------------- Standard services ------------------------------
  public:

		typedef typename Domain2D::Coordinate Integer;
		//2D point of a domain
		typedef typename Domain2D::Point Point;
		//2D vector of a domain
		typedef typename Domain2D::Vector Vector;

    /**
     * Constructor.
     */
    ArithDSS4(const Point& aFirstPoint, const Point& aSecondPoint);

    /**
     * Destructor.
     */
    ~ArithDSS4();

    // ----------------------- Interface --------------------------------------
  public:


    // ------------------------- Protected Datas ------------------------------
  protected:
    /**
		 * Computes the norm of the two components
     * of a 2D vector as the L1 norm
		 * @param x and y, two values. 
     * @return the L1 norm of a 2D vector.
     */
    Integer norm(const Integer  & x, const Integer & y) const;

    /**
		 * Checks whether the DSS lies in one quadrant or not
		 * @param aVec, the last displacement vector. 
     * @return 'true' if yes, 'false' otherwise.
     */
    bool sameQuadrant(const Vector& aVec) const;

    /**
		 * Computes the 2D vector 
		 * starting at a point of remainder 0
		 * and pointing at a point of remainder omega = |a|+|b|
     * @return the 2D vector.
     */
    Vector vectorFrom0ToOmega() const;

    /**
		 * Returns the point 
		 * that follows a given point in the DSS
		 * @param aPoint, a given point of the DSS. 
     * @return the next point.
     */
    Point next(const Point& aPoint) const;

    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:


  private:



    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class ArithDSS4


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/2d/ArithDSS4.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ArithDSS4_h

#undef ArithDSS4_RECURSES
#endif // else defined(ArithDSS4_RECURSES)
