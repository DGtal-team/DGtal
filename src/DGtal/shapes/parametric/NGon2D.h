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
 * @file NGon2D.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2011/04/12
 *
 * Header file for module NGon2D.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(NGon2D_RECURSES)
#error Recursive header files inclusion detected in NGon2D.h
#else // defined(NGon2D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define NGon2D_RECURSES

#if !defined NGon2D_h
/** Prevents repeated inclusion of headers. */
#define NGon2D_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/shapes/parametric/StarShaped2D.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class NGon2D
  /**
   * Description of template class 'NGon2D' <p>
   * \brief Aim: Model of the concept StarShaped
   * represents any regular k-gon in the plane.
   *
   * NB: A backport from ImaGene.
   */
  template <typename TSpace>
  class NGon2D final:  public StarShaped2D<TSpace>
  {
    // ----------------------- Standard services ------------------------------
  public:

    typedef TSpace Space;
    typedef typename Space::RealPoint RealPoint;
    typedef typename Space::RealVector RealVector;

    /**
     * Constructor.
     * Forbidden by default.
     */
    NGon2D() = delete;

    /**
     * Constructor.
     * @param x0 the x-coordinate of the circle center.
     * @param y0 the y-coordinate of the circle center.
     * @param r the radius of the circle.
     * @param k the number of faces of the k-gon.
     * @param phi the phase of the ngon (radian).
     */
    NGon2D( const double x0, const double y0, const double r ,
      const unsigned int k, const double phi );

    /**
     * Constructor.
     * @param aPoint the circle center.
     * @param r the radius of the circle.
     * @param k the number of faces of the k-gon.
     * @param phi the phase of the ngon (radian).
     */
    NGon2D( const RealPoint& aPoint, const double r,
     const   unsigned int k, const double phi );

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    NGon2D( const NGon2D& other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    NGon2D& operator= ( const NGon2D& other ) = delete;

    /**
     * Destructor.
     */
    ~NGon2D() = default;


  // ------------- Implementation of 'StarShaped' services ------------------
  public:

    /**
     * @return the lower bound of the shape bounding box.
     *
     */
    RealPoint getLowerBound() const
    {
      return myCenter - myRadius;
    }

    /**
     * @return the upper bound of the shape bounding box.
     *
     */
    RealPoint getUpperBound() const
    {
      return myCenter + myRadius;
    }

    /**
     * @return the center of the star-shaped object.
     */
    RealPoint center() const
    {
      return myCenter;
    }

    /**
     * Modify the shape center
     * @param newCenter the new center position
     */
    inline
    void moveTo( const RealPoint& newCenter )
    {
      myCenter = newCenter;
    }

    /**
     * @param p any point in the plane.
     *
     * @return the angle parameter between 0 and 2*Pi corresponding to
     * this point for the shape.
     */
    double parameter( const RealPoint & p ) const;


    /**
     * @param t any angle between 0 and 2*Pi.
     *
     * @return the vector (x(t),y(t)) which is the position on the
     * shape boundary.
     */
    RealPoint x( const double t ) const;

    /**
     * @param t any angle between 0 and 2*Pi.
     *
     * @return the vector (x'(t),y'(t)) which is the tangent to the
     * shape boundary.
     */
    RealVector xp( const double t ) const;

    /**
     * @param t any angle between 0 and 2*Pi.
     *
     * @return the vector (x''(t),y''(t)).
     */
    RealVector xpp( const double t ) const;


    // ------------------------- data ----------------------------
  private:

    /**
     * Center of the circle.
     */
    RealPoint myCenter;

    /**
     * Radius of the circle.
     */
    double myRadius;

    /**
     * Number of sides of the ngon.
     */
    unsigned int myK;

    /**
     * Phase of the  ngon (radian).
     */
    double myPhi;



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

  }; // end of class NGon2D


  /**
   * Overloads 'operator<<' for displaying objects of class 'NGon2D'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'NGon2D' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const NGon2D<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/shapes/parametric/NGon2D.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined NGon2D_h

#undef NGon2D_RECURSES
#endif // else defined(NGon2D_RECURSES)
