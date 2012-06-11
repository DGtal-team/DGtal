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
 * @file Ellipsoid.h
 * @author Anis Benyoub (\c anis.benyoub@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/06/05
 *
 * Header file for module Ellipsoid.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Ellipsoid_RECURSES)
#error Recursive header files inclusion detected in Ellipsoid.h
#else // defined(Ellipsoid_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Ellipsoid_RECURSES

#if !defined Ellipsoid_h
/** Prevents repeated inclusion of headers. */
#define Ellipsoid_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
#include "DGtal/shapes/parametric/StarShaped3D.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class Ellipsoid
  /**
   * Description of template class 'Ellipsoid' <p>
   * \brief Aim: Model of the concept StarShaped
   * represents any ellipsoid in the space.
   *
   */
  template <typename TSpace>
  class Ellipsoid:  public StarShaped3D<TSpace>
  {
    // ----------------------- Standard services ------------------------------
  public:

    typedef TSpace Space;
    typedef typename Space::RealPoint RealPoint;
    typedef  pair<double,double> AngularCoordinates;
   
    /**
     * Destructor.
     */
    ~Ellipsoid();
    
    /**
     * Constructor. 
     * @param x0 the x-coordinate of the Ellipsoid center.
     * @param y0 the y-coordinate of the Ellipsoid center.
     * @param y0 the y-coordinate of the Ellipsoid center.
     * @param a the factor of x.
     * @param b the factor of y.
     * @param c the factor of c.
     */
    Ellipsoid( const double x0, const double y0, const double  z0, const double a, const double b, const double  c);

    /**
     * Constructor. 
     * @param aPoint the Ellipsoid center.
     * @param aFactors the Ellipsoid factors
     */
    Ellipsoid(const RealPoint &aPoint, const RealPoint &aFactors);

    
    /**
     * Constructor. 
     * @param aPoint the Ellipsoid center.
     * @param aFactors the Ellipsoid factors
     */
    /*
    Ellipsoid(const Point &aPoint, const RealPoint &aFactors);
*/
    
    
    
  // ------------- Implementation of 'StarShaped' services ------------------
  public:

    /**
     * @return the lower bound of the Ellipsoid.
     *
     */
    RealPoint getLowerBound() const
    {

      return RealPoint(myCenter[0] - myFactors[0], myCenter[1] - myFactors[1], myCenter[2] - myFactors[2] );
    }

    /**
     * @return the upper bound of the Ellipsoid.
     *
     */
    RealPoint getUpperBound() const
    {
      return RealPoint(myCenter[0] + myFactors[0], myCenter[1] + myFactors[1], myCenter[2] + myFactors[2]);
    }

    /**
     * @return the center of the Ellipsoid.
     */
    RealPoint center() const
    {
      return myCenter;
    }
   
    /**
     * @param p any point in the space.
     *
     * @return the angle parameters wich are respectivly between [-Pi/2,Pi/2) and [-Pi,Pi] corresponding to
     * this point for the shape.
     */
    AngularCoordinates parameter( const RealPoint & p ) const;


    
    /**
     * @param t is a couple of Teta && Phi wich are respectivly between [-Pi/2,Pi/2) and [-Pi,Pi]
     *
     * @return the vector (x(t),y(t),z(t)) which is the position on the
     * shape boundary.
     */
    RealPoint x( const AngularCoordinates t ) const;




     /**
     * @param t is a couple of Teta && Phi wich are respectivly between [-Pi/2,Pi/2) and [-Pi,Pi].
     *
     * @return the vector (gradf(M)).
     */
    virtual RealPoint gradient( const AngularCoordinates t) const ;

    
    
    /**
     * @param t is a couple of Teta && Phi wich are respectivly between [-Pi/2,Pi/2) and [-Pi,Pi].
     *
     * @return the vector (rt(M)) wich is the partial derivative with respect to Teta.
     */
    virtual RealPoint rt( const AngularCoordinates t) const ;



    /**
     * @param t is a couple of Teta && Phi wich are respectivly between [[-Pi/2,Pi/2) and [-Pi,Pi].
     *
     * @return the vector (rp(M)) wich is the partial derivative with respect to Phi.
     */
    virtual RealPoint rp( const AngularCoordinates t) const ;


    /**
     * @param t is a couple of Teta && Phi wich are respectivly between [-Pi/2,Pi/2) and [-Pi,Pi].
     *
     * @return the vector (rtt(M)) wich is second the partial derivative with respect to Teta(twice).
     */
    virtual RealPoint rtt( const AngularCoordinates t) const ;



    /**
     * @param t is a couple of Teta && Phi wich are respectivly between [-Pi/2,Pi/2) and [-Pi,Pi].
     *
     * @return the vector (rpp(M)) wich is second the partial derivative with respect to Phi(twice).
     */
    virtual RealPoint rpp( const AngularCoordinates t) const ;
    

     /**
     * @param t is a couple of Teta && Phi wich are respectivly between [-Pi/2,Pi/2) and [-Pi,Pi].
     *
     * @return the vector (rpp(M)) wich is second the partial derivative with respect to Teta then Phi.
     */
    virtual RealPoint rtp( const AngularCoordinates t) const ;
    
    
    
    
    // ------------------------- data ----------------------------
  private:

    /**
     * Center of the ellipsoid.
     */
    RealPoint myCenter;
    
    /**
     * factors of the ellipsoid.
     */
    RealPoint myFactors;

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


    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    Ellipsoid();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    //  Ellipsoid ( const Ellipsoid & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Ellipsoid & operator= ( const Ellipsoid & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class Ellipsoid


  /**
   * Overloads 'operator<<' for displaying objects of class 'Ellipsoid'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Ellipsoid' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const Ellipsoid<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "Ellipsoid.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Ellipsoid_h

#undef Ellipsoid_RECURSES
#endif // else defined(Ellipsoid_RECURSES)
