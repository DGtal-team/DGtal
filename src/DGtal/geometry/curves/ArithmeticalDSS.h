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
 * @file ArithmeticalDSS.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/06/28
 *
 * Header file for module ArithmeticalDSS.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ArithmeticalDSS_RECURSES)
#error Recursive header files inclusion detected in ArithmeticalDSS.h
#else // defined(ArithmeticalDSS_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ArithmeticalDSS_RECURSES

#if !defined ArithmeticalDSS_h
/** Prevents repeated inclusion of headers. */
#define ArithmeticalDSS_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/arithmetic/IntegerComputer.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ArithmeticalDSS
  /**
   * Description of template class 'ArithmeticalDSS' <p>
   * \brief Aim:
   * @tparam TCoordinate a model of integer for the DGtal point coordinate
   * @tparam TInteger a model of integer for the DSS parameters (a, b, mu, omega)
   * @tparam adajency a integer equal to 8 (default) for 8-connected DSS, 
   * and 4 for 4-connected DSS. 
   */
  template <typename TCoordinate, 
	    typename TInteger = TCoordinate, 
	    unsigned short adjacency = 8>
  class ArithmeticalDSS
  {
    
    // ----------------------- Inner types -----------------------------------
  public:

    typedef TCoordinate Coordinate; 
    typedef DGtal::PointVector<2, Coordinate> Point; 
    typedef DGtal::PointVector<2, Coordinate> Vector; 
    typedef TInteger Integer; 

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor.
     * @param aA y-component of the direction vector
     * @param aB x-component of the direction vector
     * @param aMu intercept
     * @param aOmega thickness
     * @param aF the first point
     * @param aL the last point
     * @param aUf the first upper point
     * @param aUl the last upper point
     * @param aLf the first lower point
     * @param aLl the last lower point
     */
    ArithmeticalDSS(const Integer& aA, const Integer& aB, 
		    const Integer& aMu, const Integer& aOmega, 
		    const Point& aF, const Point& aL,
		    const Point& aUf, const Point& aUl,
		    const Point& aLf, const Point& aLl);

    /**
     * Copy constructor.
     * @param aOther the object to clone.
     */
    ArithmeticalDSS ( const ArithmeticalDSS & aOther );

    /**
     * Assignment.
     * @param aOther the object to copy.
     * @return a reference on 'this'.
     */
    ArithmeticalDSS & operator= ( const ArithmeticalDSS & aOther );

    /**
     * Equality.
     * @param aOther the object to compare with.
     * @return 'true' the two set of points that are
     * implicitly defined are equal, 'false' otherwise
     */
    bool operator== ( const ArithmeticalDSS & aOther );

    /**
     * Difference.
     * @param aOther the object to compare with.
     * @return 'true' the two set of points that are
     * implicitly defined are different, 'false' otherwise
     */
    bool operator!= ( const ArithmeticalDSS & aOther );

    /**
     * Destructor.
     */
    ~ArithmeticalDSS();

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

    /**
     * @return a parameter (y-component of the direction vector)
     */
    Integer a() const; 

    /**
     * @return b parameter (x-component of the direction vector)
     */
    Integer b() const; 

    /**
     * @return mu parameter, the intercept
     */
    Integer mu() const; 

    /**
     * @return omega parameter, the thickness
     */
    Integer omega() const; 

    /**
     * @return first point of the DSS
     */
    Point back() const; 

    /**
     * @return last point of the DSS
     */
    Point front() const; 

    /**
     * @return first upper leaning point of the DSS
     */
    Point Uf() const; 

    /**
     * @return last upper leaning point of the DSS
     */
    Point Ul() const; 

    /**
     * @return first lower leaning point of the DSS
     */
    Point Lf() const; 

    /**
     * @return last lower leaning point of the DSS
     */
    Point Ll() const; 

    /**
     * Returns the remainder of @a aPoint
     * (which does not necessarily belong to the DSS)
     * @return remainder of @a aPoint
     * @param aPoint any point
     */
    Integer r(const Point& aPoint) const; 

    /**
     * Returns the position of @a aPoint
     * (which does not necessarily belong to the DSS)
     * computed along the direction that is orthogonal 
     * to the direction vector
     * @param aPoint the point whose position is returned 
     * @return @a myA * @a aPoint[0] + @a myB * @a aPoint[1].
     */
    Integer orthogonalPosition(const Point& aPoint) const; 

    /**
     * @return 'true' if @a aPoint is in the DSL
     * of minimal parameters containing (*this), 
     * 'false' otherwise. 
     * @param aPoint any point
     */
    bool isInDSL(const Point& aPoint) const; 

    /**
     * @return 'true' if @a aPoint is in the DSS
     * 'false' otherwise. 
     * @param aPoint any point
     */
    bool isInDSS(const Point& aPoint) const; 

    /**
     * @return 'true' if @a aPoint is in the DSS
     * 'false' otherwise. 
     * @param aPoint any point
     */
    bool operator()(const Point& aPoint) const; 

    // ------------------------- Protected Datas ------------------------------
  private:

    //------------------------ parameters of the DSS --------------------------
    /**
    * y-component of the direction vector
    */
    Integer myA;
    /**
    * x-component of the direction vector
    */
    Integer myB;
    /**
    * Intercept
    */
    Integer myMu;
    /**
    * Thickness
    */
    Integer myOmega;

    // -------------------- first and last point, leaning points ---------------
    /**
    * First upper leaning point ( of remainder @a myMu )
    */
    Point myUf;
    /**
    * Last upper leaning point ( of remainder @a myMu )
    */
    Point myUl;
    /**
    * First lower leaning point ( of remainder @a myMu + @a myOmega - 1 )
    */
    Point myLf;
    /**
    * Last lower leaning point ( of remainder @a myMu + @a myOmega - 1 )
    */
    Point myLl;
    /**
    * First point
    */
    Point myF;
    /**
    * Last point 
    */
    Point myL;

    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class ArithmeticalDSS


  /**
   * Overloads 'operator<<' for displaying objects of class 'ArithmeticalDSS'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ArithmeticalDSS' to write.
   * @return the output stream after the writing.
   */
  template <typename TCoordinate, typename TInteger, unsigned short adjacency>
  std::ostream&
  operator<< ( std::ostream & out, const ArithmeticalDSS<TCoordinate, TInteger, adjacency> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/curves/ArithmeticalDSS.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ArithmeticalDSS_h

#undef ArithmeticalDSS_RECURSES
#endif // else defined(ArithmeticalDSS_RECURSES)
