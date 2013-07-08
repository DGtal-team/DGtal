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
 * @file ArithmeticalDSSKernel.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/07/02
 *
 * Header file for module ArithmeticalDSSKernel.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ArithmeticalDSSKernel_RECURSES)
#error Recursive header files inclusion detected in ArithmeticalDSSKernel.h
#else // defined(ArithmeticalDSSKernel_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ArithmeticalDSSKernel_RECURSES

#if !defined ArithmeticalDSSKernel_h
/** Prevents repeated inclusion of headers. */
#define ArithmeticalDSSKernel_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/kernel/PointVector.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template struct ArithmeticalDSSKernel
  /**
   * Description of template struct 'ArithmeticalDSSKernel' <p>
   * \brief Aim: Small tool that provide static methods
   * returning the shift vector (ie. vector translating a point of
   * remainder to a point of remainder r+omega) and the step vectors
   * (ie. vectors used to iterate over the points of a DSS).
   * These methods are specialized with respect to the chosen adjacency 
   * 8 (default) or 4. 
   *
   * @tparam TCoordinate a model of integer for the DGtal point coordinate
   * @tparam adajency integer equal to 8 (default) for 8-connected DSS, 
   * and 4 for 4-connected DSS. 
   */
  template <typename TCoordinate, 
	    unsigned short adjacency = 8>
  struct ArithmeticalDSSKernel
  {

    // ----------------------- Inner types ------------------------------------
  public:
    typedef DGtal::PointVector<2,TCoordinate> Vector; 
    typedef std::pair<Vector, Vector> Steps; 

    // ----------------------- Interface --------------------------------------
  public:
    /**
     * Given parameters @a a and @a b, this method computes the shift vector
     * translating a point of remainder r to a point of remainder r+omega.
     *
     * @param a a-parameter
     * @param b b-parameter
     * @tparam TInteger a model of integer for the parameters
     * @return shift vector
     *
     * NB: The shift vector is set to (0,0) if @a a and @a b are both null. 
     * If ( @a a , @a b ) lies between two octant (resp. quadrant) (eg. b>0
     * and a=0), the shift vector of the next octant (resp. quadrant) is 
     * chosen with respect to the counter-clockwise orientation.
     *
     * @see steps
     */
    template <typename TInteger>
    static Vector shift(const TInteger& a, const TInteger& b); 

    /**
     * Given parameters @a a and @a b, this method returns
     * the two vectors that are used to iterate over the points 
     * of a DSS of slope a/b.
     *
     * The first vector translates any point to a point of greater remainder, 
     * whereas the second one translates any point to a point of smaller remainder. 
     * Moreover, the difference between the first and the second vectors is equal 
     * to the shift vector. 
     * @see shift
     *
     * @param a a-parameter
     * @param b b-parameter
     * @tparam TInteger a model of integer for the parameters
     * @return the pair of steps
     *
     * NB: The two vectors are set to (0,0) if @a a and @a b are both null.
     * The second vector is set to (0,0) if either @a a or @a b is null. 
     */
    template <typename TInteger>
    static Steps steps(const TInteger& a, const TInteger& b); 

    /**
     * Returns the Linf (for the 8-adjacency) or L1 norm (for the 4-adjacency) 
     * of two integers  @a a and @a b, 
     *
     * @param a a-parameter
     * @param b b-parameter
     * @tparam TInteger a model of integer for the parameters
     * @return the norm
     */
    template<typename TInteger>
    static TInteger norm(const TInteger& a, const TInteger& b);
  }; 

  /////////////////////////////////////////////////////////////////////////////
  // specialization of the template struct ArithmeticalDSSKernel for 4-adjacency
  template <typename TCoordinate> 
  struct ArithmeticalDSSKernel<TCoordinate, 4>
  {
    // ----------------------- Inner types ------------------------------------
  public:
    typedef DGtal::PointVector<2,TCoordinate> Vector; 
    typedef std::pair<Vector, Vector> Steps; 

    // ----------------------- Interface --------------------------------------
      
  public: 
    template <typename TInteger>
    static Vector shift(const TInteger& a, const TInteger& b);
    template <typename TInteger>
    static Steps steps(const TInteger& a, const TInteger& b);
    template<typename TInteger>
    static TInteger norm(const TInteger& a, const TInteger& b);
  }; 

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/curves/ArithmeticalDSSKernel.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ArithmeticalDSSKernel_h

#undef ArithmeticalDSSKernel_RECURSES
#endif // else defined(ArithmeticalDSSKernel_RECURSES)
