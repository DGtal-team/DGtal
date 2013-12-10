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
 * @file OrientationFunctor2BySimpleMatrix.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/11/22
 *
 * Header file for module OrientationFunctor2BySimpleMatrix.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(OrientationFunctor2BySimpleMatrix_RECURSES)
#error Recursive header files inclusion detected in OrientationFunctor2BySimpleMatrix.h
#else // defined(OrientationFunctor2BySimpleMatrix_RECURSES)
/** Prevents recursive inclusion of headers. */
#define OrientationFunctor2BySimpleMatrix_RECURSES

#if !defined OrientationFunctor2BySimpleMatrix_h
/** Prevents repeated inclusion of headers. */
#define OrientationFunctor2BySimpleMatrix_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/kernel/CEuclideanRing.h"
#include "DGtal/kernel/SimpleMatrix.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class OrientationFunctor2BySimpleMatrix
  /**
   * \brief Aim: Class that implements an orientation functor, ie. 
   * it provides a way to compute the orientation of three given 2d points. 
   * More precisely, it returns: 
   * - zero if the three points belong to the same line
   * - strictly positive if the three points are counter-clockwise oriented
   * - striclty negative if the three points are clockwise oriented. 
   * The orientation test is performed by the evaluation of the determinant 
   * of a 3x3 matrix stored into an instance of SimpleMatrix. 
   *
   * Basic usage: 
   @code
   ...
   typedef Z2i::Point Point; 
   typedef OrientationFunctor2BySimpleMatrix<Point, Z2i::Integer> MyType; 

   MyType orientationTest; 
   orientationTest.init( Point(0,0), Point(5,2) ); 
   return orientationTest( Point(2,1) ); 
   //a strictly positive value is returned because (0,0) (5,2) (2,1) are CCW oriented
   @endcode
   *
   * @tparam TPoint a model of point
   * @tparam TInteger a model of integer for the 3x3 matrix entries and the result, 
   * at least a model of CEuclideanRing
   *
   * NB. In order to be sure that the result will be exact, you should be sure that 
   * TInteger can represent integers with 2b+3 bits if the points coordinates
   * are coded with b bits. 
   *
   * @see SimpleMatrix
   * @see OrientationFunctor2By2x2DetComputer
   */
  template <typename TPoint, typename TInteger>
  class OrientationFunctor2BySimpleMatrix
  {
    // ----------------------- Types  ------------------------------------
  public:

    /**
     * Type of points 
     */
    typedef TPoint Point; 

    /**
     * Type of matrix integral entries
     *
     * NB: the type of the points coordinates are casted into Integer
     * before being stored into the matrix. 
     */
    typedef TInteger Integer;
    BOOST_CONCEPT_ASSERT(( CEuclideanRing<Integer> )); 

    /**
     * Type of matrix
     */
    typedef SimpleMatrix<Integer, 3, 3> Matrix;

    /**
     * Type of integer for the result
     */
    typedef Integer Value; 

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Initialisation. The first two column vectors are stored into @a myMatrix
     * @param aP first point
     * @param aQ second point
     */
    void init(const Point& aP, const Point& aQ);

    /**
     * Main operator.
     * @warning OrientationFunctor2By2x2DetComputer::init() should be called before
     * @param aR any point to test
     * @return orientation of the three points @a aP @a aQ @a aR : 
     * - zero if the three points belong to the same line
     * - strictly positive if the three points are counter-clockwise oriented
     * - striclty negative if the three points are clockwise oriented
     * @see OrientationFunctor2By2x2DetComputer::init()
     */
    Value operator()(const Point& aR) const;

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

    // ------------------------- Private Datas --------------------------------
  private:

    /**
     * 3x3 matrix whose determinant provides the expected result
     */
    mutable Matrix myMatrix; 


  }; // end of class OrientationFunctor2BySimpleMatrix


  /**
   * Overloads 'operator<<' for displaying objects of class 'OrientationFunctor2BySimpleMatrix'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'OrientationFunctor2BySimpleMatrix' to write.
   * @return the output stream after the writing.
   */
  template <typename TPoint, typename TInteger>
  std::ostream&
  operator<< ( std::ostream & out, const OrientationFunctor2BySimpleMatrix<TPoint, TInteger> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/tools/determinant/OrientationFunctor2BySimpleMatrix.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined OrientationFunctor2BySimpleMatrix_h

#undef OrientationFunctor2BySimpleMatrix_RECURSES
#endif // else defined(OrientationFunctor2BySimpleMatrix_RECURSES)
