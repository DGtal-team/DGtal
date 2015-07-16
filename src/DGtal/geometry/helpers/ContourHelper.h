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
 * @file ContourHelper.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/05/02
 *
 * Header file for module ContourHelper.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ContourHelper_RECURSES)
#error Recursive header files inclusion detected in ContourHelper.h
#else // defined(ContourHelper_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ContourHelper_RECURSES

#if !defined ContourHelper_h
/** Prevents repeated inclusion of headers. */
#define ContourHelper_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class ContourHelper
/**
 * Description of class 'ContourHelper' <p>
 * \brief Aim: a helper class to process sequences of points.
 * @todo Improve class.
 */
class ContourHelper
{
    // ----------------------- Static services ------------------------------
public:
  
  /**
   * Compute the barycenter of the 2D contour.  
   * 
   * @param aContour the vector containing the 2D contour coordinates.
   * @return the resulting mean point. 
   **/
  template <typename TPoint>
  static 
  TPoint getMeanPoint(const std::vector<TPoint> & aContour);
  

  /**
   * Checks if a contour given as a sequence of point is clockwise oriented or not.
   *
   * @param aContour the vector containing the 2D contour coordinates.
   * @return 'true' if the contour is counter clockwise oriented, 'false' otherwise.
   **/

  template <typename TPoint> 
  static
  bool
  isCounterClockWise(const std::vector<TPoint> & aContour);

   
  

    // ----------------------- Standard services ------------------------------
public:

    /**
     * Destructor.
     */
    ~ContourHelper();

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

    // ------------------------- Hidden services ------------------------------
protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    ContourHelper();

private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    ContourHelper ( const ContourHelper & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    ContourHelper & operator= ( const ContourHelper & other );

    // ------------------------- Internals ------------------------------------
private:

}; // end of class ContourHelper


/**
 * Overloads 'operator<<' for displaying objects of class 'ContourHelper'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'ContourHelper' to write.
 * @return the output stream after the writing.
 */
std::ostream&
operator<< ( std::ostream & out, const ContourHelper & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods.
#include "DGtal/geometry/helpers/ContourHelper.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ContourHelper_h

#undef ContourHelper_RECURSES
#endif // else defined(ContourHelper_RECURSES)
