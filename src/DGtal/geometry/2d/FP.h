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
 * @file FP.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/01/26
 *
 * Header file for module FP.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(FP_RECURSES)
#error Recursive header files inclusion detected in FP.h
#else // defined(FP_RECURSES)
/** Prevents recursive inclusion of headers. */
#define FP_RECURSES

#if !defined FP_h
/** Prevents repeated inclusion of headers. */
#define FP_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>
#include "DGtal/kernel/CInteger.h"
#include "DGtal/geometry/2d/ArithmeticalDSS.h"
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class FP
  /**
   * Description of template class 'FP' <p>
   * \brief Aim:
   */
  template <typename TInteger, int connectivity>
  class FP
  {

    // ----------------------- Types ------------------------------
public:


  BOOST_CONCEPT_ASSERT(( CInteger<TInteger> ) );

  typedef TInteger Integer;
  typedef DGtal::PointVector<2,Integer> Point;
  typedef DGtal::PointVector<2,Integer> Vector;
  typedef DGtal::ArithmeticalDSS<TInteger,connectivity> DSS;
	typedef std::list<Point> Polygon;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor.
     */
    FP();

    /**
     * Destructor.
     */
    ~FP();

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

		//each vertex of the FP is stored in this list
		Polygon myPolygon; 

		//boolean at TRUE is the list has to be consider as circular
    //FALSE otherwise
		bool isClosed;

    // ------------------------- Hidden services ------------------------------
  protected:



  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    FP ( const FP & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    FP & operator= ( const FP & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class FP


  /**
   * Overloads 'operator<<' for displaying objects of class 'FP'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'FP' to write.
   * @return the output stream after the writing.
   */
  template <typename TInteger, int connectivity>
  std::ostream&
  operator<< ( std::ostream & out, const FP<TInteger,connectivity> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/2d/FP.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined FP_h

#undef FP_RECURSES
#endif // else defined(FP_RECURSES)
