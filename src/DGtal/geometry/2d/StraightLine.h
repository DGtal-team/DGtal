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
 * @file StraightLine.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/10/27
 *
 * Header file for module StraightLine.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(StraightLine_RECURSES)
#error Recursive header files inclusion detected in StraightLine.h
#else // defined(StraightLine_RECURSES)
/** Prevents recursive inclusion of headers. */
#define StraightLine_RECURSES

#if !defined StraightLine_h
/** Prevents repeated inclusion of headers. */
#define StraightLine_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/CInteger.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{


  /////////////////////////////////////////////////////////////////////////////
  // template class StraightLine
  /**
   * Description of template class 'StraightLine' <p>
   * \brief Aim: Represents a StraightLine uniquely
	 * defined by two 2D points and that is able
   * to return for each 2D point of the domain
   * its signed distance to itself 
   */
  template <typename TInteger>
  class StraightLine
  {

    // ----------------------- associated types ------------------------------
	public:

		//2D point and 2D vector
	  BOOST_CONCEPT_ASSERT(( CInteger<TInteger> ) );
		typedef TInteger Coordinate;
		typedef DGtal::PointVector<2,Coordinate> Point;
		typedef DGtal::PointVector<2,Coordinate> Vector;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor.
		 * @param firstPoint, secondPoint, two points
     * that uniquely define the StraightLine
     */
    StraightLine(const Point& aFirstPoint, const Point& aSecondPoint);

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    StraightLine ( const StraightLine & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    StraightLine & operator= ( const StraightLine & other );


    /**
     * Destructor. Does nothing
     */
    ~StraightLine();

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
     * Computes the signed distance of [aP] to the StraightLine
     * @param aP, the point to be tested.
     * @return the signed distance.
     */
    Coordinate signedDistance(const Point& aP) const;



    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:
		//the two points that uniquely define the StraightLine
		Point myP, myQ;
    // ------------------------- Hidden services ------------------------------
  protected:


  private:



    // ------------------------- Internals ------------------------------------
  private:




  }; // end of class StraightLine


	template <typename TInteger>
	inline
	std::ostream&
	operator<< ( std::ostream & out, 
				const StraightLine<TInteger> & object )
	{
		object.selfDisplay( out );
		return out;
	}


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/2d/StraightLine.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined StraightLine_h

#undef StraightLine_RECURSES
#endif // else defined(StraightLine_RECURSES)
