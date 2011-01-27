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
#include "DGtal/base/Exceptions.h"
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

	/////////////////////////////////////////////////////////////////////////////
	// template class adapterDSS,
	// which is a tool class for FP
	template <typename TInteger, int connectivity>
	class AdapterDSS 
	{
		protected:
			DGtal::ArithmeticalDSS<TInteger,connectivity>* myDSS;
		public:
			virtual DGtal::PointVector<2,TInteger> firstLeaningPoint() const = 0;
			virtual DGtal::PointVector<2,TInteger> lastLeaningPoint() const = 0;
	};

	template <typename TInteger, int connectivity>
	class AdapterDSS4ConvexPart : public AdapterDSS<TInteger,connectivity> 
	{
		public:
			//constructor
			AdapterDSS4ConvexPart(DGtal::ArithmeticalDSS<TInteger,connectivity>& aDSS)
			{
				this->myDSS = &aDSS;
			}
			//accessors
			virtual DGtal::PointVector<2,TInteger> firstLeaningPoint() const 
			{
				return this->myDSS->getUf();
			}
			virtual DGtal::PointVector<2,TInteger> lastLeaningPoint() const
			{
				return this->myDSS->getUl();
			}
	};

	template <typename TInteger, int connectivity>
	class AdapterDSS4ConcavePart : public AdapterDSS<TInteger,connectivity> 
	{
		public:
			//constructor
			AdapterDSS4ConcavePart(DGtal::ArithmeticalDSS<TInteger,connectivity>& aDSS)
			{
				this->myDSS = &aDSS;
			}
			//accessors
			virtual DGtal::PointVector<2,TInteger> firstLeaningPoint() const 
			{
				return this->myDSS->getLf();
			}
			virtual DGtal::PointVector<2,TInteger> lastLeaningPoint() const
			{
				return this->myDSS->getLl();
			}
	};
	/////////////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////////////
  // template class FP
  /**
   * Description of template class 'FP' <p>
   * \brief Aim:
   */
  template <typename TIterator, typename TInteger, int connectivity>
  class FP
  {

    // ----------------------- Types ------------------------------
	public:


  BOOST_CONCEPT_ASSERT(( CInteger<TInteger> ) );

  typedef DGtal::PointVector<2,TInteger> Point;
  typedef DGtal::PointVector<2,TInteger> Vector;
  typedef DGtal::ArithmeticalDSS<TInteger,connectivity> DSS;
  typedef DGtal::AdapterDSS<TInteger,connectivity> AdapterDSS;
  typedef DGtal::AdapterDSS4ConvexPart<TInteger,connectivity> AdapterDSS4ConvexPart;
  typedef DGtal::AdapterDSS4ConcavePart<TInteger,connectivity> AdapterDSS4ConcavePart;
	typedef std::list<Point> Polygon;



    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor.
     * @param aBegin pointer to the first point of the digital curve
     * @param aEnd pointer after the last point of the digital curve
     */
    FP(const TIterator& aBegin, const TIterator& aEnd) throw();

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
     * Draw the FP on a LiBoard board
     * @param board the output board where the object is drawn.
     */
    void selfDrawAsPolygon( DGtalBoard & board ) const;

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

    /**
       * Default style.
       */
    struct DefaultDrawStyle : public DrawableWithDGtalBoard
    {
        virtual void selfDraw(DGtalBoard & aBoard) const
        {
				// Set board style
				aBoard.setLineStyle(DGtalBoard::Shape::SolidStyle);
				aBoard.setPenColor(DGtalBoard::Color::Red);
				aBoard.setLineWidth(2);
				aBoard.setFillColor(DGtalBoard::Color::None);
			  }
    };
    // --------------- CDrawableWithDGtalBoard realization --------------------
  public:
    
    /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
    DrawableWithDGtalBoard* defaultStyle( std::string mode = "" ) const;
    
    /**
     * @return the style name used for drawing this object.
     */
    std::string styleName() const;

    /**
     * Draw the vertices of the FP as a polygonal line 
     * @param board the output board where the object is drawn.
     *
     */
    void selfDraw(DGtalBoard & board ) const;

  }; // end of class FP


  /**
   * Overloads 'operator<<' for displaying objects of class 'FP'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'FP' to write.
   * @return the output stream after the writing.
   */
  template <typename TIterator, typename TInteger, int connectivity>
  std::ostream&
  operator<< ( std::ostream & out, const FP<TIterator,TInteger,connectivity> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/2d/FP.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined FP_h

#undef FP_RECURSES
#endif // else defined(FP_RECURSES)
