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
#include "DGtal/kernel/RealPointVector.h"
#include "DGtal/geometry/2d/ArithmeticalDSS.h"
#include "DGtal/base/Circulator.h"
#include "DGtal/base/Exceptions.h"
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

	/////////////////////////////////////////////////////////////////////////////
	// template class adapterDSS,
	// which is a tool class for FP
	template <typename ArithmeticalDSS>
	class Adapter 
	{
		protected:
			ArithmeticalDSS* myDSS;
		public:
			virtual typename ArithmeticalDSS::Point firstLeaningPoint() const = 0;
			virtual typename ArithmeticalDSS::Point lastLeaningPoint() const = 0;
	};

	template <typename ArithmeticalDSS>
	class Adapter4ConvexPart : public Adapter<ArithmeticalDSS> 
	{
		public:
			//constructor
			Adapter4ConvexPart(ArithmeticalDSS& aDSS)
			{
				this->myDSS = &aDSS;
			}
			//accessors
			virtual typename ArithmeticalDSS::Point firstLeaningPoint() const 
			{
				return this->myDSS->getUf();
			}
			virtual typename ArithmeticalDSS::Point lastLeaningPoint() const
			{
				return this->myDSS->getUl();
			}
	};

	template <typename ArithmeticalDSS>
	class Adapter4ConcavePart : public Adapter<ArithmeticalDSS> 
	{
		public:
			//constructor
			Adapter4ConcavePart(ArithmeticalDSS& aDSS)
			{
				this->myDSS = &aDSS;
			}
			//accessors
			virtual typename ArithmeticalDSS::Point firstLeaningPoint() const 
			{
				return this->myDSS->getLf();
			}
			virtual typename ArithmeticalDSS::Point lastLeaningPoint() const
			{
				return this->myDSS->getLl();
			}
	};
	/////////////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////////////
  // template class FP
  /**
   * Description of template class 'FP' <p>
   * \brief Aim:Computes the faithful polygon (FP)
   * of a range of 4/8-connected 2D Points. 
   */
  template <typename TIterator, typename TInteger, int connectivity>
  class FP
  {

    // ----------------------- Types ------------------------------
	public:


  BOOST_CONCEPT_ASSERT(( CInteger<TInteger> ) );

  typedef DGtal::PointVector<2,TInteger> Point;
  typedef DGtal::RealPointVector<2> RealPoint;
  typedef DGtal::PointVector<2,TInteger> Vector;
  typedef DGtal::RealPointVector<2> RealVector;

  typedef DGtal::ArithmeticalDSS<TIterator,TInteger,connectivity> DSSComputer;
  typedef DGtal::ArithmeticalDSS<DGtal::Circulator<TIterator>,TInteger,connectivity> DSSComputerInLoop;

	typedef std::list<Point> Polygon;



    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor.
     * @param itb begin iterator
     * @param ite end iterator
     * @param isClosed 'true' if the range has to be considered as circular, 
     * 'false' otherwise. 
     */
    FP(const TIterator& itb, const TIterator& ite, const bool& isClosed) throw( InputException ) ;

    /**
     * Destructor.
     */
    ~FP();

    // ----------------------- Interface --------------------------------------
  public:



    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    /**
     * @return number of FP vertices
     */
    typename Polygon::size_type size() const;


    /**
     * @return the vertices of the FP
     * NB: O(n)
     */
    template <typename OutputIterator>
    OutputIterator copyFP(OutputIterator result) const; 

    /**
     * @return the vertices of the MLP
     * NB: O(n)
     */
    template <typename OutputIterator>
    OutputIterator copyMLP(OutputIterator result) const; 


    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:

		//each vertex of the FP is stored in this list
		Polygon myPolygon; 

		//TRUE if the list has to be consider as circular
    //FALSE otherwise
		bool myFlagIsClosed;

    // ------------------------- Hidden services ------------------------------
  protected:



  private:

    /**
     * @param [aDSS] a DSS lying on a range
     * @param [anAdapter] an Adapter to [aDSS] for convex part
     * if 'true' is returned, for concave part otherwise
     * @param [i] an iterator pointing after the front of [aDSS] 
     * @return 'true' if [aDSS] begins a convex part, 'false' otherwise
     */
    template<typename DSS, typename Adapter>
    bool initConvexityConcavity( DSS &aDSS,  
                                 Adapter* &anAdapter,
                                 const typename DSS::ConstIterator& i );

    /**
     * @param [currentDSS] a DSS lying on a range
     * @param [adapter] an Adapter to [currentDSS]
     * @param [isConvex], 'true' if [currentDSS] is in a convex part, 'false' otherwise
     * @param [i] an iterator pointing after the front of [currentDSS] 
     * @param the algorithm stops when [i] == [end]
     */
    template<typename DSS, typename Adapter>
    void mainAlgorithm( DSS &currentDSS, Adapter* adapter, 
                        bool isConvex, 
                        typename DSS::ConstIterator i, 
                        const typename DSS::ConstIterator& end )  throw( InputException ) ;


    /**
     * gets a MLP vertex from three consecutive vertices of the FP.
     * @param a previous vertex of the FP
     * @param b current vertex of the FP
     * @param c next vertex of the FP
     * @return vertex of the MLP, which is 
     * the tranlated of b by (+- 0.5, +- 0.5)
     */
    RealPoint getRealPoint (const Point& a,const Point& b, const Point& c) const;

    /**
     * @param v any Vector
     * @param q a quandrant number (0,1,2 or 3)
     * @return 'true' if [v] lies in quadrant number [q], 'false' otherwise
     */

    bool quadrant (const Vector& v, const int& q) const;

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

    // ------------------------- Display ------------------------------------
  public: 


    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;


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


    /**
     * Draw the FP on a LiBoard board
     * @param board the output board where the object is drawn.
     */
    void selfDrawAsPolygon( DGtalBoard & board ) const;

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
