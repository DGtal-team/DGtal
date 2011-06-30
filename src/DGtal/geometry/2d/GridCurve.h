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
 * @file GridCurve.h
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 *
 * @date 2011/06/27
 *
 * Header file for module GridCurve.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GridCurve_RECURSES)
#error Recursive header files inclusion detected in GridCurve.h
#else // defined(GridCurve_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GridCurve_RECURSES

#if !defined GridCurve_h
/** Prevents repeated inclusion of headers. */
#define GridCurve_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <sstream>
#include <vector>
#include <iterator>
#include <cstddef>


#include "DGtal/base/BasicTypes.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Exceptions.h"
#include "DGtal/io/readers/PointListReader.h"

#include "DGtal/topology/KhalimskySpaceND.h"

#include "DGtal/io/DGtalBoard.h"



//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class GridCurve
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of class 'GridCurve' <p> Aim: describes an
   * alternative sequence of signed 0-cell (pointels) and 1-cell (linels)
   * in any dimension, closed or open. For instance, the
   * topological boundary of a  simply connected digital set is a
   * closed grid curve. This object provides several ranges, such as
   * PointsRange used to get the (integer) coordinates of the pointels
   * of the grid curve. 
   *
   * Example :
   * @code 

   * @endcode
   */

  template <typename KSpace>
  class GridCurve
  {

  public: 

  typedef typename KSpace::Space::Point Point;
  typedef typename KSpace::Space::Point Vector;

  typedef typename KSpace::SCell Cell;
  typedef typename std::vector<Cell> Storage;





    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~GridCurve(){};


    /**
     * Constructor.
     */
    GridCurve(){};


    /**
     * Init.
     * @param aVectorOfPoints the vector containing the sequence of grid points. 
     */
    void initFromVector( const std::vector<Point> aVectorOfPoints ) throw(ConnectivityException);

    /**
     * Init.
     * @param in any input stream,
     */
    void initFromVectorStream(std::istream & in );

    /**
     * Init.
     * @param in any input stream,
     */
    void initFromFreemanChainStream(std::istream & in ) throw(InputException);

    /**
     * Outputs the grid curve to the stream [out].
     * @param out any output stream,
     */
    void writeVectorToStream( std::ostream & out );

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    GridCurve( const GridCurve & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    GridCurve & operator=( const GridCurve & other );

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



     
    // ------------------------- private Datas --------------------------------
  private:


    // ------------------------- Public Datas --------------------------------
  public:
    KSpace myK;

    Storage my0Cells; 
    Storage my1Cells; 


    // ------------------------- Internal --------------------------------
  private:

    //conversion methods
    Cell PointTo0Cell(const Point& aPoint);
    Cell PointVectorTo1Cell(const Point& aPoint, const Vector& aVector);
 /*   Point 0CellToPoint(const Cell& aCell);
    Vector 1CellToVector(const Cell& aCell);*/
    

    // ------------------------- Drawing services --------------------------------
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
       Draw the object on a DGtalBoard board
       @param board the output board where the object is drawn.
    */
    void selfDraw(DGtalBoard & board ) const;
    
    /**
       Draw the points on a DGtalBoard board
       @param board the output board where the object is drawn.
    */
    void selfDrawPoints(DGtalBoard & board ) const; 
    /**
       Draw the grid edges on a DGtalBoard board
       @param board the output board where the object is drawn.
    */
    void selfDrawEdges(DGtalBoard & board ) const;

  private: 

    /**
     * Default Style Functor for selfDraw methods
     *
     * @param aBoard
     */

    struct SelfDrawStyle
    {
      SelfDrawStyle(DGtalBoard & aBoard)
      {
      }
    };

    struct DefaultDrawStyle : public DrawableWithDGtalBoard
    {
      virtual void selfDraw( DGtalBoard & aBoard ) const
      {
      }
    };

    struct DefaultDrawStylePoints : public DrawableWithDGtalBoard
    {
      virtual void selfDraw( DGtalBoard & aBoard ) const
      {
      }
    };

  struct DefaultDrawStyleEdges : public DrawableWithDGtalBoard
    {
      virtual void selfDraw( DGtalBoard & aBoard ) const
      {
	      aBoard.setLineStyle (LibBoard::Shape::SolidStyle );
	      aBoard.setFillColor(DGtalBoard::Color::None);
      }
    };

  // ------------------------- inner classes --------------------------------

  public: 



  }; // end of class GridCurve




  /**
   * Overloads 'operator<<' for displaying objects of class 'GridCurve'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'GridCurve' to write.
   * @return the output stream after the writing.
   */
  template<typename KSpace>
  std::ostream&
  operator<< ( std::ostream & out, const GridCurve<KSpace> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/geometry/2d/GridCurve.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GridCurve_h

#undef GridCurve_RECURSES
#endif // else defined(GridCurve_RECURSES)
