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
#include <utility>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/readers/PointListReader.h"

#include "DGtal/base/ConstIteratorAdapter.h"
#include "DGtal/base/Modifier.h"
#include "DGtal/base/Circulator.h"

#include "DGtal/topology/KhalimskySpaceND.h"

#include "DGtal/io/boards/Board2D.h"



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

  template <typename TKSpace>
  class GridCurve
  {

  public: 
    typedef TKSpace KSpace; 
  
    typedef typename KSpace::Point Point;
    typedef typename KSpace::Point Vector;

    typedef typename KSpace::SCell SCell;
    typedef typename std::vector<SCell> Storage;





    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~GridCurve(){};

    /**
     * Constructor.
     */
    GridCurve(const KSpace& aKSpace) : myK(aKSpace) {};

    /**
     * Default Constructor.
     */
    GridCurve(){};


    /**
     * Init.
     * @param aVectorOfPoints the vector containing the sequence of grid points. 
     */
    bool initFromVector( const std::vector<Point>& aVectorOfPoints ) throw(ConnectivityException);

    /**
     * Init.
     * @param in any input stream,
     */
    bool initFromVectorStream(std::istream & in );


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


    /**
     * @return 'true' if grid curve is open, 'false' otherwise
     */
    bool isOpen() const;

    /**
     * @return 'true' if grid curve is closed, 'false' otherwise
     */
    bool isClosed() const;

     
    // ------------------------- private Datas --------------------------------
  private:
    KSpace myK;

    Storage my0SCells; 
    Storage my1SCells; 

    // ------------------------- Public Datas --------------------------------
  public:



    // ------------------------- Internal --------------------------------
  private:

    //conversion methods
    SCell PointTo0SCell(const Point& aPoint);
    SCell PointVectorTo1SCell(const Point& aPoint, const Vector& aVector);
    

    // ------------------------- Drawing services --------------------------------
  public: 

    /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
    DrawableWithBoard2D* defaultStyle( std::string mode = "" ) const;
    
    /**
     * @return the style name used for drawing this object.
     */
    std::string styleName() const;
    
    /**
       Draw the object on a Board2D board
       @param board the output board where the object is drawn.
    */
    void selfDraw(Board2D & board ) const;


    // ------------------------- inner classes --------------------------------

  public: 

    ///////////////////////////////////////////////////////////////////////////////
    // Includes range types.
    #include "DGtal/geometry/2d/GridCurveRanges.ih"

    /**
     * Accessor of a range of 0-cells
     * @return SCellsRange
     */
    typename GridCurve::SCellsRange get0SCellsRange() const {
      return SCellsRange(my0SCells);
    } 

    /**
     * Accessor of a range of 1-cells
     * @return SCellsRange
     */
    typename GridCurve::SCellsRange get1SCellsRange() const {
      return SCellsRange(my1SCells);
    } 

    /**
     * Accessor of the range of the integer coordinates of the pointels
     * @return PointsRange
     */
    typename GridCurve::PointsRange getPointsRange() const {
      return PointsRange(this);
    } 

    /**
     * Accessor of the range of the (real coordinates of the) midpoints of each 1-cell
     * @return MidPointsRange
     */
    typename GridCurve::MidPointsRange getMidPointsRange() const {
      return MidPointsRange(this);
    } 

    /**
     * Range of the pair of point and displacement vector
     * (integer coordinates) associated to the 1-cells 
     * @return ArrowsRange
     */
    typename GridCurve::ArrowsRange getArrowsRange() const {
      return ArrowsRange(this);
    } 

    /**
     * @return InnerPointsRange
     */
    typename GridCurve::InnerPointsRange getInnerPointsRange() const {
      return InnerPointsRange(this);
    } 

    /**
     * @return OuterPointsRange
     */
    typename GridCurve::OuterPointsRange getOuterPointsRange() const {
      return OuterPointsRange(this);
    } 

    /**
     * @return IncidentPointsRange
     */
    typename GridCurve::IncidentPointsRange getIncidentPointsRange() const {
      return IncidentPointsRange(this);
    } 

    /**
     * @return CodesRange
     */
    typename GridCurve::CodesRange getCodesRange() const {
      return CodesRange(this);
    } 

  }; // end of class GridCurve




  /**
   * Overloads 'operator<<' for displaying objects of class 'GridCurve'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'GridCurve' to write.
   * @return the output stream after the writing.
   */
  template<typename TKSpace>
  std::ostream&
  operator<< ( std::ostream & out, const GridCurve<TKSpace> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods.
#include "DGtal/geometry/2d/GridCurve.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GridCurve_h

#undef GridCurve_RECURSES
#endif // else defined(GridCurve_RECURSES)
