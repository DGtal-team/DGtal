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
 * @brief Header file for module GridCurve.cpp
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
#include "DGtal/io/readers/PointListReader.h"

#include "DGtal/base/Circulator.h"
#include "DGtal/base/ConstIteratorAdapter.h"

#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/SCellsFunctors.h"

#include "DGtal/io/boards/Board2D.h"


//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{



  /////////////////////////////////////////////////////////////////////////////
  // class GridCurve
  /////////////////////////////////////////////////////////////////////////////
    /**
    * @brief Aim: describes an
    * alternative sequence of signed 0-cell (pointels) and 1-cell (linels)
    * in any dimension, closed or open. For instance, the
    * topological boundary of a simply connected digital set is a
    * closed grid curve in 2d. 
  
    * Note that an open grid curve always begins and ends with a 0-cell
    * so that the number of 0-cells is equal to the number of 1-cells plus one.  
    * A closed gird curve always begins with a 0-cell too, but always ends
    * with a 1-cell, so that is has as many 0-cells as 1-cells. 
    * 
    * @tparam TKSpace Khalimsky space
    
    Using the namespace Z2i, defined in StdDefs.h, you can instanciate a grid curve as follows:
    @snippet geometry/curves/representation/exampleGridCurve2d.cpp GridCurveDeclaration

     This object provides several IO services. 
     For instance, you can read a grid curve from a data file, 
     which contains the (digital) coordinates of the 0-cells (pointels): 
    @snippet geometry/curves/representation/exampleGridCurve2d.cpp GridCurveFromDataFile
     Note that if the first and last 0-cells of the file have the same coordinates (i)
     or if only one of their coordinates differ by 1 (ii), then the grid curve is considered
     as closed. In case (i), the last 0-cell is removed, whereas in case (ii), a 1-cell is added. 
     
     You can also build a grid curve from the contour of a digital set as follows: 
    @snippet geometry/curves/representation/exampleGridCurve2d.cpp GridCurveFromDigitalSet
    
     To save a grid curve in a data file, GridCurve provides the special method writeVectorToStream():
    @snippet geometry/curves/representation/exampleGridCurve2d.cpp GridCurveToDataFile
    
     The stream mechanism is used to display the true content of the grid curve: 
    @snippet geometry/curves/representation/exampleGridCurve2d.cpp GridCurveStandardOutput

     In 2d, the grid curve can be drawn in a vector graphics file as follows:
    @snippet geometry/curves/representation/exampleGridCurve2d.cpp GridCurveToGraphics
     See @ref dgtal_dgtalboard to learn more about the 2d drawing mechanism
     used in DGtal. 

     Moreover, this object provides several ranges as nested types: 
    
    - in nd:
        - SCellsRange to iterate over the (signed) cells (0-cells or 1-cells),
        - PointsRange to iterate over the 0-cells viewed as integer points,
        - MidPointsRange to iterate over the midpoint of the 1-cells,
        - ArrowsRange to iterator over the (signed) 1-cells viewed as a pair point-vector
        (the point stands for the starting point of the arrow, the vector gives
        the orientation or the arrow). 
    - in 2d: 
        - InnerPointsRange to iterate over the 2-cells, viewed as integer points, 
        that are <em>directly</em> incident to the (signed) 1-cells,
        - OuterPointsRange to iterate over the 2-cells, viewed as integer points, 
        that are <em>indirectly</em> incident to the (signed) 1-cells,
        - IncidentPointsRange to iterate over the pairs of 2-cells 
        that are incident to the 1-cells (both inner points and outer points),
        - CodesRange to iterate over the (signed) 1-cells viewed as codes {0,1,2,3}
    
     You can get an access to these nine ranges through the following methods: 

    - get0SCellsRange()
    - get1SCellsRange()
    - getPointsRange()
    - getMidPointsRange()
    - getArrowsRange()
    - getInnerPointsRange()
    - getOuterPointsRange()
    - getIncidentPointsRange()
    - getCodesRange()
    
    Each range can be displayed in the standard output or can be drawn
    (except CodesRange) in a vector graphics file as shown in the 
    following snippet: 
    @snippet geometry/curves/representation/exampleGridCurve2d.cpp GridCurveIncidentPointsRangeIO
    
     Moreover, each range has the following inner types: 

     - ConstIterator
     - ConstReverseIterator
     - ConstCirculator
     - ConstReverseCirculator

     And each range provides these (circular)iterator services: 

     - begin() : begin ConstIterator
     - end() : end ConstIterator
     - rbegin() : begin ConstReverseIterator
     - rend() : end ConstReverseIterator
     - c() : ConstCirculator
     - rc() : ConstReverseCirculator
     
     You can use these services to iterate over the elements of a given range
     as follows: 
     @snippet geometry/curves/representation/exampleGridCurve2d.cpp GridCurveRangeIterators
         
    * @see exampleGridCurve2d.cpp testGridCurve.cpp
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
     * @param aKSpace the Khalimsky space where the grid curve lies. 
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
     * Init.
     * @param aVectorOfSCells the vector containing the sequence of signed 1-cells. 
     */
    bool initFromSCellsVector( const std::vector<SCell>& aVectorOfSCells ) throw(ConnectivityException);

    /**
     * Outputs the grid curve to the stream @a out.
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
    /**
     * Khalimsky space
     */
    KSpace myK;
    /**
     * list of 0-cells
     */
    Storage my0SCells; 
    /**
     * list of 1-cells
     */
    Storage my1SCells; 

    // ------------------------- Public Datas --------------------------------
  public:



    // ------------------------- Internal --------------------------------
  private:

    /**
     * @return the signed 0-cell associated to a point of integer coordinates 
     */
    SCell PointTo0SCell(const Point& aPoint);
    /**
     * @return the signed 1-cell associated to a pair point - shift vector, 
     * both of integer coordinates 
     */
    SCell PointVectorTo1SCell(const Point& aPoint, const Vector& aVector);
    

    // ------------------------- Drawing services --------------------------------
  public: 

    /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
    DrawableWithBoard2D* defaultStyle( std::string mode="" ) const;
    
    friend DGtal::DrawableWithBoard2D* defaultStyle(const DGtal::GridCurve<TKSpace> &aGC, std::string mode = "" )
    {
      return aGC.defaultStyle(mode);
    }
    
    /**
     * @return the style name used for drawing this object.
     */
    std::string className() const;


    /**
       Draw the object on a Board2D board
       @param board the output board where the object is drawn.
    */
    void selfDraw(Board2D & board ) const;

    friend void draw(Board2D & aBoard, const DGtal::GridCurve<TKSpace> &aGC)
    {
      aGC.selfDraw(aBoard);
    }
    
    // ------------------------- inner classes --------------------------------

  public: 

    ///////////////////////////////////////////////////////////////////////////////
    // Includes range inner types.
    #include "DGtal/geometry/curves/representation/GridCurveRanges.ih"

    /**
     * Accessor to the range of signed 0-cells
     * @return an instance of SCellsRange
     */
    typename GridCurve::SCellsRange get0SCellsRange() const {
      return SCellsRange(my0SCells);
    } 

    /**
     * Accessor to the range of signed 1-cells
     * @return an instance of SCellsRange
     */
    typename GridCurve::SCellsRange get1SCellsRange() const {
      return SCellsRange(my1SCells);
    } 

    /**
     * Accessor to the range of the points of integer coordinates,
     * which are associated to 0-cells of the grid curve 
     * @return an instance of PointsRange
     */
    typename GridCurve::PointsRange getPointsRange() const {
      return PointsRange(this);
    } 

    /**
     * Accessor to the range of the (real coordinates of the) midpoints of each 1-cell
     * @return an instance of MidPointsRange
     */
    typename GridCurve::MidPointsRange getMidPointsRange() const {
      return MidPointsRange(this);
    } 

    /**
     * Accessor to the range of the pairs point - shift vector
     * (of integer coordinates) associated to the 1-cells of the grid curve 
     * @return an instance of ArrowsRange
     */
    typename GridCurve::ArrowsRange getArrowsRange() const {
      return ArrowsRange(this);
    } 

    /**
     * @return an instance of InnerPointsRange
     */
    typename GridCurve::InnerPointsRange getInnerPointsRange() const {
      return InnerPointsRange(this);
    } 

    /**
     * @return an instance of OuterPointsRange
     */
    typename GridCurve::OuterPointsRange getOuterPointsRange() const {
      return OuterPointsRange(this);
    } 

    /**
     * @return an instance of IncidentPointsRange
     */
    typename GridCurve::IncidentPointsRange getIncidentPointsRange() const {
      return IncidentPointsRange(this);
    } 

    /**
     * @return an instance of CodesRange
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
#include "DGtal/geometry/curves/representation/GridCurve.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GridCurve_h

#undef GridCurve_RECURSES
#endif // else defined(GridCurve_RECURSES)
