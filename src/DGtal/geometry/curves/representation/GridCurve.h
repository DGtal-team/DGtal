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

#include "DGtal/topology/CCellularGridSpaceND.h"
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
    * @brief Aim: describes a sequence of signed d-cells, closed or open, 
    * basically 1-cells or (n-1)-cells, in a space of dimension n (d <= n).
    * For instance, the topological boundary of a simply connected 
    * digital set is a closed sequence of 1-cells in 2d. 
    * 
    * @tparam TKSpace Khalimsky space, a model of CCellularGridSpaceND
    
    Using the namespace Z2i, defined in StdDefs.h, you can instanciate a grid curve as follows:
    @snippet geometry/curves/representation/exampleGridCurve2d.cpp GridCurveDeclaration

     This object provides several IO services. 
     For instance, you can read a grid curve from a data file, 
     which contains the (digital) coordinates of the 0-cells (pointels) in nd: 
    @snippet geometry/curves/representation/exampleGridCurve2d.cpp GridCurveFromDataFile
     Note that if the first and last 0-cells of the file have the same coordinates (i)
     or if only one of their coordinates differ by 1 (ii), then the grid curve is considered
     as closed, ie. scells directly incident to the last signed cell and indirectly incident 
     to the first signed cell are the same.
     
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
    
     - SCellsRange to iterate over the signed d-cells (0 <= d <= n),
     - PointsRange to iterate over the signed d-cells viewed as digital points 
     (of integer coordinates) (0 <= d <= n),
     - MidPointsRange to iterate over the signed d-cells viewed as real points 
     (of double coordinates) (0 <= d <= n),
     - ArrowsRange to iterator over the signed d-cells viewed as a pairs point-vector:
     the point stands for the starting point of the arrow, the vector gives
     the orientation or the arrow (meaningful for d = 1 and n >= 2). 
     - InnerPointsRange to iterate over the (d+1)-cells, viewed as digital points, 
     that are <em>directly</em> incident to the signed d-cells (meaningful for d = n-1 and n >= 2)
     - OuterPointsRange to iterate over the (d+1)-cells, viewed as digital points, 
     that are <em>indirectly</em> incident to the signed d-cells (meaningful for d = n-1 and n >= 2)
     - IncidentPointsRange to iterate over the pairs of (d+1)-cells 
     that are incident to the signed d-cells (both inner points and outer points) 
     (meaningful for d = n-1 and n >= 2)
     - CodesRange to iterate over the signed d-cells viewed as codes {0,1,2,3} (only valid if n = 2 and d = 1)
    
     You can get an access to these eight ranges through the following methods: 

    - getSCellsRange()
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

  template <typename TKSpace = KhalimskySpaceND<2> >
  class GridCurve
  {

  public: 
    typedef TKSpace KSpace; 
    BOOST_CONCEPT_ASSERT(( CCellularGridSpaceND< KSpace > )); 
  
    typedef typename KSpace::Point Point;
    typedef typename KSpace::Point Vector;

    typedef typename KSpace::SCell SCell;
    typedef typename std::vector<SCell> Storage; 

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~GridCurve();

    /**
     * Constructor.
     * @param aKSpace the Khalimsky space where the grid curve lies. 
     */
    GridCurve(const KSpace& aKSpace);

    /**
     * Default Constructor.
     * (the underlying Khalimsky space is default constructed). 
     */
    GridCurve();

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

    // ----------------------- streams ------------------------------

    /**
     * Init.
     * @param in any input stream,
     */
    bool initFromVectorStream(std::istream & in );

    /**
     * Outputs the grid curve to the stream @a out.
     * @param out any output stream,
     */
    void writeVectorToStream( std::ostream & out );

    // ----------------------- Initializations ------------------------------

    /**
     * Deprecated name, use initFromPointsVector instead
     * Init.
     * @param aVectorOfPoints the vector containing a sequence of grid points (digital coordinates).
     * @see initFromPointsRange
     */
    bool initFromVector( const std::vector<Point>& aVectorOfPoints ) throw(ConnectivityException);

    /**
     * Init from a STL vector of points.
     * @param aVectorOfPoints the vector containing a sequence of grid points (digital coordinates).
     * @see initFromPointsRange
     */
    bool initFromPointsVector( const std::vector<Point>& aVectorOfPoints ) throw(ConnectivityException);

    /**
     * Init from a range of points.
     * @param itb begin iterator
     * @param ite end iterator
     */
    template <typename TIterator>
    bool initFromPointsRange( const TIterator& itb, const TIterator& ite ) throw(ConnectivityException);



    /**
     * Init from a STL vector of signed cells.
     * @param aVectorOfSCells the vector containing the sequence of signed cells. 
     * @see initFromSCellsRange
     */
    bool initFromSCellsVector( const std::vector<SCell>& aVectorOfSCells ) throw(ConnectivityException);

    /**
     * Init from a range of signed cells.
     * @param itb begin iterator
     * @param ite end iterator
     */
    template <typename TIterator>
    bool initFromSCellsRange( const TIterator& itb, const TIterator& ite ) throw(ConnectivityException);


    // ----------------------- open/closed ------------------------------


    /**
     * Checks whether the grid curve is open of closed. 
     * Signed cells directly incident to the last scell
     * and indirectly incident to the first scell
     * should be the same in case of a closed grid curve.
     *
     * @return 'true' if grid curve is closed, 'false' otherwise
     */
    bool isClosed() const;

    /**
     * @return 'true' if the grid curve is not closed, 'false' otherwise
     * @see isClosed
     */
    bool isOpen() const;

     
    // ------------------------- private Datas --------------------------------
  private:
    /**
     * Pointer on a Khalimsky space
     */
    const KSpace* myKPtr;
    /**
     * bool equal to 'true' if this owns the Khalimsky space
     * but 'false' otherwise
     */
    bool myFlagIsOwned;

    /**
     * list of signed cells
     */
    Storage mySCells; 


    // ------------------------- Public Datas --------------------------------
  public:



    // ------------------------- Internal --------------------------------
  private:

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
     * Accessor to the range of signed cells
     * @return an instance of SCellsRange
     */
    typename GridCurve::SCellsRange getSCellsRange() const {
      return SCellsRange(mySCells);
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
