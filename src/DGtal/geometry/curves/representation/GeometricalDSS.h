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
 * @file GeometricalDSS.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/09/26
 *
 * Header file for module GeometricalDSS.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GeometricalDSS_RECURSES)
#error Recursive header files inclusion detected in GeometricalDSS.h
#else // defined(GeometricalDSS_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GeometricalDSS_RECURSES

#if !defined GeometricalDSS_h
/** Prevents repeated inclusion of headers. */
#define GeometricalDSS_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include <boost/static_assert.hpp>
#include "DGtal/base/CowPtr.h"
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/geometry/curves/representation/SegmentComputerUtils.h"

#include "DGtal/geometry/tools/Preimage2D.h"
#include "DGtal/shapes/fromPoints/StraightLineFrom2Points.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class GeometricalDSS
  /**
   * @brief Aim:
   * On-line recognition of a digital straight segment (DSS)
   * defined as a sequence of connected grid edges such that 
   * there is at least one straight line that separates the centers 
   * of the two incident pixels of each grid edge. 
   *
   * @note On either side, the pixels centers are included. 
   * The class of segments considered here is thus larger
   * than the one considered in ArithmeticalDSS 
   * (the equivalence would be true if the pixels centers 
   * were included on one side but excluded on the other side)
   *
   * The algorithm computes and maintains the preimage
   * of the whole set of separating straight lines in linear time
   * using Preimage2D and the algorithm of O'Rourke (1981). 
   *
   * @note Joseph O'Rourke, An on-line algorithm for fitting straight lines between data ranges,
  Communications of the ACM, Volume 24, Issue 9, September 1981, 574--578. 
   *
   * This class is a model of the concept CBidirectionalSegmentComputer. 
   *
   * It should be used with the Curve object (defined in StdDefs.h)
   * and its IncidentPointsRange as follows:
   * @snippet geometry/curves/representation/exampleGeometricalDSS.cpp GeometricalDSSUsage
   *
   * @tparam TConstIterator ConstIterator type on STL pairs of 2D points 
  *
   * @see testGeometricalDSS.cpp  exampleGeometricalDSS.cpp  Preimage2D ArithmeticalDSS
   */
  template <typename TConstIterator>
  class GeometricalDSS
  {

  public:

    //requiered types
    typedef TConstIterator ConstIterator;
    typedef GeometricalDSS<ConstIterator> Self; 
    typedef GeometricalDSS<std::reverse_iterator<ConstIterator> > Reverse;

    //point type
    typedef typename IteratorCirculatorTraits<ConstIterator>::Value Pair; 
    typedef typename Pair::first_type Point;

    //Pair::first_type and Pair::second_type should be the same type;
    BOOST_STATIC_ASSERT( ( ConceptUtils::SameType
                           < typename Pair::first_type, typename Pair::second_type >
                           ::value ) );

    //preimage
    typedef StraightLineFrom2Points<Point> StraightLine; 
    typedef Preimage2D<StraightLine> Preimage; 
    typedef CowPtr<Preimage> PreimagePtr; 

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor.
     */
    GeometricalDSS();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    GeometricalDSS ( const Self& other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    Self& operator= ( const Self& other );

    /**
     * Destructor.
     */
    ~GeometricalDSS();

    /**
    *  Equality operator
    * @param other the object to compare with.
    * @return 'true' if equal, 'false' otherwise
    */
    bool operator==( const Self & other) const;

    /**
    *  Difference operator
    * @param other the object to compare with.
    * @return 'true' if not equal, 'false' otherwise.
    */
    bool operator!=( const Self & other) const;

    /** 
     * @return a default-constructed instance of Self 
     */    
    Self getSelf() const;

    /**
     * @return a default-constructed instance of Reverse.
     */
     Reverse getReverse() const; 
    

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    /**
     * @return segment begin iterator.
     */
    ConstIterator begin() const;

    /**
     * @return segment end iterator.
     */
    ConstIterator end() const;

    // ----------------------- accessors --------------------------------------

    /**
     * @return first upper leaning point.
     */
    Point getUf() const;

    /**
     * @return last upper leaning point.
     */
    Point getUl() const;

    /**
     * @return first lower leaning point.
     */
    Point getLf() const;

    /**
     * @return last lower leaning point.
     */
    Point getLl() const;

    /**
     * @return 'true' if CW, 'false' if CCW
     */
    bool isClockwiseOriented() const;

    /**
     * Get the parameters of one separating straight line
     * @param alpha  (returned) x-component of the normal
     * @param beta  (returned) y-component of the normal
     * @param gamma  (returned) intercept
     */
    void getParameters(double& alpha, double& beta, double& gamma) const;
    
    /**
     * Projects the point ( @a x , @a y ) onto the 
     * straight line of parameters ( @a alpha , @a beta , @a gamma )
     * @param x  (returned) x-coordinate of the point
     * @param y  (returned) y-coordinate of the point
     * @param alpha  x-component of the direction vector
     * @param beta  y-component of the direction vector
     * @param gamma  intercept
     */
    void projects(double& x, double& y, 
                const double& alpha, const double& beta, const double& gamma) const;

    // ----------------------- growth operations --------------------------------------

    /**
     * Segment initialization
     * @param anIt  any iterator
     */
    void init(const ConstIterator& anIt);

    /**
     * Forward extension of the segment.
     *
     * @return 'true' if the segment is extended
     * and 'false' otherwise.
     */
    bool extendForward();

    /**
     * Forward extension test.
     *
     * @return 'true' if the segment can be extended
     * and 'false' otherwise.
     */
    bool isExtendableForward();

    /**
     * Decide whether the extension of the segment
     * would result in a concave part or not.
     *
     * @return 'true' if the extension of the segment
     * results in a concave part and 'false' otherwise.
     *
     * NB: a true returned value implies that isExtendableForward() returns 'false'
     */
    bool isConcave();

    /**
     * Decide whether the extension of the segment
     * would result in a convex part or not.
     *
     * @return 'true' if the extension of the segment
     * results in a convex part and 'false' otherwise.
     *
     * NB: a true returned value implies that isExtendableForward() returns 'false'
     */
    bool isConvex();

    /**
     * Backward extension of the segment.
     *
     * @return 'true' if the segment is extended
     * and 'false' otherwise.
     */
    bool extendBackward();

    /**
     * Backward extension test.
     *
     * @return 'true' if the segment can be extended
     * and 'false' otherwise.
     */
    bool isExtendableBackward();

    /**
     * Decide whether the extension of the segment
     * would result in a concave part or not.
     *
     * @return 'true' if the extension of the segment
     * results in a concave part and 'false' otherwise.
     *
     * NB: a true returned value implies that isExtendableBackward() returns 'false'
     */
    bool isOppositeEndConcave();

    /**
     * Decide whether the extension of the segment
     * would result in a convex part or not.
     *
     * @return 'true' if the extension of the segment
     * results in a convex part and 'false' otherwise.
     *
     * NB: a true returned value implies that isExtendableBackward() returns 'false'
     */
    bool isOppositeEndConvex();

    //------------------ display -------------------------------
    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
    //DrawableWithBoard2D* defaultStyle( std::string mode="" ) const;
    
    /**
     * @return the style name used for drawing this object.
     */
    std::string className() const;

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:
    /**
     * segment begin iterator.
     */
    ConstIterator myBegin;
    /**
     * segment end iterator.
     */
    ConstIterator myEnd;
    /**
     * Pointer to the preimage.
     */
    PreimagePtr myPreimagePtr; 
    /**
     * Flag equal to 'true' if the segment contains at least two pairs 
     * (the orientation is known) and 'false' otherwise. 
     */
    bool myFlagIsInit; 
    /**
     * Flag equal to 'true' if the pairs of points are clockwise oriented, 
     * 'false' otherwise.
     */
    bool myFlagIsCW; 

    // ------------------------- Hidden services ------------------------------
  protected:


  private:

    

    // ------------------------- Internals ------------------------------------
  private:
    
    // ------------------------- Private Datas --------------------------------
  private:

  }; // end of class GeometricalDSS


  /**
   * Overloads 'operator<<' for displaying objects of class 'GeometricalDSS'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'GeometricalDSS' to write.
   * @return the output stream after the writing.
   */
  template <typename TConstIterator>
  std::ostream&
  operator<< ( std::ostream & out, const GeometricalDSS<TConstIterator> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/curves/representation/GeometricalDSS.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GeometricalDSS_h

#undef GeometricalDSS_RECURSES
#endif // else defined(GeometricalDSS_RECURSES)
