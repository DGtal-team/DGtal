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
 * @file GeometricalDCA.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/09/26
 *
 * @brief Header file for module GeometricalDCA.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GeometricalDCA_RECURSES)
#error Recursive header files inclusion detected in GeometricalDCA.h
#else // defined(GeometricalDCA_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GeometricalDCA_RECURSES

#if !defined GeometricalDCA_h
/** Prevents repeated inclusion of headers. */
#define GeometricalDCA_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CowPtr.h"
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/geometry/2d/SegmentComputerUtils.h"

#include "DGtal/geometry/2d/GeometricalDSS.h"
#include "DGtal/geometry/2d/CircleFrom2Points.h"
#include "DGtal/geometry/2d/Preimage2D.h"
#include "DGtal/geometry/2d/CircleFrom3Points.h"
#include "DGtal/geometry/2d/Point2ShapePredicate.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class GeometricalDCA
  /**
   * @brief Aim:
   *
   * This class is a model of the concept CBidirectionalSegmentComputer. 
   *
   * It should be used with the Curve object (defined in StdDefs.h)
   * and its IncidentPointsRange as follows:
   *
   * @tparam TConstIterator ConstIterator type on STL pairs of 2D points 
  *
   * @see testGeometricalDCA.cpp  exampleGeometricalDCA.cpp testGeometricalDSS.cpp  exampleGeometricalDSS.cpp 
   */
  template <typename TConstIterator>
  class GeometricalDCA
  {

  public:

    //requiered types
    typedef TConstIterator ConstIterator;
    typedef GeometricalDCA<ConstIterator> Self; 
    typedef GeometricalDCA<std::reverse_iterator<ConstIterator> > Reverse;

    //point type
    typedef typename IteratorCirculatorTraits<ConstIterator>::Value Pair; 
    //Pair::first_type and Pair::second_type should be the same type;
    BOOST_STATIC_ASSERT(( ConceptUtils::SameType<typename Pair::first_type, typename Pair::second_type >::value ));
    typedef typename Pair::first_type Point;
    BOOST_STATIC_ASSERT(( Point::dimension == 2 ));
  
  private: 
    
    //other types used for the recognition
    typedef CowPtr<GeometricalDSS<ConstIterator> > GeometricalDSSPtr; 
    typedef CircleFrom3Points<Point> Circle; 
      
    //Predicates used to decide whether the current circle is still seperating or not
    typedef Point2ShapePredicate<Circle,false,true> 
      PInCirclePred; 
    typedef Point2ShapePredicate<Circle,true,true> 
      QInCirclePred; 
  
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor.
     */
    GeometricalDCA();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    GeometricalDCA ( const Self& other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    Self& operator= ( const Self& other );

    /**
     * Destructor.
     */
    ~GeometricalDCA();

    /**
    *  Equality operator
    * @param other the object to compare with.
    * @return 'true' if equal, 'false' otherwise
    *
    * NB: linear in the size of the segment
    */
    bool operator==( const Self & other) const;

    /**
    *  Difference operator
    * @param other the object to compare with.
    * @return 'true' if not equal, 'false' otherwise.
    *
    * NB: linear in the size of the segment
    */
    bool operator!=( const Self & other) const;

    /**
     * Accessor to an instance of the reverse type.
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

    //------------------ accessors -------------------------------
    
    /**
     * @return a separating circle.
     */
    Circle getSeparatingCircle() const;
    
    // ----------------------- growth operations --------------------------------------

    /**
     * Segment initialization
     * @param anIt  any iterator
     */
    void init(const ConstIterator& anIt);

    /**
     * Forward extension of the segment.
     */
    bool extend();

    /**
     * Forward extension test.
     */
    bool isExtendable();

    /**
     * Backward extension of the segment.
     */
    bool extendOppositeEnd();

    /**
     * Backward extension test.
     */
    bool isOppositeEndExtendable();

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
    DrawableWithBoard2D* defaultStyle( std::string mode="" ) const;
    
    /**
     * @return the style name used for drawing this object.
     */
    std::string styleName() const;
    
    /**
       Draw the object on a Board2D board
       @param board the output board where the object is drawn.
    */
    void selfDraw(Board2D & board ) const;

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
     * Pointer to the geometrical DSS.
     */
    GeometricalDSSPtr mySegPtr; 
    /**
     * Separating circle.
     */
    Circle myCircle; 
    /**
     * Flag equal to 'true' if @a mySegPtr has finished its extension 
     * 'false' otherwise. 
     */
    bool myFlagIsInit; 

    // ------------------------- Hidden services ------------------------------
  protected:


  private:


    // ------------------------- Internals ------------------------------------
  private:
    
    /**
     * Check if the two sets of points can be separated by circles
     * passing through the given point @a aPole. 
     * If yes, return the four points of support of the partial preimage.
     * The pole and either @a Pf and @a Ql or @a Qf and @a Pl 
     * implicitely describe a separating circle. 
     *
     * @param itb begin iterator on STL pairs of 2D points.
     * @param ite end iterator on STL pairs of 2D points.
     * @param aPole the point the circles pass through.
     * @param Pf  (returned) first inner point of support.
     * @param Pl  (returned) last inner point of support.
     * @param Qf  (returned) first outer point of support.
     * @param Ql  (returned) last outer point of support.
     *
     * @tparam TIterator type of iterator (normal or reverse type)
     * 
     * @return 'true' if the sets of points can be separated, 'false' otherwise
     */
    template <typename TIterator>
    bool isCircularlySeparable(const TIterator& itb, const TIterator& ite, 
                                              const Point& aPole, 
                                              Point& Pf, Point& Pl, Point& Qf, Point& Ql);  
  
    // ------------------------- Private Datas --------------------------------
  private:

    /**
     * Default drawing style for GeometricalDCA.
     */
    struct DefaultDrawStyle : public DrawableWithBoard2D
    {
      /**
       * Draw the GeometricalDCA on a board
       * @param board the output board where the object is drawn.
       */
      virtual void selfDraw(Board2D & aBoard) const
      {
        aBoard.setLineStyle(Board2D::Shape::SolidStyle);
        aBoard.setPenColor(Color::Red);
        aBoard.setLineWidth(1.5);
        aBoard.setFillColor(Color::None);
      }
    };

  }; // end of class GeometricalDCA


  /**
   * Overloads 'operator<<' for displaying objects of class 'GeometricalDCA'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'GeometricalDCA' to write.
   * @return the output stream after the writing.
   */
  template <typename TConstIterator>
  std::ostream&
  operator<< ( std::ostream & out, const GeometricalDCA<TConstIterator> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/2d/GeometricalDCA.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GeometricalDCA_h

#undef GeometricalDCA_RECURSES
#endif // else defined(GeometricalDCA_RECURSES)
