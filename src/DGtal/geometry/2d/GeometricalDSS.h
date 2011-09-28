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
#include "DGtal/base/CowPtr.h"
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/geometry/2d/SegmentComputerUtils.h"

#include "DGtal/geometry/2d/Preimage2D.h"
#include "DGtal/geometry/2d/StraightLineFrom2Points.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class GeometricalDSS
  /**
   * \brief Aim:
   * @tparam TConstIterator ConstIterator type on STL pairs of 2D points 
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
     * Get the parameters of one separating straight line
     * @param alpha  (returned) x-component of the normal
     * @param beta  (returned) y-component of the normal
     * @param gamma  (returned) intercept
     */
    void getParameters(double& alpha, double& beta, double& gamma) const;


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
     * preimage.
     */
    PreimagePtr myPreimagePtr; 


    // ------------------------- Hidden services ------------------------------
  protected:


  private:

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

    // ------------------------- Internals ------------------------------------
  private:
    
    // ------------------------- Private Datas --------------------------------
  private:

    /**
     * Default drawing style for GeometricalDSS.
     */
    struct DefaultDrawStyle : public DrawableWithBoard2D
    {
      /**
       * Draw the GeometricalDSS on a board
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
#include "DGtal/geometry/2d/GeometricalDSS.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GeometricalDSS_h

#undef GeometricalDSS_RECURSES
#endif // else defined(GeometricalDSS_RECURSES)
