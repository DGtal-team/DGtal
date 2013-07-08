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
 * @file ArithmeticalDSSComputer.h 
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 * @date 2010/07/01
 *
 * @brief Header file for module ArithmeticalDSSComputer.cpp
 *
 * This file is part of the DGtal library.
 *
 * @see ArithmeticalDSSComputer.cpp testArithDSS.cpp
 */

#if defined(ArithmeticalDSSComputer_RECURSES)
#error Recursive header files inclusion detected in ArithmeticalDSSComputer.h
#else // defined(ArithmeticalDSSComputer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ArithmeticalDSSComputer_RECURSES

#if !defined ArithmeticalDSSComputer_h
/** Prevents repeated inclusion of headers. */
#define ArithmeticalDSSComputer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>
#include "DGtal/base/Exceptions.h"
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/base/ReverseIterator.h"
#include "DGtal/geometry/curves/ArithmeticalDSS.h"
#include "DGtal/geometry/curves/SegmentComputerUtils.h"

//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{


  /////////////////////////////////////////////////////////////////////////////
  // class ArithmeticalDSSComputer
  /**
   * Description of class 'ArithmeticalDSSComputer' <p>
   * \brief Aim:
   * Dynamic recognition of a digital straight segment (DSS)
   * defined as the sequence of simply connected points (x,y)
   * such that mu <= ax - by < mu + omega.  
   *
   * @note I. DEBLED-RENNESSON, J.-P. REVEILLES, 
   * A linear algorithm for segmentation of digital curves, 
   * International Journal of Pattern Recognition and Artificial
   * Intelligence, Volume 9, N. 6, December 1995. 
   *
   * This class is a model of the concept CDynamicBidirectionalSegmentComputer. 
   *
   *
   * Here is a short example of how to use this class:
   * @snippet geometry/curves/ArithmeticalDSSComputer.cpp ArithmeticalDSSComputer4Usage
   *
   * You must get: 
   *
   * @code
[ArithmeticalDSSComputer]
Parameters (a,b,mu,omega)=(2, 5, -4, 7)
Number of upper patterns: 0
Number of lower patterns: 1
First point [PointVector] {0, 0} Last point [PointVector] {6, 3}
Leaning points:
   Uf [PointVector] {3, 2}
   Ul [PointVector] {3, 2}
   Lf [PointVector] {1, 0}
   Ll [PointVector] {6, 2}
Steps:
   [PointVector] {1, 0}
   [PointVector] {0, 1}
[End ArithmeticalDSSComputer]
   * @endcode
   *
   * @tparam TIterator  type ConstIterator on 2D points,
   * @tparam TInteger  type of scalars used for the DSS parameters
   * (satisfying CInteger) 
   * @tparam adjacency  an integer equal to
   * 4 for standard (4-connected) DSS or 8 for naive (8-connected) DSS. 
   * (Any other integers act as 8). 
   *
   * @see ArithmeticalDSSComputer.cpp testArithDSS.cpp 
   */
  template <typename TIterator, 
    typename TInteger = typename IteratorCirculatorTraits<TIterator>::Value::Coordinate, 
    unsigned short adjacency = 8>
  class ArithmeticalDSSComputer
  {

    // ----------------------- inner types ------------------------------
  public:

    //integer types
    typedef typename IteratorCirculatorTraits<TIterator>::Value::Coordinate Coordinate; 
    typedef TInteger Integer;
    BOOST_CONCEPT_ASSERT(( CInteger<Coordinate> ));
    BOOST_CONCEPT_ASSERT(( CInteger<Integer> ));

    //requiered types
    typedef TIterator ConstIterator;
    typedef ArithmeticalDSSComputer<ConstIterator,TInteger,adjacency> Self; 
    typedef ArithmeticalDSSComputer<ReverseIterator<ConstIterator>,TInteger,adjacency> Reverse;

    //2D point and 2D vector
    typedef typename IteratorCirculatorTraits<ConstIterator>::Value Point; 
    typedef typename IteratorCirculatorTraits<ConstIterator>::Value Vector; 
    BOOST_STATIC_ASSERT(( Point::dimension == 2 ));

    //DSS representation type
    typedef ArithmeticalDSS<Coordinate, Integer, adjacency> DSS; 
    typedef DSS Primitive; 

    // ----------------------- Standard services ------------------------------
  public:


    /**
     * Default constructor.
     * not valid
     */
    ArithmeticalDSSComputer();

    /**
     * Constructor with initialisation
     * @param it an iterator on 2D points
     * @see init
     */
    ArithmeticalDSSComputer(const ConstIterator& it);

    /**
     * Initialisation.
     * @param it an iterator on 2D points
     */
    void init(const ConstIterator& it);


    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    ArithmeticalDSSComputer ( const Self & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    Self& operator= ( const Self & other );

    /** 
     * @return a default-constructed instance of Self 
     */
    Self getSelf() const;

    /**
     * @return a default-constructed instance of Reverse
     */
    Reverse getReverse() const;
        
    /**
     * Equality operator.
     * @param other the object to compare with.
     * @return 'true' either if the leaning points perfectly match
     * or if the first leaning points match to the last ones
     * (same DSS scanned in the reverse way) 
     * and 'false' otherwise
     */
    bool operator==( const Self & other ) const;

    /**
     * Difference operator.
     * @param other the object to compare with.
     * @return 'false' if equal
     * 'true' otherwise
     */
    bool operator!=( const Self & other ) const;

    /**
     * Destructor.
     */
    ~ArithmeticalDSSComputer(){};

    // ----------------------- Interface --------------------------------------
  public:
     
    /**
     * Tests whether the current DSS can be extended at the front.
     *  
     * @return 'true' if yes, 'false' otherwise.
     */
    bool isExtendableForward();

    /**
     * Tests whether the current DSS can be extended at the back.
     *  
     * @return 'true' if yes, 'false' otherwise.
     */
    bool isExtendableBackward();
    
    /**
     * Tests whether the current DSS can be extended at the front.
     * Computes the parameters of the extended DSS if yes.
     * @return 'true' if yes, 'false' otherwise.
     */
    bool extendForward();

    /**
     * Tests whether the current DSS can be extended at the back.
     * Computes the parameters of the extended DSS if yes.
     * @return 'true' if yes, 'false' otherwise.
     */
    bool extendBackward();

    /**
     * Removes the first point of the DSS (at back) 
     * if it has more than two points
     * @return 'true' if the first point is removed, 'false' otherwise.
     */
    bool retractForward();

    /**
     * Removes the last point of the DSS (at front)
     * if it has more than two points
     * @return 'true' if the last point is removed, 'false' otherwise.
     */
    bool retractBackward();


    // ------------------------- Accessors ------------------------------
    /**
     * @return the current arithmetical DSS.
     */
    const Primitive& primitive() const;
    /**
     * @return an Integer of value @a myA.
     */
    Integer getA() const;
    /**
     * @return an Integer of value @a myB.
     */
    Integer getB() const;
    /**
     * @return an Integer of value @a myMu.
     */
    Integer getMu() const;
    /**
     * @return an Integer of value @a myOmega.
     */
    Integer getOmega() const;
    /**
     * Accessor to the first upper leaning point
     * @return first upper leaning point.
     */
    Point getUf() const;
    /**
     * Accessor to the last upper leaning point
     * @return last upper leaning point.
     */
    Point getUl() const;
    /**
     * Accessor to the first lower leaning point
     * @return first lower leaning point.
     */
    Point getLf() const;
    /**
     * Accessor to the last lower leaning point
     * @return last lower leaning point.
     */
    Point getLl() const;
    /**
     * Accessor to the first added point to the DSS
     * @return the first point of the DSS.
     * @see getFirstPoint
     */
    Point getBackPoint() const ;
    /**
     * Accessor to the last added point to the DSS
     * @return the last point of the DSS.
     * @see getLastPoint
     */
    Point getFrontPoint() const ;
    /**
     * Accessor to the first added point to the DSS
     * @return the first point of the DSS.
     * @see getBackPoint
     */
    Point getFirstPoint() const ;
    /**
     * Accessor to the last added point to the DSS
     * @return the last point of the DSS.
     * @see getFrontPoint
     */
    Point getLastPoint() const;
    /**
     * @return begin iterator of the DSS range.
     */
    ConstIterator begin() const;
    /**
     * @return end iterator of the DSS range.
     */
    ConstIterator end() const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;


    // ------------------ Useful tools -----------------------------------

    /**
     * Computes the remainder of a point
     * (that does not necessarily belong to the DSS)
     * @param it an iterator on points
     * @return the remainder of @a *it.
     */
    Integer getRemainder(const ConstIterator & it) const;

    /**
     * Computes the remainder of a point
     * (that does not necessarily belong to the DSS)
     * @param aPoint the point whose remainder is returned 
     * @return @a myA * @a aPoint[0] - @a myB * @a aPoint[1].
     */
    Integer getRemainder( const Point& aPoint ) const;

    /**
     * Computes the position of a point
     * (that does not necessarily belong to the DSS)
     * @param it an iterator on points
     * @return the position of @a *it.
     */
    Integer getPosition(const ConstIterator & it) const;

    /**
     * Computes the position of a point
     * (that does not necessarily belong to the DSS)
     * @param aPoint the point whose position is returned 
     * @return @a myA * @a aPoint[0] + @a myB * @a aPoint[1].
     */
    Integer getPosition( const Point& aPoint ) const;

    /**
     * Checks whether a point is in the DSL
     * of parameters ( @a myA, @a myB, @a myMu, @a myOmega )
     * @param aPoint the point to be checked 
     * @return 'true' if yes, 'false' otherwise
     */
    bool isInDSL( const Point& aPoint ) const;

    /**
     * Checks whether a point is in the DSL
     * of parameters ( @a myA, @a myB, @a myMu, @a myOmega )
     * @param it an iterator on the point to be checked
     * @return 'true' if yes, 'false' otherwise
     */
    bool isInDSL(const ConstIterator & it) const;

    /**
     * Checks whether a point belongs to the DSS or not
     * @param aPoint the point to be checked
     * @return 'true' if yes, 'false' otherwise
     */
    bool isInDSS( const Point& aPoint ) const;

    /**
     * Checks whether a point belongs to the DSS or not
     * @param it an iterator on the point to be checked
     * @return 'true' if yes, 'false' otherwise
     */
    bool isInDSS(const ConstIterator & it) const;


    // ------------------------- Hidden services ------------------------------
  private:


    // ------------------------- Protected Datas ------------------------------
  protected:

    /**
    * DSS representation
    */
    DSS myDSS; 
    /**
    * begin iterator
    */
    ConstIterator myBegin; 
    /**
    * end iterator
    */
    ConstIterator myEnd; 
    
    // ------------------ Display ------------------------------------------

  public:
    
    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) ; 

    
  }; // end of class ArithmeticalDSSComputer


/**
 * Overloads 'operator<<' for displaying objects of class 'ArithmeticalDSSComputer'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'ArithmeticalDSSComputer' to write.
 * @return the output stream after the writing.
 */
template <typename TIterator, typename TInteger, unsigned short adjacency>
std::ostream&
operator<< ( std::ostream & out,  ArithmeticalDSSComputer<TIterator,TInteger,adjacency> & object )
{
  object.selfDisplay( out);
  return out;
}


} // namespace DGtal



///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods.
#include "DGtal/geometry/curves/ArithmeticalDSSComputer.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ArithmeticalDSSComputer_h

#undef ArithmeticalDSSComputer_RECURSES
#endif // else defined(ArithmeticalDSSComputer_RECURSES)
