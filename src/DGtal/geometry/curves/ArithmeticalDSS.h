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
 * @file ArithmeticalDSS.h 
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 * @date 2010/07/01
 *
 * @brief Header file for module ArithmeticalDSS.cpp
 *
 * This file is part of the DGtal library.
 *
 * @see ArithmeticalDSS.cpp testArithDSS.cpp
 */

#if defined(ArithmeticalDSS_RECURSES)
#error Recursive header files inclusion detected in ArithmeticalDSS.h
#else // defined(ArithmeticalDSS_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ArithmeticalDSS_RECURSES

#if !defined ArithmeticalDSS_h
/** Prevents repeated inclusion of headers. */
#define ArithmeticalDSS_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>
#include "DGtal/base/Exceptions.h"
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/base/ReverseIterator.h"
#include "DGtal/io/Color.h"

#include "DGtal/geometry/curves/SegmentComputerUtils.h"

//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{


  /////////////////////////////////////////////////////////////////////////////
  // class ArithmeticalDSS
  /**
   * Description of class 'ArithmeticalDSS' <p>
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
   * @snippet geometry/curves/ArithmeticalDSS.cpp ArithmeticalDSS4Usage
   *
   * You must get: 
   *
   * @code
[ArithmeticalDSS]
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
[End ArithmeticalDSS]
   * @endcode
   *
   * @tparam TIterator  type ConstIterator on 2D points,
   * @tparam TInteger  type of scalars used for the DSS parameters
   * (satisfying CInteger) 
   * @tparam connectivity  an integer equal to
   * 4 for standard (4-connected) DSS or 8 for naive (8-connected) DSS. 
   * (Any other integers act as 8). 
   *
   * @see ArithmeticalDSS.cpp testArithDSS.cpp 
   */
  template <typename TIterator, 
    typename TInteger = typename IteratorCirculatorTraits<TIterator>::Value::Coordinate, 
    int connectivity = 8>
  class ArithmeticalDSS
  {



    //////////////////////////////////////////////////////////////////////////////
    // generic class for computing the norm of (a,b)
    // max(a,b) for 8-connectivity 
    // |a|+|b| for 4-connectivity

    //default (for 8-connectivity) 
    template <typename TInt, int c>
    struct Tools
    {
      static TInt norm(const TInt& a, const TInt& b) 
      {
	TInt x;
	if (a>=0) x = a;
	else x = -a;
	TInt y;
	if (b>=0) y = b;
	else y = -b;
	return (x>=y)?x:y;
      }
      static TInt dualNorm(const TInt& a, const TInt& b) 
      {
	if (a > 0) {
	  if (b > 0) {
	    return (a+b);
	  } else {
	    return (a-b);    
	  }
	} else {
	  if (b > 0) {
	    return (-a+b);            
	  } else {
	    return (-a-b);    
	  }
	}
      }
    };

    //specialisation for 4-connectivity
    template <typename TInt>
    struct Tools<TInt,4> 
    {
      static TInt norm(const TInt& a, const TInt& b) 
      {
	if (a > 0) {
	  if (b > 0) {
	    return (a+b);
	  } else {
	    return (a-b);    
	  }
	} else {
	  if (b > 0) {
	    return (-a+b);            
	  } else {
	    return (-a-b);    
	  }
	}
      }

      static TInt dualNorm(const TInt& a, const TInt& b) 
      {
	TInt x;
	if (a>=0) x = a;
	else x = -a;
	TInt y;
	if (b>=0) y = b;
	else y = -b;
	return (x>=y)?x:y;
      }

    };


    // ----------------------- inner types ------------------------------
  public:

    //entier
    BOOST_CONCEPT_ASSERT(( CInteger<TInteger> ));
    typedef TInteger Integer;

    //requiered types
    typedef TIterator ConstIterator;
    typedef ArithmeticalDSS<ConstIterator,TInteger,connectivity> Self; 
    typedef ArithmeticalDSS<ReverseIterator<ConstIterator>,TInteger,connectivity> Reverse;

    //2D point and 2D vector
    typedef typename IteratorCirculatorTraits<ConstIterator>::Value Point; 
    typedef typename IteratorCirculatorTraits<ConstIterator>::Value Vector; 

    //Point should be 2D Point
    //uncomment if CPointVector is written
    //BOOST_CONCEPT_ASSERT(( CPointVector<Point> ));
    BOOST_STATIC_ASSERT(( Point::dimension == 2 ));

    typedef PointVector<2,double> PointD;  


    // ----------------------- Standard services ------------------------------
  public:


    /**
     * Default constructor.
     * not valid
     */
    ArithmeticalDSS();

    /**
     * Constructor with initialisation
     * @param it an iterator on 2D points
     * @see init
     */
    ArithmeticalDSS(const ConstIterator& it);

    /**
     * Initialisation.
     * @param it an iterator on 2D points
     */
    void init(const ConstIterator& it);


    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    ArithmeticalDSS ( const Self & other );

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
    ~ArithmeticalDSS(){};

    // ----------------------- Interface --------------------------------------
  public:
     
    /**
     * deprecated
     * @see isExtendableForward()
     *
     * Tests whether the union between a point 
     * (adding to the front of the DSS 
     * with respect to the scan orientaion) 
     * and a DSS is a DSS. 
     * @param itf an iterator on a sequence of points
     * @return 'true' if the union is a DSS, 'false' otherwise.
     */
    bool isExtendableForward(const ConstIterator & itf) ;

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
     * deprecated
     * @see extendForward()
     *
     * Tests whether the union between a point 
     * (adding to the front of the DSS 
     * with respect to the scan orientaion) 
     * and a DSS is a DSS. 
     * Computes the parameters of the new DSS 
     * with the adding point if true.
     * @param itf an iterator on a sequence of points
     * @return 'true' if the union is a DSS, 'false' otherwise.
     */
    bool extendForward(const ConstIterator & itf)  ;

    /**
     * deprecated
     * @see extendBackward()
     *
     * Tests whether the union between a point 
     * (adding to the back of the DSS 
     * with respect to the scan orientaion) 
     * and a DSS is a DSS. 
     * Computes the parameters of the new DSS 
     * with the adding point if true.
     * @param itb an iterator on a sequence of points
     * @return 'true' if the union is a DSS, 'false' otherwise.
     */
    bool extendBackward(const ConstIterator & itb) ;

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
     * deprecated
     * Accessor to the first added point to the DSS
     * @return the first point of the DSS.
      */
    Point getBackPoint() const ;
    /**
     * deprecated
     * Accessor to the last added point to the DSS
     * @return the last point of the DSS.
     */
    Point getFrontPoint() const ;
    /**
     * Accessor to the first added point to the DSS
     * @return the first point of the DSS.
     */
    Point getFirstPoint() const ;
    /**
     * Accessor to the last added point to the DSS
     * @return the last point of the DSS.
     */
    Point getLastPoint() const;
    /**
     * deprecated
     * @see begin() end()
     *
     * Accessor to the iterator at the back of the DSS
     * @return iterator pointing to the back of the DSS.
     */
    ConstIterator getBack() const ;
    /**
     * deprecated
     * @see begin() end()
     *
     * Accessor to the iterator at the front of the DSS
     * @return iterator pointing to the front of the DSS.
     */
    ConstIterator getFront() const ;
    
    /**
     *  
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

    /**
     * Tests whether the union between @a lastPoint 
     * and the DSS is a DSS. 
     * Computes the parameters of the new DSS 
     * with the adding point if true.
     *
     * @param lastPoint  the point to add
     * @param lastMove  shift vector to the point to add
     *
     * @return 'true' if the union is a DSS, 'false' otherwise.
     */
    bool isExtendableForward( const Point & lastPoint, 
			      const Vector & lastMove );
    
    /**
     * Tests whether the union between a point 
     * (pointing to by @a it) and the DSS is a DSS. 
     * Computes the parameters of the new DSS 
     * with the adding point if true.
     *
     * @param it  an iterator on a 2D point
     * @param lastIt  an iterator pointing to the end of the DSS 
     * @param lastMove  shift vector to the point pointed to by @a it
     * @param Uf  first upper leaning point  
     * @param Ul  last upper leaning point 
     * @param Lf  first lower leaning point  
     * @param Ll  last lower leaning point 
     * 
     * @return 'true' if the union is a DSS, 'false' otherwise.
     */
    bool extendForward( const ConstIterator & it, 
			ConstIterator & lastIt, 
			const Vector & lastMove,
			Point & Uf,  Point & Ul,
			Point & Lf,  Point & Ll );
    
    /**
     * Removes the end point of a DSS
     * (pointing to by @a firstIt)
     *
     * @param firstIt  an iterator pointing to the end of the DSS 
     * @param lastIt  an iterator pointing to the other end of the DSS 
     * @param nextIt  an iterator pointing to the point of the DSS close
     to the one pointing to by @a firstIt
     * @param Uf  first upper leaning point  
     * @param Ul  last upper leaning point 
     * @param Lf  first lower leaning point  
     * @param Ll  last lower leaning point 
     * @param s  a signed integer equal to 1 or -1
     *
     * @return 'true'.
     */
    bool retractForward( ConstIterator & firstIt,
                        ConstIterator & lastIt,
                        ConstIterator & nextIt,       
                        Point & Uf,  Point & Ul,
                        Point & Lf,  Point & Ll,
                        const Integer& s );


    /**
     * Checks whether the DSS has less or more
     * than two shift vectors (steps)
     * between two consecutive points
     * (must be called only in the main stage)
     * @param aStep  the last shift vector. 
     * @return 'true' if less or equal, 'false' otherwise.
     */
    bool hasLessThanTwoSteps(const Vector& aStep) const;


    /**
     * Returns the 2D vector 
     * starting at a point of remainder 0
     * and pointing to the closer point of
     * remainder omega
     * @return the 2D vector.
     */
    Vector vectorFrom0ToOmega() const;



    // ------------------------- Protected Datas ------------------------------
  protected:

    //parameters of the DSS
    /**
    * y-component of the direction vector
    */
    Integer myA;
    /**
    * x-component of the direction vector
    */
    Integer myB;
    /**
    * Intercept
    */
    Integer myMu;
    /**
    * Thickness
    */
    Integer myOmega;

    //number of upper and lower patterns
    /**
    * Number of upper patterns ( @a myUf = @a myNbUpPat . ( @a myB , @a myA ) + @a myUl )
    */
    Integer myNbUpPat;
    /**
    * Number of lower patterns ( @a myLf = @a myNbLowPat . ( @a myB , @a myA ) + @a myLl )
    */
    Integer myNbLowPat; 

    
    /**
    * Steps of the DSS (eg. right and up in the first octant)
    */
    std::vector<Vector> mySteps;
    
    // ------------------------- Private Datas --------------------------------
  
  private:


    // ------------------ Display ------------------------------------------

  public:
    //leaning points (here because Display2DFactory, todo: accessors)
    /**
    * First upper leaning point ( of remainder @a myMu )
    */
    Point myUf;
    /**
    * Last upper leaning point ( of remainder @a myMu )
    */
    Point myUl;
    /**
    * First lower leaning point ( of remainder @a myMu + @a myOmega - 1 )
    */
    Point myLf;
    /**
    * Last lower leaning point ( of remainder @a myMu + @a myOmega - 1 )
    */
    Point myLl;
    
    //Iterators to the first (at the back) and last (at the front) 
    //points of the DSS  (here because Display2DFactory, todo: accessors)
    /**
    * ConstIterator pointing to the back of the DSS
    */
    ConstIterator myF;
    /**
    * ConstIterator pointing to the front of the DSS
    */
    ConstIterator myL;
    
    /**
     * Projects the point @a m onto the straight line of slope ( @a myA / @a myB) 
     * and intercept @a myMu + ( @a myOmega - 1 )/2 ).
     * @param m any point (may not be part of the DSS).
     * @return the projected point.
     */
    PointD project( const Point & m ) const;

    /**
     * Projects the point @a m onto the straight line whose points have
     * remainder @a r.
     *
     * @param m any point (may not be part of the DSS).
     *
     * @param r the remainder (may not be an integer).
     *
     * @return the projected point.
     */
    PointD project( const Point & m, double r ) const;
    
    /**
     * Projects the point @a m onto the straight line going through point @a p.
     *
     * @param m any point (may not be part of the DSS).
     *
     * @param p any point (may not be part of the DSS).
     *
     * @return the projected point.
     */
    PointD project( const Point & m, const Point & p ) const;
  

    /**
     * @return the projected length of the segment.
     */
    double projectedSegmentLength() const;
    
    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) ; 
    
    
    // ------------------------- Private Datas --------------------------------
  private:



    // --------------- CDrawableWithBoard2D realization --------------------
  public:
    
    /**
     * Default drawing style object.
     * @param mode the drawing mode.
     * @return the dyn. alloc. default style for this object.
     */
    //DrawableWithBoard2D* defaultStyle( std::string mode = "" ) const;
    
    /**
     * @return the style name used for drawing this object.
     */
    std::string className() const;

    
  }; // end of class ArithmeticalDSS


/**
 * Overloads 'operator<<' for displaying objects of class 'ArithmeticalDSS'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'ArithmeticalDSS' to write.
 * @return the output stream after the writing.
 */
template <typename TIterator, typename TInteger, int connectivity>
std::ostream&
operator<< ( std::ostream & out,  ArithmeticalDSS<TIterator,TInteger,connectivity> & object )
{
  object.selfDisplay( out);
  return out;
}


} // namespace DGtal



///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods.
#include "DGtal/geometry/curves/ArithmeticalDSS.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ArithmeticalDSS_h

#undef ArithmeticalDSS_RECURSES
#endif // else defined(ArithmeticalDSS_RECURSES)
