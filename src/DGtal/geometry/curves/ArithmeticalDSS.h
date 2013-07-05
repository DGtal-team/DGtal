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
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/06/28
 *
 * Header file for module ArithmeticalDSS.cpp
 *
 * This file is part of the DGtal library.
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
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/arithmetic/IntegerComputer.h"
#include "DGtal/base/ReverseIterator.h"

#include "DGtal/geometry/curves/ArithmeticalDSSKernel.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ArithmeticalDSS
  /**
   * Description of template class 'ArithmeticalDSS' <p>
   * \brief Aim: This class represents a naive (resp. standard) 
   * digital straight segment (DSS), ie. the sequence of 
   * simply 8- (resp. 4-)connected digital points between @a myF and @a myL
   * and contained in the naive (resp. standard) digital straight
   * line (DSL) of parameters @a myA @a myB @a myMu @a myOmega.  
   *
   * 
   * @tparam TCoordinate a model of integer for the DGtal point coordinate
   * @tparam TInteger a model of integer for the DSS parameters (a, b, mu, omega)
   * @tparam adajency a integer equal to 8 (default) for naive and 8-connected DSS, 
   * and 4 for standard and 4-connected DSS. 
   */
  template <typename TCoordinate, 
	    typename TInteger = TCoordinate, 
	    unsigned short adjacency = 8>
  class ArithmeticalDSS
  {

    // ----------------------- Inner types -----------------------------------
  public:

    typedef TCoordinate Coordinate; 
    typedef DGtal::PointVector<2, Coordinate> Point; 
    typedef DGtal::PointVector<2, Coordinate> Vector; 
    typedef std::pair<Vector,Vector> Steps; 
    typedef TInteger Integer; 

  /**
   * \brief Aim: This class aims at representing an iterator
   * that provides a way to scan the points of a DSS
   * It is both a model of readable iterator and of
   * bidirectional iterator. 
   */
  class ConstIterator : public 
  std::iterator<std::bidirectional_iterator_tag, 
		Point, int, Point*, Point> 
  {
    // ------------------------- Private data -----------------------
  private:

    /// Constant aliasing pointer to the DSS visited by the iterator
    const ArithmeticalDSS* myDSSPtr;

    /// The current point
    Point  myCurrentPoint;

    /// Quantity to add to the current remainder
    Integer  myQuantityToAdd;

    /// Quantity to remove to the current remainder
    Integer  myQuantityToRemove;

    /// Upper bound on the remainders
    Integer  myUpperBound;
    /// Lower bound on the remainders
    Integer  myLowerBound;

    /// Remainder of the current point
    Integer  myCurrentRemainder;

    // ------------------------- Standard services -----------------------
  public:

    /**
     * Default constructor (not valid).
     */
    ConstIterator();

    /**
     * Constructor.
     * @param aDSS an arithmetical DSS
     * @param aPoint a point of the DSL containing @a aDSS
     */
    ConstIterator( const ArithmeticalDSS* aDSS, const Point& aPoint ); 

    /**
     * Copy constructor.
     * @param aOther the iterator to clone.
     */
    ConstIterator( const ConstIterator & aOther );
    /**
     * Assignment.
     * @param aOther the iterator to copy.
     * @return a reference on 'this'.
     */
    ConstIterator& operator= ( const ConstIterator & aOther );

    /**
     * Destructor. Does nothing.
     */
    ~ConstIterator(); 

    // ------------------------- iteration services -------------------------
  public:

    /**
     * @return the current point
     */
    Point operator*() const;

    /**
     * Moves @a myCurrentPoint to the next point of the DSS
     */
    void next(); 

    /**
     * Pre-increment.
     * Goes to the next point of the DSS.
     */
    ConstIterator& operator++(); 

    /**
     * Post-increment.
     * Goes to the next point of the DSS.
     */
    ConstIterator operator++(int); 

    /**
     * Moves @a myCurrentPoint to the previous point of the DSS
     */
    void previous();

    /**
     * Pre-decrement.
     * Goes to the previous point in the DSS.
     */
    ConstIterator& operator--();

    /**
     * Post-decrement.
     * Goes to the previous point in the DSS.
     */
    ConstIterator operator--(int);

    /**
     * Equality operator.
     *
     * @param aOther the iterator to compare with 
     * (must be defined on the same DSS).
     *
     * @return 'true' if their current points coincide.
     */
    bool operator== ( const ConstIterator & aOther ) const;

    /**
     * Inequality operator.
     *
     * @param aOther the iterator to compare with 
     * (must be defined on the same DSS).
     *
     * @return 'true' if their current points differ.
     */
    bool operator!= ( const ConstIterator & aOther ) const;

  }; //end of inner class ConstIterator
    
    typedef DGtal::ReverseIterator<ConstIterator> ConstReverseIterator; 

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor.
     * The user gives all the (redondant) parameters and 
     * should be sure that the resulting DSS is valid. 
     *
     * @param aA y-component of the direction vector
     * @param aB x-component of the direction vector
     * @param aMu intercept
     * @param aOmega thickness
     * @param aF the first point
     * @param aL the last point
     * @param aUf the first upper point
     * @param aUl the last upper point
     * @param aLf the first lower point
     * @param aLl the last lower point
     */
    ArithmeticalDSS(const Integer& aA, const Integer& aB, 
		    const Integer& aMu, const Integer& aOmega, 
		    const Point& aF, const Point& aL,
		    const Point& aUf, const Point& aUl,
		    const Point& aLf, const Point& aLl);

    /**
     * Constructor.
     * Minimal set of parameters to build the DSS
     * in constant time. 
     * The user should be sure that the slope is
     * consistent with the position of the leaning 
     * points. 
     *
     * @param aA y-component of the direction vector
     * @param aB x-component of the direction vector
     * @param aMu intercept
     * @param aF the first point
     * @param aL the last point
     * @param aUf the first upper point
     * @param aUl the last upper point
     * @param aLf the first lower point
     * @param aLl the last lower point
     */
    ArithmeticalDSS(const Integer& aA, const Integer& aB,
		    const Point& aF, const Point& aL,
		    const Point& aUf, const Point& aUl,
		    const Point& aLf, const Point& aLl);

    /**
     * Constructor of a pattern from its two end points.
     *
     * @param aF the first point
     * @param aL the last point
     */
    ArithmeticalDSS(const Point& aF, const Point& aL);

    /**
     * Copy constructor.
     * @param aOther the object to clone.
     */
    ArithmeticalDSS ( const ArithmeticalDSS & aOther );

    /**
     * Assignment.
     * @param aOther the object to copy.
     * @return a reference on 'this'.
     */
    ArithmeticalDSS & operator= ( const ArithmeticalDSS & aOther );

    /**
     * Equality.
     * @param aOther the object to compare with.
     * @return 'true' the two set of points that are
     * implicitly defined are equal, 'false' otherwise
     */
    bool operator== ( const ArithmeticalDSS & aOther );

    /**
     * Difference.
     * @param aOther the object to compare with.
     * @return 'true' the two set of points that are
     * implicitly defined are different, 'false' otherwise
     */
    bool operator!= ( const ArithmeticalDSS & aOther );

    /**
     * Destructor.
     */
    ~ArithmeticalDSS();

    // ----------------------- Interface --------------------------------------
  public:

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
     * @return a parameter (y-component of the direction vector)
     */
    Integer a() const; 

    /**
     * @return b parameter (x-component of the direction vector)
     */
    Integer b() const; 

    /**
     * @return mu parameter, the intercept
     */
    Integer mu() const; 

    /**
     * @return omega parameter, the thickness
     */
    Integer omega() const; 

    /**
     * @return first point of the DSS
     */
    Point back() const; 

    /**
     * @return last point of the DSS
     */
    Point front() const; 

    /**
     * @return first upper leaning point of the DSS
     */
    Point Uf() const; 

    /**
     * @return last upper leaning point of the DSS
     */
    Point Ul() const; 

    /**
     * @return first lower leaning point of the DSS
     */
    Point Lf() const; 

    /**
     * @return last lower leaning point of the DSS
     */
    Point Ll() const; 

    /**
     * @return the shift vector translating a point
     * of remainder r to a point of remainder r+omega
     */
    Vector shift() const; 
    /**
     * @return the two vectors used to iterate
     * over the DSS point. 
     */
    Steps steps() const; 

    /**
     * Returns the remainder of @a aPoint
     * (which does not necessarily belong to the DSS)
     * @return remainder of @a aPoint
     * @param aPoint any point
     */
    Integer remainder(const Point& aPoint) const; 

    /**
     * Returns the position of @a aPoint
     * (which does not necessarily belong to the DSS)
     * computed along the direction that is orthogonal 
     * to the direction vector
     * @param aPoint the point whose position is returned 
     * @return the position
     */
    Integer orthogonalPosition(const Point& aPoint) const; 

    /**
     * @return 'true' if @a aPoint is in the DSL
     * of minimal parameters containing (*this), 
     * 'false' otherwise. 
     * @param aPoint any point
     */
    bool isInDSL(const Point& aPoint) const; 

    /**
     * @return 'true' if @a aPoint is in the DSS
     * 'false' otherwise. 
     * @param aPoint any point
     */
    bool isInDSS(const Point& aPoint) const; 

    /**
     * @return 'true' if @a aPoint is in the DSS
     * 'false' otherwise. 
     * @param aPoint any point
     * @see isInDSS
     */
    bool operator()(const Point& aPoint) const; 

    // ----------------------- Iterator services -------------------------------
    /**
     * @return begin iterator,
     * which points to the first point returned by back() 
     */
    ConstIterator begin() const; 

    /**
     * @return end iterator, 
     * which points after the last point returned by front() 
     */
    ConstIterator end() const; 

    /**
     * @return begin reverse iterator,
     * which points to the last point returned by front() 
     */
    ConstReverseIterator rbegin() const; 

    /**
     * @return end iterator, 
     * which points before the first point returned by back() 
     */
    ConstReverseIterator rend() const; 


    // ----------------------- Setters -----------------------------------------
    /**
     * Set the DSS slope (ie. @a myA and @a myB parameters) 
     * @param aA new a-parameter
     * @param aB new b-parameter
     * @see extend
     */
    void setSlope(const Integer& aA, const Integer& aB); 

    /**
     * Set the intercept and the thickness of the DSS
     * (ie. @a myMu and @a myOmega parameters) 
     * @param aMu new mu parameter
     * @param aOmega new omega parameter
     * @see extend
     */
    void setMuOmega(const Integer& aMu, const Integer& aOmega); 

    // ----------------------- Dynamic methods --------------------------------

    /**
     * Tests whether the union between a point 
     * and the DSS is a DSS. 
     *
     * @param aNewPoint the point to test
     * @param aStep vector between @a aNewPoint and 
     * @a aEndPoint oriented from back to front. 
     * @param aEndPoint end point of the DSS next to which 
     * @a aNewPoint should be 
     * 
     * @return an integer equal to 0 if the union between 
     * aNewPoint and *this is not a DSS, but strictly greater 
     * than 0 otherwise. The value gives the way of updating
     *  the members of the DSS: 
     * 1: initialization of the first step
     * 2: repetition of the first step
     * 3: initialization of the second step on the left
     * 4: initialization of the second step on the right
     * 5: weakly interior on the left
     * 6: weakly interior on the right
     * 7: weakly exterior on the left
     * 8: weakly exterior on the right
     * 9: strongly interior
     * @see extend
     */
    unsigned short int isExtendable( const Point& aNewPoint,
				     const Vector& aStep, 
				     const Point& aEndPoint ) const;

    /**
     * Tests whether the union between a point 
     * and the DSS is a DSS. 
     * Computes the parameters of the new DSS 
     * with the adding point if true.
     *
     * @param aNewPoint the point to add
     * @param aStep vector between @a aNewPoint and 
     * @a aEndPoint oriented from back to front. 
     * @param aEndPoint last point of the DSS next to which 
     * @a aNewPoint should be 
     * @param aFirstLeft first leaning point located on the left side of the DSS
     * @param aLastLeft last leaning point located on the left side of the DSS  
     * @param aFirstRight first leaning point located on the right side of the DSS
     * @param aLastRight last leaning point located on the right side of the DSS 
     * 
     * @return 'true' if the union is a DSS, 'false' otherwise.
     * @see isExtendable
     */
    bool extend( const Point& aNewPoint, const Vector& aStep, 
		 Point& aEndPoint, 
		 Point& aFirstLeft,  Point& aLastLeft,
		 Point& aFirstRight,  Point& aLastRight );

    /**
     * Tests whether the union between a point, 
     * which is located at the front of the DSS,
     * and the DSS is a DSS. 
     * Computes the parameters of the new DSS 
     * with the adding point if true.
     *
     * @param aNewPoint the point to add
     * 
     * @return 'true' if the union is a DSS, 'false' otherwise.
     * @see extend
     */
    bool extendForward( const Point& aNewPoint );
    /**
     * Tests whether the union between a point, 
     * which is located at the back of the DSS,
     * and the DSS is a DSS. 
     * Computes the parameters of the new DSS 
     * with the adding point if true.
     *
     * @param aNewPoint the point to add
     * 
     * @return 'true' if the union is a DSS, 'false' otherwise.
     * @see extend
     */
    bool extendBackward( const Point& aNewPoint );

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Tests whether @a aStep is one of the two steps of the DSS, 
     * stored in @a mySteps. 
     *
     * @param aStep any step to test
     * @pre steps of @a mySteps should not be null
     * @return 'true' if @a aStep is one of the two steps of the DSS,
     * 'false' otherwise.
     * @see isExtendable
     */
    bool isOneOfTheTwoSteps( const Vector& aStep ) const; 

    // ------------------------- Protected Datas ------------------------------
  protected:

    // -------------------- first and last point, leaning points ---------------
    /**
    * First point
    */
    Point myF;
    /**
    * Last point 
    */
    Point myL;
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

    // -------------------- steps ---------------------------------------------
    /**
    * Pair of steps used to iterate over the DSS points
    */
    Steps mySteps;
    /**
    * Shift vector (translating a point of remainder r to a point of remainder r+omega 
    */
    Vector myShift;

    //------------------------ parameters of the DSS --------------------------
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



  }; // end of class ArithmeticalDSS


  /**
   * Overloads 'operator<<' for displaying objects of class 'ArithmeticalDSS'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ArithmeticalDSS' to write.
   * @return the output stream after the writing.
   */
  template <typename TCoordinate, typename TInteger, unsigned short adjacency>
  std::ostream&
  operator<< ( std::ostream & out, const ArithmeticalDSS<TCoordinate, TInteger, adjacency> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/curves/ArithmeticalDSS.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ArithmeticalDSS_h

#undef ArithmeticalDSS_RECURSES
#endif // else defined(ArithmeticalDSS_RECURSES)
