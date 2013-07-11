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
#include "DGtal/base/Exceptions.h"
#include "DGtal/base/ReverseIterator.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/arithmetic/IntegerComputer.h"

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
    typedef TInteger Integer; 

    typedef DGtal::PointVector<2, Coordinate> Point; 
    typedef SpaceND<2, Coordinate> Space; 
    typedef Point Element; 
    typedef Point Vector; 
    typedef std::pair<Vector,Vector> Steps; 


    typedef DGtal::PointVector<2, double> PointD; 

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
    ArithmeticalDSS(const Coordinate& aA, const Coordinate& aB, 
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
     * @param aF the first point
     * @param aL the last point
     * @param aUf the first upper point
     * @param aUl the last upper point
     * @param aLf the first lower point
     * @param aLl the last lower point
     *
     * NB: in O(1)
     */
    ArithmeticalDSS(const Coordinate& aA, const Coordinate& aB,
		    const Point& aF, const Point& aL,
		    const Point& aUf, const Point& aUl,
		    const Point& aLf, const Point& aLl);

    /**
     * Construction of a sequence of patterns 
     * (or reversed patterns) from two end points.
     *
     * @param aF the first point
     * @param aL the last point
     * @param isOnTheUpperLine boolean equal to 'true' if the
     * two given end points are upper leaning points (pattern case, default), 
     * 'false' if they are lower leaning points (reversed pattern case).  
     *
     * NB: logarithmic-time in the greatest component of the vector
     * starting from @a aF and pointing to @a aL
     */
    ArithmeticalDSS(const Point& aF, const Point& aL, 
		    const bool& isOnTheUpperLine = true);

    /**
     * Construction from a range of iterator on points. 
     *
     * @param aItb begin iterator
     * @param aIte end iterator
     * @tparam Iterator a model of forward iterator
     *
     * NB: linear-time in the size of the range
     */
    template <typename Iterator>
    ArithmeticalDSS(const Iterator& aItb, const Iterator& aIte) throw( InputException );

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
    bool operator== ( const ArithmeticalDSS & aOther ) const;

    /**
     * Difference.
     * @param aOther the object to compare with.
     * @return 'true' the two set of points that are
     * implicitly defined are different, 'false' otherwise
     */
    bool operator!= ( const ArithmeticalDSS & aOther ) const;

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
     *
     * NB: in O( log( max( |@a myA |,| @a myB | ) ) )
     */
    bool isValid() const;

    /**
     * Checks the validity of the DSS when it contains only one point.
     * @return 'true' if the DSS is valid, 'false' otherwise.
     * @see isValid
     */
    bool checkOnePoint() const;

    /**
     * Checks the consistency between the parameters and 
     * the leaning points: upper leaning points should
     * have a remainder equal to @a myMu while lower 
     * leaning points should have a remainder equal to 
     * @a myMu + @a myOmega -1
     * @return 'true' if this property is fulfilled, 'false' otherwise.
     * @see isValid
     */
    bool checkLeaningPointsRemainder() const;

    /**
     * Checks whether the difference between two extremal
     * upper (resp. lower) leaning points is equal to the 
     * direction vector ( @a myA, @a myB ) scaled by 
     * an integer. 
     * @param fromFirstToLast vector starting from the 
     * first upper (resp. lower) leaning point and 
     * pointing to the last upper (resp. lower) leaning point. 
     * @return 'true' if this property if fulfilled, 'false' otherwise.
     * @see isValid
     *
     * NB: in logarithmic time
     */
    bool checkLeaningPointsAndSlope(const Vector& fromFirstToLast) const;

    /**
     * Checks the shift vector and the DSS steps, 
     * which depend on the octant/quadrant of the DSS.
     * @return 'true' if this property if fulfilled, 'false' otherwise.
     * @see isValid
     */
    bool checkShiftAndSteps() const;

    /**
     * @return a parameter (y-component of the direction vector)
     */
    Coordinate a() const; 

    /**
     * @return b parameter (x-component of the direction vector)
     */
    Coordinate b() const; 

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
     * Returns the position of @a aPoint
     * (which does not necessarily belong to the DSS)
     * computed along the direction given by @a myShift 
     * @param aPoint the point whose position is returned 
     * @return the position
     */
    Integer directionalPosition(const Point& aPoint) const; 

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

    /**
     * Removes an end point to retract the DSS.  
     *
     * @param aFirst end point to remove, 
     * viewed as the first point of the DSS
     * @param aIt iterator used to get the point
     * following @a aFirst
     * @tparam TIterator a model of forward iterator
     * @param aLast opposite end of the DSS
     * @param aBezout returned weakly exterior point used to 
     * compute the new slope and to update the leaning points
     * @param aLeaningPoint returned leaning point used to 
     * compute the new slope and to update the leaning points
     * @param aFirstUpper first leaning point after @a aFirst
     * @param aLastUpper last leaning point close to @a aLast
     * @param aFirstLower first leaning point after @a aFirst
     * @param aLastLower last leaning point close to @a aLast
     *
     * @return 'true' is the slope has to be updated, 
     * 'false' otherwise
     *
     * @see updateLeaningPoints
     * @see retractForward retractBackward
     */
    template<typename TIterator>
    bool retract( Point& aFirst, 
		  TIterator aIt, 
		  const Point& aLast,
		  Point& aBezout, 
		  Point& aLeaningPoint, 
		  Point& aFirstUpper, 
		  Point& aLastUpper, 
		  Point& aFirstLower,
		  Point& aLastLower);


    /**
     * Removes the front point of the DSS 
     * if it remains strictly more than one point
     *
     * @return 'true' if the retraction has
     * been done, 'false' otherwise
     *
     * @see retract
     */
    bool retractForward();

    /**
     * Removes the back point of the DSS
     * if it remains strictly more than one point
     *
     * @return 'true' if the retraction has
     * been done, 'false' otherwise
     *
     * @see retract
     */
    bool retractBackward();

    // ------------------------- Display services ------------------------------
    /**
     * Projects the point @a aM onto the straight line whose points have
     * remainder @a aR.
     *
     * @param aM any point (may not be part of the DSS).
     * @param aR the remainder (may not be an integer).
     *
     * @return the projected point.
     */
    PointD project( const Point& aM, double aR ) const;
    
    /**
     * Projects the point @a aM onto the straight line going through point @a aP.
     *
     * @param aM any point (may not be part of the DSS).
     * @param aP any point (may not be part of the DSS).
     *
     * @return the projected point.
     */
    PointD project( const Point & aM, const Point & aP ) const;

    /**
     * @return the style name used for drawing this object.
     */
    std::string className() const;

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Returns the bezout vector (u,v) of a given
     * direction vector of slope @a aA / @a aB
     * such that u and @a aB (resp. v and @a aA)
     * have the same sign.  
     * @return bezout vector 
     * @param aA y-component of the direction vector
     * @param aB x-component of the dirention vector
     * @param aR a remainder equal to either 1 or -1
     * 
     * @see setPattern
     *
     * NB: this method uses the extended Euclid's algorithm
     * and runs in logarithmic time. 
     */
    Vector bezoutVector(const Coordinate& aA, 
			const Coordinate& aB, 
			const Coordinate& aR) const; 

    /**
     * Tests whether @a aStep is one of the two steps of the DSS, 
     * stored in @a mySteps. 
     *
     * @param aStep any step to test
     * @pre steps of @a mySteps should not be null
     * @return 'true' if @a aStep is one of the two steps of the DSS,
     * 'false' otherwise.
     *
     * @see isExtendable
     */
    bool isOneOfTheTwoSteps( const Vector& aStep ) const; 


    // ----------------------- Setters -----------------------------------------
    /**
     * Set the DSS slope (ie. @a myA and @a myB parameters) 
     * @param aA new a-parameter
     * @param aB new b-parameter
     * @see extend
     */
    void setSlope(const Coordinate& aA, const Coordinate& aB); 

    /**
     * Set the intercept and the thickness of the DSS
     * (ie. @a myMu and @a myOmega parameters) 
     * @param aMu new mu parameter
     * @param aOmega new omega parameter
     * @see extend
     */
    void setMuOmega(const Integer& aMu, const Integer& aOmega); 

    /**
     * Initializes a pattern from its two end points, which are
     * two leaning points, either located on the upper leaning line
     * or on the lower leaning line. 
     * Besides the leaning points, this method set the shift vector
     * the steps and the slope, but neither the intercept nor the 
     * thickness, which should be set after. 
     *
     * @param aFirstLeaningPoint first point of the DSS
     * @param aFirstLeaningPoint last point of the DSS
     * @param aFirstOppositeLeaningPoint returned first leaning point 
     * located on the opposite leaning line
     * @param aLastOppositeLeaningPoint returned last leaning point 
     * located on the opposite leaning line
     * @param sign integer equal to '1' if the two end points are
     * located on the upper leaning line, '-1' if they are located
     * on the lower leaning line. It gives the remainder of the 
     * shorthest Bezout vector starting from @a aFirstLeaningPoint 
     * and pointing to @a aLastLeaningPoint
     *
     * NB: logarithmic in time. 
     */
    void setPattern(const Point& aFirstLeaningPoint, 
		    const Point& aLastLeaningPoint,
		    Point& aFirstOppositeLeaningPoint, 
		    Point& aLastOppositeLeaningPoint, 
		    const short& sign);
    /**
     * Updates the parameters of the DSS
     * (slope, intercept, thickness, steps, 
     * shift vector) after the retraction. 
     *
     * @param aDirection direction vector
     *
     * @see retractForward retractBackward
     */
    void updateParameters( const Vector& aNewDirection );

    /**
     * Updates the leaning points of the DSS
     * if the end point is a leaning point
     * that has to be removed from the DSS. 
     *
     * @param aDirection direction vector
     * @param aFirst new end of the DSS 
     * @param aLast opposite end of the DSS 
     * @param aBezout weakly exterior point used to 
     * compute the new slope and to update the leaning points
     * @param aFirstAtOppositeSide first leaning point located
     * at the side that is not affected by the removal
     * @param aLastAtOppositeSide last leaning point located
     * at the side that is not affected by the removal but that has 
     * to be updated
     * @param aFirstAtRemovalSide first leaning point that 
     * is removed and has to be updated. 
     * @param aLastAtRemovalSide last leaning point located
     * on the same side than @a aFirstAtRemovalSide 
     *
     * @return 'true' is the slope has to be updated, 
     * 'false' otherwise
     *
     * @see retract
     */
    bool updateLeaningPoints( const Vector& aDirection, 
			      const Point& aFirst,
			      const Point& aLast, 
			      const Point& aBezout, 
			      const Point& aFirstAtOppositeSide, 
			      Point& aLastAtOppositeSide, 
			      Point& aFirstAtRemovalSide,
			      const Point& aLastAtRemovalSide);


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
    Coordinate myA;
    /**
     * x-component of the direction vector
     */
    Coordinate myB;
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
// Aliases
namespace DGtal
{
  /**
   * Description of class 'StandardDSS4' <p> Aim: 
   * represents a sequence of 4-connected digital points
   * contained in a standard DSL. 
   *
   * @tparam  TCoordinate a model of integer for the points coordinates
   * @tparam  TInteger a model of integer for the DSL parameters
   *
   * @see ArithmeticalDSS
   */
  template <typename TCoordinate, typename TInteger = TCoordinate>
  class StandardDSS4: 
    public ArithmeticalDSS<TCoordinate, TInteger, 4>
  {
  public: 
    typedef ArithmeticalDSS<TCoordinate, TInteger, 4> Super; 

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
    StandardDSS4(const typename Super::Coordinate& aA, const typename Super::Coordinate& aB, 
		 const typename Super::Integer& aMu, const typename Super::Integer& aOmega, 
		 const typename Super::Point& aF, const typename Super::Point& aL,
		 const typename Super::Point& aUf, const typename Super::Point& aUl,
		 const typename Super::Point& aLf, const typename Super::Point& aLl);

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
     * @param aF the first point
     * @param aL the last point
     * @param aUf the first upper point
     * @param aUl the last upper point
     * @param aLf the first lower point
     * @param aLl the last lower point
     */
    StandardDSS4(const typename Super::Coordinate& aA, const typename Super::Coordinate& aB,
		 const typename Super::Point& aF, const typename Super::Point& aL,
		 const typename Super::Point& aUf, const typename Super::Point& aUl,
		 const typename Super::Point& aLf, const typename Super::Point& aLl);

    /**
     * Construction of a sequence of patterns 
     * (or reversed patterns) from two end points.
     *
     * @param aF the first point
     * @param aL the last point
     * @param isOnTheUpperLine boolean equal to 'true' if the
     * two given end points are upper leaning points (pattern case, default), 
     * 'false' if they are lower leaning points (reversed pattern case).  
     *
     * NB: logarithmic-time in the greatest component of the vector
     * starting from @a aF and pointing to @a aL
     */
    StandardDSS4(const typename Super::Point& aF, const typename Super::Point& aL,
		 const bool& isOnTheUpperLine = true);

    /**
     * Construction from a range of iterators on points. 
     *
     * @param aItb begin iterator
     * @param aIte end iterator
     * @tparam Iterator a model of forward iterator
     *
     * NB: linear-time in the size of the range
     */
    template <typename Iterator>
    StandardDSS4(const Iterator& aItb, const Iterator& aIte) throw( InputException );

    /**
     * Copy constructor.
     * @param aOther the object to clone.
     */
    StandardDSS4 ( const StandardDSS4 & aOther ); 

    /**
     * Assignment.
     * @param aOther the object to copy.
     * @return a reference on 'this'.
     */
    StandardDSS4 & operator= ( const StandardDSS4 & aOther );

  }; 

  /**
   * Description of class 'NaiveDSS8' <p> Aim: 
   * represents a sequence of 8-connected digital points
   * contained in a naive DSL. 
   *
   * @tparam  TCoordinate a model of integer for the points coordinates
   * @tparam  TInteger a model of integer for the DSL parameters
   *
   * @see ArithmeticalDSS
   */
  template <typename TCoordinate, typename TInteger = TCoordinate>
  class NaiveDSS8: 
    public ArithmeticalDSS<TCoordinate, TInteger, 8>
  {
  public: 
    typedef ArithmeticalDSS<TCoordinate, TInteger, 8> Super; 

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
    NaiveDSS8(const typename Super::Coordinate& aA, const typename Super::Coordinate& aB, 
	      const typename Super::Integer& aMu, const typename Super::Integer& aOmega, 
	      const typename Super::Point& aF, const typename Super::Point& aL,
	      const typename Super::Point& aUf, const typename Super::Point& aUl,
	      const typename Super::Point& aLf, const typename Super::Point& aLl);
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
     * @param aF the first point
     * @param aL the last point
     * @param aUf the first upper point
     * @param aUl the last upper point
     * @param aLf the first lower point
     * @param aLl the last lower point
     */
    NaiveDSS8(const typename Super::Coordinate& aA, const typename Super::Coordinate& aB,
	      const typename Super::Point& aF, const typename Super::Point& aL,
	      const typename Super::Point& aUf, const typename Super::Point& aUl,
	      const typename Super::Point& aLf, const typename Super::Point& aLl);


    /**
     * Construction of a sequence of patterns 
     * (or reversed patterns) from two end points.
     *
     * @param aF the first point
     * @param aL the last point
     * @param isOnTheUpperLine boolean equal to 'true' if the
     * two given end points are upper leaning points (pattern case, default), 
     * 'false' if they are lower leaning points (reversed pattern case).  
     *
     * NB: logarithmic-time in the greatest component of the vector
     * starting from @a aF and pointing to @a aL
     */
    NaiveDSS8(const typename Super::Point& aF, const typename Super::Point& aL,
	      const bool& isOnTheUpperLine = true);

    /**
     * Construction from a range of iterators on points. 
     *
     * @param aItb begin iterator
     * @param aIte end iterator
     * @tparam Iterator a model of forward iterator
     *
     * NB: linear-time in the size of the range
     */
    template <typename Iterator>
    NaiveDSS8(const Iterator& aItb, const Iterator& aIte) throw( InputException );

    /**
     * Copy constructor.
     * @param aOther the object to clone.
     */
    NaiveDSS8 ( const NaiveDSS8 & aOther ); 

    /**
     * Assignment.
     * @param aOther the object to copy.
     * @return a reference on 'this'.
     */
    NaiveDSS8 & operator= ( const NaiveDSS8 & aOther );

  }; 
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/curves/ArithmeticalDSS.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
#endif // !defined ArithmeticalDSS_h

#undef ArithmeticalDSS_RECURSES
#endif // else defined(ArithmeticalDSS_RECURSES)
