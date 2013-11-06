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
 * @file ArithmeticalDSL.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/06/28
 *
 * Header file for module ArithmeticalDSL.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ArithmeticalDSL_RECURSES)
#error Recursive header files inclusion detected in ArithmeticalDSL.h
#else // defined(ArithmeticalDSL_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ArithmeticalDSL_RECURSES

#if !defined ArithmeticalDSL_h
/** Prevents repeated inclusion of headers. */
#define ArithmeticalDSL_h

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

#include "DGtal/geometry/curves/ArithmeticalDSLKernel.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  //forward declaration of ArithmeticalDSS for the friendship
  template <typename TCoordinate, 
	    typename TInteger, 
	    unsigned short adjacency>
  class ArithmeticalDSS; 

  /////////////////////////////////////////////////////////////////////////////
  // template class ArithmeticalDSL
  /**
   * Description of template class 'ArithmeticalDSL' <p>
   * \brief Aim: This class represents a naive (resp. standard) 
   * digital straight line (DSL), ie. a set of digital points
   * \f$ (x,y) \in \mathbb{Z}^2 \f$ such that 
   * \f$ \mu <= ax - by < \mu + \omega \f$ 
   * with \f$ \gcd(a,b) = 1 \f$ and \f$ \omega = \max(|a|,|b|) \f$
   * (resp. \f$ \omega = |a| + |b| \f$). 
   * 
   * @tparam TCoordinate a model of integer for the DGtal point coordinates
   * @tparam TInteger a model of integer for the DSL parameters mu and omega
   * @tparam adajency a integer equal to 8 (default) for naive DSL, 
   * and 4 for standard DSL. 
   */
  template <typename TCoordinate, 
	    typename TInteger = TCoordinate, 
	    unsigned short adjacency = 8>
  class ArithmeticalDSL
  {

    // ----------------------- Friend class -----------------------------------
  public:
    friend class ArithmeticalDSS<TCoordinate, TInteger, adjacency>; 

    // ----------------------- Inner types -----------------------------------
  public:

    typedef TCoordinate Coordinate; 
    typedef TInteger Integer; 

    typedef DGtal::PointVector<2, Coordinate> Point; 
    typedef SpaceND<2, Coordinate> Space; 
    typedef Point Element; 
    typedef Point Vector; 

    typedef std::pair<Vector,Vector> Steps; 


    /**
     * \brief Aim: This class aims at representing an iterator
     * that provides a way to scan the points of a DSL
     * It is both a model of readable iterator and of
     * bidirectional iterator. 
     */
    class ConstIterator : public 
    std::iterator<std::bidirectional_iterator_tag, 
		  Point, int, Point*, Point> 
    {
      // ------------------------- Private data -----------------------
    private:

      /// Constant aliasing pointer to the DSL visited by the iterator
      const ArithmeticalDSL* myDSLPtr;

      /// The current point
      Point  myCurrentPoint;

      /// Quantity to add to the current remainder
      Integer  myQuantityToAdd;

      /// Quantity to remove to the current remainder
      Integer  myQuantityToRemove;

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
       * @param aDSL an arithmetical DSL
       * @param aPoint a point of the DSL containing @a aDSL
       */
      ConstIterator( const ArithmeticalDSL* aDSL, const Point& aPoint ); 

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
       * Moves @a myCurrentPoint to the next point of the DSL
       */
      void next(); 

      /**
       * Pre-increment.
       * Goes to the next point of the DSL.
       */
      ConstIterator& operator++(); 

      /**
       * Post-increment.
       * Goes to the next point of the DSL.
       */
      ConstIterator operator++(int); 

      /**
       * Moves @a myCurrentPoint to the previous point of the DSL
       */
      void previous();

      /**
       * Pre-decrement.
       * Goes to the previous point in the DSL.
       */
      ConstIterator& operator--();

      /**
       * Post-decrement.
       * Goes to the previous point in the DSL.
       */
      ConstIterator operator--(int);

      /**
       * Equality operator.
       *
       * @param aOther the iterator to compare with 
       * (must be defined on the same DSL).
       *
       * @return 'true' if their current points coincide.
       */
      bool operator== ( const ConstIterator & aOther ) const;

      /**
       * Inequality operator.
       *
       * @param aOther the iterator to compare with 
       * (must be defined on the same DSL).
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
     * should be sure that the resulting DSL is valid. 
     *
     * @see isValid
     *
     * @param aA y-component of the direction vector
     * @param aB x-component of the direction vector
     * @param aLowerIntercept lower intercept 
     * @param aUpperIntercept upper intercept
     * @param aSteps pair of steps used to iterate over the DSL points
     * @param aShift shift vector translating a point of remainder r 
     * to a point of remainder r +  @a aOmega
     */
    ArithmeticalDSL(const Coordinate& aA, const Coordinate& aB, 
		    const Integer& aLowerIntercept, const Integer& aUpperIntercept, 
		    const Steps& aSteps, const Vector& aShift);

    /**
     * Constructor with the minimal set of parameters.
     *
     * @param aA y-component of the direction vector
     * @param aB x-component of the direction vector
     * @param aMu intercept
     */
    ArithmeticalDSL(const Coordinate& aA, const Coordinate& aB,
		    const Integer& aMu);


    /**
     * Copy constructor.
     * @param aOther the object to clone.
     */
    ArithmeticalDSL ( const ArithmeticalDSL & aOther );

    /**
     * Assignment.
     * @param aOther the object to copy.
     * @return a reference on 'this'.
     */
    ArithmeticalDSL& operator= ( const ArithmeticalDSL & aOther );

    /**
     * Returns a copy of '*this' with a reverse orientation, ie. 
     * with parameters @a -myA ,  @a -myB ,  
     * @a -myUpperIntercept , @a -myLowerIntercept.
     * @return the negation of '*this'.
     */
    ArithmeticalDSL negate () const;

    /**
     * Equality.
     * @param aOther the object to compare with.
     * @return 'true' if the parameters of the DSL
     * are equal (orientation matters), 'false' otherwise
     */
    bool equalsTo ( const ArithmeticalDSL & aOther ) const;

    /**
     * Equality.
     * @param aOther the object to compare with.
     * @return 'true' the two sets of points that are
     * implicitly defined are equal (without respect to 
     * the orientation), 'false' otherwise
     * @see equalsTo
     */
    bool operator== ( const ArithmeticalDSL & aOther ) const;

    /**
     * Difference.
     * @param aOther the object to compare with.
     * @return 'true' the two set of points that are
     * implicitly defined are different, 'false' otherwise
     */
    bool operator!= ( const ArithmeticalDSL & aOther ) const;

    /**
     * Destructor.
     */
    ~ArithmeticalDSL();

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the consistency between the slope parameters, ie. 
     * @a myA , @a myB and the translating vectors @a myShift , 
     * @a mySteps  
     */
    bool checkShiftAndSteps() const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     *
     * NB: runs in O( log( max( |@a myA |,| @a myB | ) ) )
     * because it checks if @a myA and @a myB are relatively primes. 
     */
    bool isValid() const;

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
     * @return the shift vector translating a point
     * of remainder r to a point of remainder r+omega
     */
    Vector shift() const; 
    /**
     * @return the two vectors used to iterate
     * over the DSL point. 
     */
    Steps steps() const; 

    /**
     * Returns the remainder of @a aPoint
     * with respect to the direction vector
     * of component @a aA and @a aB
     * @param aA y-component of the direction vector
     * @param aB x-component of the direction vector
     * @param aPoint any point
     * @return remainder of @a aPoint
     */
    static Integer remainder(const Integer& aA, 
			     const Integer& aB, 
			     const Point& aPoint); 

    /**
     * Returns the remainder of @a aPoint
     * (which does not necessarily belong to the DSL)
     * @return remainder of @a aPoint
     * @param aPoint any point
     */
    Integer remainder(const Point& aPoint) const; 

    /**
     * Returns the position of @a aPoint
     * (which does not necessarily belong to the DSL)
     * computed along the direction that is orthogonal 
     * to the direction vector
     * @param aPoint the point whose position is returned 
     * @return the position
     */
    Integer orthogonalPosition(const Point& aPoint) const; 

    /**
     * Returns the position of @a aPoint
     * (which does not necessarily belong to the DSL)
     * computed along the direction orthogonal to @a myShift 
     * @param aPoint the point whose position is returned 
     * @return the position
     */
    Integer directionalPosition(const Point& aPoint) const; 

    /**
     * Returns a boolean equal to 'true' if @a aP1 is 
     * located (strictly) before @a aP2 in the direction 
     * orthogonal to @a myShift, 'false' otherwise. 
     * @param aP1 any point
     * @param aP2 any point
     * @return 'true' is @a aP2 is strictly before @a aP2, 
     * 'false' otherwise
     * @see beforeOrEqual directionalPosition
     */
    bool before (const Point& aP1, const Point& aP2) const; 

    /**
     * Returns a boolean equal to 'true' if @a aP1 is 
     * located before @a aP2 or is equal to @a aP2, 
     * 'false' otherwise. 
     * @param aP1 any point
     * @param aP2 any point
     * @return 'true' is @a aP2 is before or equal to @a aP2, 
     * 'false' otherwise
     * @see before directionalPosition
     */
    bool beforeOrEqual (const Point& aP1, const Point& aP2) const; 

    /**
     * @return 'true' if @a aPoint is in the DSL
     * 'false' otherwise. 
     * @param aPoint any point
     */
    bool isInDSL(const Point& aPoint) const; 

    /**
     * @return 'true' if @a aPoint is in the DSL
     * 'false' otherwise. 
     * @param aPoint any point
     * @see isInDSL
     */
    bool operator()(const Point& aPoint) const; 


    // ----------------------- Iterator services -------------------------------
    /**
     * @param aPoint any point of the DSL
     * @pre aPoint sould belongs to the DSL
     * @return begin iterator,
     * which points to @a aPoint 
     */
    ConstIterator begin(const Point& aPoint) const; 

    /**
     * @param aPoint any point of the DSL
     * @pre aPoint sould belongs to the DSL
     * @return end iterator,
     * which points to the point located after @a aPoint 
     */
    ConstIterator end(const Point& aPoint) const; 

    /**
     * @param aPoint any point of the DSL
     * @pre aPoint sould belongs to the DSL
     * @return begin reverse iterator,
     * which points to @a aPoint  
     */
    ConstReverseIterator rbegin(const Point& aPoint) const; 

    /**
     * @param aPoint any point of the DSL
     * @pre aPoint sould belongs to the DSL
     * @return end iterator, 
     * which points to the point located before @a aPoint
     */
    ConstReverseIterator rend(const Point& aPoint) const; 


    // ------------------------- Protected Datas ------------------------------
  protected:

    // -------------------- steps ---------------------------------------------
    /**
     * Pair of steps used to iterate over the DSL points
     * NB: the first one is always the steps starting from an upper leaning point
     * of remainder @a myLowerBound 
     * NB: the second one can be equal to (0,0) in the degenerated case, ie. when 
     * - @a myA or @a myB is equal to 0, 
     * - @a myA == @a myB in the naive case. 
     * In the other cases, the first step minus the second step is always equal to 
     * vector @a myShift
     */
    Steps mySteps;
    /**
     * Shift vector (translating a point of remainder r to a point of remainder r+omega) 
     */
    Vector myShift;

    //------------------------ parameters of the DSL --------------------------
    /**
     * y-component of the direction vector
     */
    Coordinate myA;
    /**
     * x-component of the direction vector
     */
    Coordinate myB;
    /**
     * Lower intercept
     * NB: for any upper leaning point U, we have remainder(U) == myLowerBound.
     * NB: myLowerBound <= myUpperBound
     */
    Integer myLowerBound;
    /**
     * Upper intercept
     * NB: for any lower leaning point L, we have remainder(L) == myUpperBound.
     * NB: if myA and myB are not both null, myUpperBound == myLowerBound + omega() - 1
     * NB: myLowerBound <= myUpperBound
     */
    Integer myUpperBound;



  }; // end of class ArithmeticalDSL


  /**
   * Overloads 'operator<<' for displaying objects of class 'ArithmeticalDSL'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ArithmeticalDSL' to write.
   * @return the output stream after the writing.
   */
  template <typename TCoordinate, typename TInteger, unsigned short adjacency>
  std::ostream&
  operator<< ( std::ostream & out, const ArithmeticalDSL<TCoordinate, TInteger, adjacency> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Aliases
namespace DGtal
{
  /**
   * Description of class 'StandardDSL4' <p> Aim: 
   * represents standard DSL. 
   *
   * @tparam  TCoordinate a model of integer for the points coordinates
   * @tparam  TInteger a model of integer for the DSL parameters
   *
   * @see ArithmeticalDSL
   */
  template <typename TCoordinate, typename TInteger = TCoordinate>
  class StandardDSL: 
    public ArithmeticalDSL<TCoordinate, TInteger, 4>
  {
  public: 
    typedef ArithmeticalDSL<TCoordinate, TInteger, 4> Super; 

  public: 

    /**
     * Constructor.
     *
     * @param aA y-component of the direction vector
     * @param aB x-component of the direction vector
     * @param aMu intercept
     */
    StandardDSL (const typename Super::Coordinate& aA, 
		 const typename Super::Coordinate& aB,
		 const typename Super::Integer& aMu);

    /**
     * Copy constructor.
     * @param aOther the object to clone.
     */
    StandardDSL ( const StandardDSL & aOther ); 

    /**
     * Assignment.
     * @param aOther the object to copy.
     * @return a reference on 'this'.
     */
    StandardDSL & operator= ( const StandardDSL & aOther );

  }; 

  /**
   * Description of class 'NaiveDSL' <p> Aim: 
   * represents a naive DSL. 
   *
   * @tparam  TCoordinate a model of integer for the points coordinates
   * @tparam  TInteger a model of integer for the DSL parameters
   *
   * @see ArithmeticalDSL
   */
  template <typename TCoordinate, typename TInteger = TCoordinate>
  class NaiveDSL: 
    public ArithmeticalDSL<TCoordinate, TInteger, 8>
  {
  public: 
    typedef ArithmeticalDSL<TCoordinate, TInteger, 8> Super; 

  public: 
    /**
     * Constructor.
     *
     * @param aA y-component of the direction vector
     * @param aB x-component of the direction vector
     * @param aMu intercept
     */
    NaiveDSL (const typename Super::Coordinate& aA, 
	      const typename Super::Coordinate& aB,
	      const typename Super::Integer& aMu);


    /**
     * Copy constructor.
     * @param aOther the object to clone.
     */
    NaiveDSL ( const NaiveDSL & aOther ); 

    /**
     * Assignment.
     * @param aOther the object to copy.
     * @return a reference on 'this'.
     */
    NaiveDSL & operator= ( const NaiveDSL & aOther );

  }; 
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/curves/ArithmeticalDSL.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
#endif // !defined ArithmeticalDSL_h

#undef ArithmeticalDSL_RECURSES
#endif // else defined(ArithmeticalDSL_RECURSES)
