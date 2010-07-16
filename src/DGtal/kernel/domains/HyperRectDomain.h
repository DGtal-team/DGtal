#pragma once

/**
 * @file HyperRectDomain.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/05/25
 *
 * Header file for module HyperRectDomain.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(HyperRectDomain_RECURSES)
#error Recursive header files inclusion detected in HyperRectDomain.h
#else // defined(HyperRectDomain_RECURSES)
/** Prevents recursive inclusion of headers. */
#define HyperRectDomain_RECURSES

#if !defined HyperRectDomain_h
/** Prevents repeated inclusion of headers. */
#define HyperRectDomain_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CSpace.h"
#include "DGtal/kernel/BasicPointPredicates.h"
#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/kernel/domains/HyperRectDomain_Iterator.h"
#include "DGtal/kernel/domains/HyperRectDomain_SpanIterator.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class HyperRectDomain
  /**
   * Description of class 'HyperRectDomain' <p> \brief Aim:
   * Parallelepidec region of a digital space, model of a 'CDomain'.
   *
   * The following code snippet demonstrates how to use \p HyperRectDomain
   *
   *  \code
   *  #include <DGtal/kernel/Space.h>
   *  #include <DGtal/kernel/domains/HyperRectDomain.h>
   * ...
   *
   * //We create a digital Space based on 'int' integers and in dimension 4
   * typedef DGtal::Space<int,4> Space4DType;
   * typedef Space4DType::TPoint Point4DType;
   *
   * const int rawA[ ] = { 1, 2, 3 ,4};
   * const int rawB[ ] = { 5, 5, 3 ,4};
   * Point4DType A ( rawA );
   * Point4DType B ( rawB );
   *
   * //Domain construction from two points
   * DGtal::HyperRectDomain<Space4DType> myDomain ( A, B );
   *
   * //We just iterate on the Domain points and print out the point coordinates.
   * std::copy ( myDomain.begin(),
   *             myDomain.end(),
   *             std::ostream_iterator<Point4DType> ( std::cout, " " ) );
   *  \endcode
   *
   *
   * \see testHyperRectDomain.cpp
   * \see testHyperRectDomain-snippet.cpp
   */
  template<typename TSpace>
  class HyperRectDomain
  {
    // ----------------------- Standard services ------------------------------
  public:

    BOOST_CONCEPT_ASSERT(( CSpace<TSpace> ));
    

    // typedef TSpace DigitalSpace;
    // typedef TSpace SpaceType;
    typedef TSpace Space;
    typedef HyperRectDomain<Space> Domain;
    typedef typename Space::Point Point;
    typedef typename Space::Vector Vector;
    typedef typename Space::SizeType SizeType;
    typedef typename Point::Coordinate Coordinate;

    // BOOST_CONCEPT_ASSERT(( CDomain< HyperRectDomain >));

    ///Typedef of domain iterators
    typedef HyperRectDomain_Iterator<Point> ConstIterator;
    typedef HyperRectDomain_SpanIterator<Point> ConstSpanIterator;

    typedef IsWithinPointPredicate<Point> Predicate;

    /**
     * Default Constructor.
     */
    HyperRectDomain();

    /**
     * Constructor from  two points \param aPointA and \param aPoint B
     * defining the space diagonal.
     *
     */
    HyperRectDomain ( const Point &aPointA, const Point &aPointB );


    /**
     * Destructor.
     */
    ~HyperRectDomain();

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    HyperRectDomain ( const HyperRectDomain & other );
    
    
    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    HyperRectDomain & operator= ( const HyperRectDomain & other );


    
    //------------- Global Iterator
    /**
     * begin() iterator.
     *
     **/
    const ConstIterator& begin() const;

#ifdef CPP0X_INITIALIZER_LIST
     /**
     * begin() iterator with an order different from lexicographic.
     *
     **/
    ConstIterator begin(std::initializer_list<unsigned int> permutation);
#endif

    /**
     * begin(aPoint) iterator. Returns an iterator starting at \param aPoint
     *
     **/
    ConstIterator begin ( const Point &aPoint ) const;

    /**
     * end() iterator.
     *
     **/
    const ConstIterator& end() const;

#ifdef CPP0X_INITIALIZER_LIST
     /**
     * end() iterator with an order different from lexicographic.
     *
     **/
    ConstIterator end(std::initializer_list<unsigned int> permutation);
#endif

    //------------- Span Iterator
    /**
     * Returns a Span iterator starting at \param aPoint and moving toward the dimension \param aDimension.
     *
     **/
    ConstSpanIterator span_begin ( const Point &aPoint, const std::size_t aDimension) const;


    /**
     * Creates a end() Span iterator along the dimension \param aDimension.
     *
     **/
    ConstSpanIterator span_end (const std::size_t aDimension) const;



    // ----------------------- Interface --------------------------------------
  public:

    /**
     * @return  the extent of the HyperRectDomain
     *
     **/
    std::size_t extent() const;


    /** 
     * Returns the extent of the HyperRectDomain (static method).
     * 
     * @param aLowerBound  a Point corresponding to the lower bound.
     * @param aUpperBound a Point corresponding to the upper bound.
     * 
     * @return the extent.
     */    
    static std::size_t extent(const Point &aLowerBound, const Point &aUpperBound)
    {
      std::size_t val = 1;
      for (unsigned int k =  0; k < TSpace::staticDimension ; k++)
	val *= (aUpperBound.at(k) - aLowerBound.at(k) + 1);

      return val;
    }


    /**
     * Returns the lowest point of the space diagonal.
     *
     **/
    const Point &lowerBound() const;

    /**
     * Returns the highest point of the space diagonal.
     *
     **/
    const Point &upperBound() const ;

    /**
     * @param p any point.
     * @return 'true' if point [p] is inside this domain.
     */
    bool isInside( const Point & p ) const;

    /**
     * @return a const reference to the "IsInside" predicate.
     * 
     * NB: Could have used template class DomainPredicate but, for
     * performance reason, directly used the IsWithinPointPredicate
     * which fits perfectly.
     */
    const Predicate & predicate() const;

     /**
     * Draw the object (as a Point) on a LiBoard board
     * @param board the output board where the object is drawn.
     * @param asGrid to choose between paving vs. grid representation.
     */
    void selfDraw ( LibBoard::Board & board, bool asGrid=true) const;

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

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:


    // ------------------------- Hidden services ------------------------------
  private:


    ///The lowest point of the space diagonal
    Point myLowerBound;
    ///The highest point of the space diagonal
    Point myUpperBound;

    /// "IsInside" predicate.
    Predicate myPredicate;

    /// Begin iterator
    ConstIterator myIteratorBegin;

    /// End iterator
    ConstIterator myIteratorEnd;
  }; // end of class HyperRectDomain


  /**
   * Overloads 'operator<<' for displaying objects of class 'HyperRectDomain'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'HyperRectDomain' to write.
   * @return the output stream after the writing.
   */
  template<typename TSpace>
  std::ostream&
  operator<< ( std::ostream & out, const HyperRectDomain<TSpace> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/domains/HyperRectDomain.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined HyperRectDomain_h

#undef HyperRectDomain_RECURSES
#endif // else defined(HyperRectDomain_RECURSES)
