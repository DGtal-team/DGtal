#pragma once

/**
 * @file DomainAdjacency.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/10
 *
 * Header file for module DomainAdjacency.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DomainAdjacency_RECURSES)
#error Recursive header files inclusion detected in DomainAdjacency.h
#else // defined(DomainAdjacency_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DomainAdjacency_RECURSES

#if !defined DomainAdjacency_h
/** Prevents repeated inclusion of headers. */
#define DomainAdjacency_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/domains/DomainPredicate.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class DomainAdjacency
  /**
   * Description of template class 'DomainAdjacency' <p> \brief Aim:
   * Given a domain and an adjacency, limits the given adjacency to
   * the specified domain for all adjacency and neighborhood
   * computations.
   *
   * This class is useful for limiting adjacencies that are defined
   * for unlimited spaces.
   *
   * @tparam TDomain the type of the domain.
   * @tparam TAdjacency the type of the adjacency.
   */
  template <typename TDomain, typename TAdjacency>
  class DomainAdjacency
  {
  public:
    typedef TDomain Domain;
    typedef TAdjacency Adjacency;
    typedef typename Domain::Point Point;
    typedef DomainPredicate< Domain > DomainPredicateType;
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor.
     */
    DomainAdjacency( const TDomain & domain, const TAdjacency & adjacency );

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    DomainAdjacency( const DomainAdjacency & other );

    /**
     * Destructor.
     */
    ~DomainAdjacency();

    // ----------------------- Adjacency services -----------------------------
  public:

    /**
     * @param p1 any point in this space.
     * @param p2 any point in this space.
     *
     * @return 'true' iff p1 is adjacent to p2 according to this
     * adjacency relation.
     */
    bool isAdjacentTo( const Point & p1, const Point & p2 ) const; 

    /**
     * @param p1 any point in this space.
     * @param p2 any point in this space.
     *
     * @return 'true' iff p1 is adjacent to p2 according to this
     * adjacency relation and p1 != p2.
     */
    bool isProperlyAdjacentTo( const Point & p1, const Point & p2 ) const; 

    /**
     * Outputs the whole neighborhood of point [p] satisfying the
     * given predicate as a sequence of *out_it++ = ...
     *
     * @tparam OutputIterator any output iterator (like
     * std::back_insert_iterator< std::vector<int> >).
     *
     * @tparam PointPredicate any predicate object.
     *
     * @param p any point of this space.
     * @param out_it any output iterator.
     * @param pred the predicate.
     */
    template <typename OutputIterator, 
	      typename PointPredicate>
    void writeNeighborhood( const Point & p, 
			    OutputIterator & out_it,
			    const PointPredicate & pred ) const;

    template <typename OutputIterator>
    void writeNeighborhood( const Point & p, 
			    OutputIterator & out_it ) const;

    /**
     * Outputs the whole neighborhood of point [p] (except p itself)
     * satisfying the given predicate as a sequence of *out_it++ = ...
     *
     * @tparam OutputIterator any output iterator (like
     * std::back_insert_iterator< std::vector<int> >).
     *
     * @tparam PointPredicate any predicate object
     *
     * @param p any point of this space.
     * @param out_it any output iterator.
     * @param pred the predicate.
     */
    template <typename OutputIterator, 
	      typename PointPredicate>
    void writeProperNeighborhood( const Point & p, 
				  OutputIterator & out_it,
				  const PointPredicate & pred ) const;

    template <typename OutputIterator>
    void writeProperNeighborhood( const Point & p, 
				  OutputIterator & out_it ) const;

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

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:
    /**
     * The predicate for testing if a point belongs to the domain.
     */
    DomainPredicateType myPred;

    /**
     * The adjacency relation.
     */
    const TAdjacency & myAdjacency;
    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    DomainAdjacency();

  private:

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    DomainAdjacency & operator= ( const DomainAdjacency & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class DomainAdjacency


  /**
   * Overloads 'operator<<' for displaying objects of class 'DomainAdjacency'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DomainAdjacency' to write.
   * @return the output stream after the writing.
   */
  template <typename TDomain, typename TAdjacency>
  std::ostream&
  operator<< ( std::ostream & out, 
	       const DomainAdjacency<TDomain, TAdjacency> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/topology/DomainAdjacency.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DomainAdjacency_h

#undef DomainAdjacency_RECURSES
#endif // else defined(DomainAdjacency_RECURSES)
