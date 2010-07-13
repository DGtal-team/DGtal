#pragma once

/**
 * @file DomainPredicate.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/10
 *
 * Header file for module DomainPredicate.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DomainPredicate_RECURSES)
#error Recursive header files inclusion detected in DomainPredicate.h
#else // defined(DomainPredicate_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DomainPredicate_RECURSES

#if !defined DomainPredicate_h
/** Prevents repeated inclusion of headers. */
#define DomainPredicate_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/BasicPointPredicates.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class DomainPredicate
  /**
   * Description of template class 'DomainPredicate' <p> \brief Aim:
   * The predicate returning true iff the point is in the domain given
   * at construction. It is just a wrapper class around the methods
   * Domain::isInside( const Point & ), where \c Domain stands for any
   * model of CDomain.
   *
   * It is used by domains to define the type by their method \c
   * predicate().
   *
   * Model of CPointPredicate
   *
   * @see DomainAdjacency::predicate.
   */
  template <typename TDomain>
  struct DomainPredicate
  {
    // ----------------------- Standard services ------------------------------
  public:
    typedef TDomain Domain;
    typedef typename Domain::Point Point;

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    DomainPredicate ( const Domain & domain );

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    DomainPredicate ( const DomainPredicate & other );

   /**
     * @param p any point.
     * @return true iff p is in the domain.
     */
    bool operator()( const Point & p ) const;

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    DomainPredicate();

  private:
    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    DomainPredicate & operator= ( const DomainPredicate & other );

    // ------------------------- Internals ------------------------------------
  private:
    const Domain* const myDomain;

  }; // end of struct DomainPredicate


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/domains/DomainPredicate.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DomainPredicate_h

#undef DomainPredicate_RECURSES
#endif // else defined(DomainPredicate_RECURSES)
