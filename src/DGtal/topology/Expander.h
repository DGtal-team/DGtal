#pragma once

/**
 * @file Expander.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/10
 *
 * Header file for module Expander.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Expander_RECURSES)
#error Recursive header files inclusion detected in Expander.h
#else // defined(Expander_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Expander_RECURSES

#if !defined Expander_h
/** Prevents repeated inclusion of headers. */
#define Expander_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/sets/DigitalSetSelector.h"
#include "DGtal/kernel/sets/DigitalSetDomain.h"
#include "DGtal/topology/Object.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class Expander
  /**
   * Description of template class 'Expander' <p> \brief Aim: This
   * class is useful to visit an object by adjacencies, layer by
   * layer.
   *
   * The expander implements a breadth-first algorithm on the graph of
   * adjacencies. It can be used not only to detect connected
   * component but also to identify the layers of the object located
   * at a given distance of a starting set.
   *
   * The \b core of the expander is at the beginning the set of points
   * at distance 0. Each layer is at a different distance from the
   * initial core. The expander may move point by point or layer by
   * layer.
   *
   * @tparam TObject the type of the digital object.
   *
   * @code
   * Point p( ... );
   * ObjectType object( ... );
   * typedef Expander< ObjectType > ObjectExpander;
   * ObjectExpander expander( object, p );
   * while ( ! expander.finished() )
   *   {
   *     std::cout << "Layer " << expander.distance() << " :";
   *     for ( ObjectExpander::ConstIterator it = expander.begin();
   *           it != expander.end();
   *           ++it )
   *        std::cout << " " << *it;
   *     std::cout << endl;
   *     expander.nextLayer();
   *   }
   * @endcode
   *
   * @see testExpander.cpp
   * @see testObject.cpp
   */
  template <typename TObject>
  class Expander
  {
    // ----------------------- Associated types ------------------------------
  public:
    typedef TObject Object;
    typedef typename Object::SizeType SizeType;
    typedef typename Object::Point Point;
    typedef typename Object::Domain Domain;
    typedef typename Object::SmallSet SmallSet;
    typedef typename Object::DigitalSet DigitalSet;
    typedef typename Object::DigitalTopology DigitalTopology;
    typedef typename Object::ForegroundAdjacency ForegroundAdjacency;
    typedef typename Domain::Space Space;
    typedef typename DigitalSet::Iterator Iterator;
    typedef typename DigitalSet::ConstIterator ConstIterator;
    typedef DigitalSetDomain<DigitalSet> ObjectDomain;
    typedef DigitalSetDomain<DigitalSet> CoreDomain;
    typedef typename DigitalSetSelector< Domain, SMALL_DS + HIGH_ITER_DS >::Type NeighborhoodSet;
    typedef DomainAdjacency< ObjectDomain, ForegroundAdjacency> ObjectAdjacency;
    typedef typename ObjectDomain::Predicate ObjectDomainPredicate; 
    typedef typename CoreDomain::Predicate InCoreDomainPredicate; 
    //typedef DomainPredicate< CoreDomain > InCoreDomainPredicate;
    typedef NotPointPredicate< InCoreDomainPredicate > NotInCoreDomainPredicate;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~Expander();

    /**
     * Constructor from a point. This point provides the initial core
     * of the expander.
     *
     * @param object the digital object in which the expander expands.
     * @param p any point in the given object.
     */
    Expander( const Object & object, const Point & p );

    /**
     * Constructor from iterators. All points visited between the
     * iterators should be distinct two by two. The so specified set
     * of points provides the initial core of the expander.
     *
     * @tparam the type of an InputIterator pointing on a Point.
     *
     * @param object the digital object in which the expander expands.
     * @param b the begin point in a set.
     * @param e the end point in a set.
     */
    template <typename PointInputIterator>
    Expander( const Object & object,
	      PointInputIterator b, PointInputIterator e );


    // ----------------------- Expansion services ------------------------------
  public:

    /**
     * @return 'true' if all possible elements have been visited.
     */
    bool finished() const;

    /**
     * @return the current distance to the initial core, or
     * equivalently the index of the current layer.
     */
    SizeType distance() const;

    /**
     * Extract next layer. You might used begin() and end() to access
     * all the elements of the new layer.
     *
     * @return 'true' if there was another layer, or 'false' if it was the
     * last (ie. reverse of finished() ).
     */
    bool nextLayer();

    /**
     * @return a const reference on the (current) core set of points.
     */
    const DigitalSet & core() const;

    /**
     * @return a const reference on the (current) layer set of points.
     */
    const DigitalSet & layer() const;

    /**
     * @return the iterator on the first element of the layer.
     */
    ConstIterator begin() const;

    /**
     * @return the iterator after the last element of the layer.
     */
    ConstIterator end() const;

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
     * The domain in which the object is lying.
     */
    const Domain & myEmbeddingDomain;

    /**
     * The object where the expansion takes place.
     */
    const Object & myObject;

    /**
     * The domain corresponding to the object.
     */
    ObjectDomain myObjectDomain;

    /**
     * The adjacency that is used in myObjectDomain.
     */
    ObjectAdjacency myObjectAdjacency;

    /**
     * Set representing the core of the expansion: the expansion should not
     * enter the core.
     */
    DigitalSet myCore;

    /**
     * Set representing the current layer.
     */
    DigitalSet myLayer;

    /**
     * Current distance to origin.
     */
    SizeType myDistance;

    /**
     * Boolean stating whether the expansion is over or not.
     */
    bool myFinished;

    /**
     * Predicate in-core.
     */
    InCoreDomainPredicate myInCorePred;

    /**
     * Predicate ensuring the not-in-core expansion. 
     */
    NotInCoreDomainPredicate myNotInCorePred;

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    Expander();

    /**
     * Computes the next layer just around [src]. The member 'm_core' must
     * be up to date (i.e, [src] is a subset of 'm_core').  'm_layer' is
     * cleared in this method.  At first call, [src] should be 'm_core',
     * then [src] should be 'm_layer'.
     *
     * @param src the set around which the new layer is computed.
     */
    void computeNextLayer( const DigitalSet & src );

    /**
     * Push the layer into the current core and clear it. Must be called
     * before computeNewLayer.
     */
    void endLayer();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    Expander ( const Expander & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Expander & operator= ( const Expander & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class Expander


  /**
   * Overloads 'operator<<' for displaying objects of class 'Expander'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Expander' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const Expander<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/topology/Expander.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Expander_h

#undef Expander_RECURSES
#endif // else defined(Expander_RECURSES)
