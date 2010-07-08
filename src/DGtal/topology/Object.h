#pragma once

/**
 * @file Object.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/07
 *
 * Header file for module Object.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Object_RECURSES)
#error Recursive header files inclusion detected in Object.h
#else // defined(Object_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Object_RECURSES

#if !defined Object_h
/** Prevents repeated inclusion of headers. */
#define Object_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CowPtr.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class Object
  /**
   * Description of template class 'Object' <p> \brief Aim: An object
   * (or digital object) represents a set in some digital space
   * associated with a digital topology.
   *
   * The digital topology induces a connectedness relation on the
   * object (transitive closure of the foreground adjacency) and a
   * connectedness relation on the complement of the set (transitive
   * closure of the background adjacency).
   *
   * Objects may be connected or not. The connectedness is stored with
   * the object, if it is known. Objects have a border, which are the
   * points which touch the complement in the sense of background
   * adjacency.
   *
   * @tparam DigitalTopologyType any realization of DigitalTopology.
   * @tparam DigitalSet any model of CDigitalSet.
   */
  template <typename DigitalTopologyType, typename DigitalSet>
  class Object
  {
    // ----------------------- Standard services ------------------------------
  public:
    typedef typename DigitalSet::SizeType SizeType;

    /**
     * Constructor. 
     *
     * @param topology the digital topology chosen for this set, a copy of
     * which is stored in the object.
     *
     * @param aPointSet the set of points of the object. It is copied
     * in the object.
     */
    Object( const DigitalTopologyType & topology, 
	    const DigitalSet & aPointSet );

    /**
     * Constructor. 
     *
     * @param topology the digital topology chosen for this set, a copy of
     * which is stored in the object.
     *
     * @param aPointSet the set of points of the object. It is smartly
     * reference in the object.
     */
    Object( const DigitalTopologyType & topology, 
	    const CowPtr<DigitalSet> & aPointSet );

    /**
     * Constructor by attachment of a dynamically allocated point set. 
     *
     * @param topology the digital topology chosen for this set, a copy of
     * which is stored in the object.
     *
     * @param aPointSetPtr a dynamically allocated pointer on a set of
     * points which is afterwards handled by this (which will take
     * care of its deletion).
     */
    Object( const DigitalTopologyType & topology, 
	    DigitalSet* aPointSetPtr );
 
    /**
     * Copy constructor.
     * @param other the object to clone.
     *
     * The copy is smart in the sense that the digital set is
     * referenced, and will be copied only if the set is changed.
     */
    Object ( const Object & other );

    /**
     * Destructor.
     */
    ~Object();

    /**
     * @return the number of elements in the set.
     */
    SizeType size() const;

    /**
     * A const reference on the point set defining the points of the
     * digital object.
     */
    const DigitalSet & pointSet() const;

    /**
     * A reference on the point set defining the points of the
     * digital object (may duplicate the set).
     */
    DigitalSet & pointSet();

    // ----------------------- Object services --------------------------------
  public:

    /**
     * @return the
     */
    Object border() const;

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
     * the digital topology of the object.
     */
    DigitalTopologyType myTopo;

    /**
     * A copy on write pointer on the associated (owned or not) point set
     */
    CowPtr<DigitalSet> myPointSet;

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    Object();

  private:


    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Object & operator= ( const Object & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class Object


  /**
   * Overloads 'operator<<' for displaying objects of class 'Object'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Object' to write.
   * @return the output stream after the writing.
   */
  template <typename DigitalTopologyType, typename DigitalSet>
  std::ostream&
  operator<< ( std::ostream & out, 
	       const Object<DigitalTopologyType, DigitalSet> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/topology/Object.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Object_h

#undef Object_RECURSES
#endif // else defined(Object_RECURSES)
