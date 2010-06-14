#pragma once

/** 
 * @file Space.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/05/14
 * 
 * Header file for module Space.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Space_RECURSES)
#error Recursive header files inclusion detected in Space.h
#else // defined(Space_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Space_RECURSES

#if !defined Space_h
/** Prevents repeated inclusion of headers. */
#define Space_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/Point.h"
#include "DGtal/kernel/Vector.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{
  
  /////////////////////////////////////////////////////////////////////////////
  // class Space
  /** 
   * Description of class 'Space' <p>
   *
   * Aim: Space defines the fundamental structure of a Digital Space.
   *
   */
  
  template <class IntT, std::size_t Dimension>
  class Space
  {
  public:

    /// \todo fixer des concept check sur IntT
    BOOST_CONCEPT_ASSERT((boost::EqualityComparable<IntT>));
    BOOST_CONCEPT_ASSERT((boost::Assignable<IntT>));
    
    
    typedef IntT IntegerType;
    typedef std::size_t  DimensionType;
    
    typedef Point<IntT, Dimension> PointType;
    typedef Vector<IntT,Dimension> VectorType;
    
    typedef Space<IntT,Dimension> SpaceType;

    // static constants
    static const DimensionType staticDimension = Dimension;
    
    //typedef Matrix<DimensionT,DimensionT,IntT> Matrix;
    template <std::size_t Codimension>
    struct Subcospace {
      typedef Space<IntT,Dimension-Codimension> Type;
    };
    template <std::size_t Subdimension>
    struct Subspace {
      typedef Space<IntT,Subdimension> Type;
    };
    
    
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor
     *
     */
    Space();
    
    /**
     * Destructor. 
     */
    ~Space();

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Does nothing.
     */
    Space( const Space & other );

    /**
     * @return the digital space of specified subdimension of this space. 
     */
    template <std::size_t Subdimension>
    static
    typename Subspace<Subdimension>::Type subspace();

    /**
     * @return the digital space of specified codimension of this space. 
     */
    template <std::size_t Codimension>
    static
    typename Subcospace<Codimension>::Type subcospace();


    /**
     * @return the dimension of the digital space.
     */
    static DimensionType dimension() ;

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
     void selfDisplay( std::ostream & out ) const;

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:


  private:


    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Space & operator=( const Space & other );
  
    // ------------------------- Internals ------------------------------------
  private:
  
  }; // end of class Space


  /**
   * Overloads 'operator<<' for displaying objects of class 'Space'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Space' to write.
   * @return the output stream after the writing.
   */
  template <typename IntT, std::size_t Dimension>
  std::ostream&
  operator<<( std::ostream & out, const Space<IntT,Dimension> & object );

  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#include "DGtal/kernel/Space.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Space_h

#undef Space_RECURSES
#endif // else defined(Space_RECURSES)
