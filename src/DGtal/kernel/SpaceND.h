#pragma once

/** 
 * @file SpaceND.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/05/14
 * 
 * Header file for module SpaceND.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SpaceND_RECURSES)
#error Recursive header files inclusion detected in SpaceND.h
#else // defined(SpaceND_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SpaceND_RECURSES

#if !defined SpaceND_h
/** Prevents repeated inclusion of headers. */
#define SpaceND_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CInteger.h"
#include "DGtal/kernel/PointVector.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{
  
  /////////////////////////////////////////////////////////////////////////////
  // class SpaceND
  /** 
   * Description of class 'SpaceND' <p>
   *
   * Aim: SpaceND defines the fundamental structure of a Digital Space in ND.
   * \todo documentation here!
   *
   * @tparam Dimension the dimension of the Digital Space.
   * @tparam Integer the Integer class used to specify the arithmetic (default type = int).
   */
  
  template <std::size_t Dimension, typename Integer = int>
  class SpaceND
  {
  public:

    /// \todo fixer des concept check sur Integer
    BOOST_CONCEPT_ASSERT((CInteger<Integer>));
    
    
    typedef Integer TInteger;
    typedef typename IntegerTraits<Integer>::UnsignedVersion UnsignedInteger;
    typedef std::size_t  DimensionType;
    
    typedef PointVector<Dimension, Integer> Point;
    typedef PointVector<Dimension, Integer> Vector;
    typedef Integer SizeType;
    
    typedef SpaceND<Dimension,Integer> Space;

    // static constants
    static const DimensionType staticDimension = Dimension;
    
    //typedef Matrix<DimensionT,DimensionT,Integer> Matrix;
    template <std::size_t Codimension>
    struct Subcospace {
      typedef SpaceND<Dimension-Codimension,Integer> Type;
    };
    template <std::size_t Subdimension>
    struct Subspace {
      typedef SpaceND<Subdimension,Integer> Type;
    };
    
    
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor
     *
     */
    SpaceND() {};
    
    /**
     * Destructor. 
     */
    ~SpaceND() {};

    /**
     * @return the digital space of specified subdimension of this space. 
     */
    template <std::size_t Subdimension>
    static
    typename Subspace<Subdimension>::Type subspace()
      {
	ASSERT( Subdimension <= Dimension );
	return SpaceND<Subdimension,Integer>();
      }
    

    /**
     * @return the digital space of specified codimension of this space. 
     */
    template <std::size_t Codimension>
    static
    typename Subcospace<Codimension>::Type subcospace()
      {
	ASSERT( Codimension <= Dimension );
	return SpaceND<Dimension-Codimension,Integer>();
      }


    /**
     * @return the dimension of the digital space.
     */
    static DimensionType dimension() { return Dimension; }

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    static void selfDisplay( std::ostream & out )
    {
      out << "[SpaceND dim=" << dimension() << " size_elem=" << sizeof( Integer ) << " ]";
    }

  private:
    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    SpaceND & operator=( const SpaceND & other );
  
    // ------------------------- Internals ------------------------------------
  private:
  
  }; // end of class SpaceND


  /**
   * Overloads 'operator<<' for displaying objects of class 'SpaceND'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'SpaceND' to write.
   * @return the output stream after the writing.
   */
  template <std::size_t Dimension, typename Integer>
  static std::ostream&
  operator<<( std::ostream & out, const SpaceND<Dimension,Integer> & object )
  {
    object.selfDisplay( out );
    return out;
  }

  
} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SpaceND_h

#undef SpaceND_RECURSES
#endif // else defined(SpaceND_RECURSES)
