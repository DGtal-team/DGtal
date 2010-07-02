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
#include "DGtal/kernel/CInteger.h"
#include "DGtal/kernel/PointVector.h"
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
  
  template <typename Integer, std::size_t Dimension>
  class Space
  {
  public:

    /// \todo fixer des concept check sur Integer
    BOOST_CONCEPT_ASSERT((CInteger<Integer>));
    
    
    typedef Integer TInteger;
    typedef std::size_t  TDimension;
    
    typedef PointVector<Integer, Dimension> Point;
    typedef PointVector<Integer,Dimension> Vector;
    typedef Integer SizeType;
    
    typedef Space<Integer,Dimension> TSpace;

    // static constants
    static const TDimension staticDimension = Dimension;
    
    //typedef Matrix<DimensionT,DimensionT,Integer> Matrix;
    template <std::size_t Codimension>
    struct Subcospace {
      typedef Space<Integer,Dimension-Codimension> Type;
    };
    template <std::size_t Subdimension>
    struct Subspace {
      typedef Space<Integer,Subdimension> Type;
    };
    
    
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor
     *
     */
    Space() {};
    
    /**
     * Destructor. 
     */
    ~Space() {};

    /**
     * @return the digital space of specified subdimension of this space. 
     */
    template <std::size_t Subdimension>
    static
    typename Subspace<Subdimension>::Type subspace()
      {
	ASSERT( Subdimension <= Dimension );
	return Space<Integer,Subdimension>();
      }
    

    /**
     * @return the digital space of specified codimension of this space. 
     */
    template <std::size_t Codimension>
    static
    typename Subcospace<Codimension>::Type subcospace()
      {
	ASSERT( Codimension <= Dimension );
	return Space<Integer,Dimension-Codimension>();
      }


    /**
     * @return the dimension of the digital space.
     */
    static TDimension dimension() { return Dimension; }

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    static void selfDisplay( std::ostream & out )
    {
      out << "[Space dim=" << dimension() << " size_elem=" << sizeof( Integer ) << " ]";
    }

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
  template <typename Integer, std::size_t Dimension>
  static std::ostream&
  operator<<( std::ostream & out, const Space<Integer,Dimension> & object )
  {
    object.selfDisplay( out );
    return out;
  }

  
} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Space_h

#undef Space_RECURSES
#endif // else defined(Space_RECURSES)
