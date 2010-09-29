#pragma once

/**
 * @file Morton.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/09/10
 *
 * Header file for module Morton.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Morton_RECURSES)
#error Recursive header files inclusion detected in Morton.h
#else // defined(Morton_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Morton_RECURSES

#if !defined Morton_h
/** Prevents repeated inclusion of headers. */
#define Morton_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <boost/array.hpp>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/utils/Bits.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class Morton
  /**
   * Description of template class 'Morton' <p>
   * \brief Aim: @todo
   */
  template <typename THashKey, typename TPoint >
  class Morton
  {
  public:
    typedef THashKey HashKey;
    typedef TPoint Point;
    typedef typename Point::Coordinate Coordinate;
    static const std::size_t Dimension = Point::Dimension;    
    

    /**
     * Constructor
     *
     */
    Morton();
    
    /**
     * Interleave the bits of the nbIn inputs.
     * @param input an array of the nbIn values to mix in.
     * @param output The result
     */ 
    void interleaveBits(const Point  & aPoint, HashKey & output) const;


    /**
     * Returns the key corresponding to the coordinates passed in the parameters.
     *
     * @param treeDepth The depth at which the coordinates are to be
     * read (usualy corresponds to the deepest leave).  
     *
     * @param coordinates An array containing the coordinates to
     * convert into a key.
     */
    HashKey keyFromCoordinates(const std::size_t treeDepth, const Point & coordinates) const;

    /**
     * Computes the coordinates correspponding to a key.
     *
     * @param key The key.
     * @param coordinates Will contain the resulting coordinates.
     */
    void coordinatesFromKey(const HashKey key, Point & coordinates) const;
    /**
     * Returns the parent key of a key passed in parameter.
     *
     * @param key The key.
     */ 
    inline HashKey parentKey(const HashKey key) const
    {
      return key >> Dimension;
    }

    /**
     * Computes the brother keys (ie the keys having the same parent) of the key passed in parameter.
     *
     * @param key The key.
     * @param result Will contain the resulting brother keys.
     */ 
    void brotherKeys(const HashKey key, HashKey* result ) const;

    
    /**
     * Computes the children keys of the key passed in parameter.
     *
     * @param key The key.
     * @param result Will contain the resulting children keys.
     */ 
    void childrenKeys(const HashKey key, HashKey* result ) const;
    
  private: 
    
    boost::array< HashKey,LOG2<sizeof(HashKey)*8>::VALUE> myDilateMasks;
    boost::array< HashKey,LOG2<sizeof(HashKey)*8>::VALUE> myContractMasks;
  };
} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Includes inline functions
#include "DGtal/kernel/images/Morton.ih"

#endif // !defined Morton_h

#undef Morton_RECURSES
#endif // else defined(Morton_RECURSES)
