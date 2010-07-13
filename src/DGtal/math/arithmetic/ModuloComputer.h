#pragma once

/**
 * @file ModuloComputer.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/02
 *
 * Header file for module ModuloComputer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ModuloComputer_RECURSES)
#error Recursive header files inclusion detected in ModuloComputer.h
#else // defined(ModuloComputer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ModuloComputer_RECURSES

#if !defined ModuloComputer_h
/** Prevents repeated inclusion of headers. */
#define ModuloComputer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/IntegerTraits.h"
#include "DGtal/kernel/CUnsignedInteger.h"
#include "DGtal/kernel/CInteger.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ModuloComputer
  /**
   * Description of template class 'ModuloComputer' <p>
   *
   * \brief implements basic functions on modular arithmetic.
   *
   * Example:
   *
   *  @code
   *   ...
   *   //Construct an arithmetic modulo 15
   *   // Note that MyInteger must be a model of the concept CInteger
   *   ModuloComputer<MyInteger> modular(15);
   *   typedef MyInteger::UnsignedVersion myUnsignedInteger;
   *
   *   myUnsignedInteger a;
   *
   *   a = modular.cast( 2 );   //a contains the value 2
   *   a = modular.cast( -1 );  //a contains the value 14
   *   modular.increment( a ); //a contains the value 0
   *   ...
   *  @endcode
   */
  template <typename TInteger>
  class ModuloComputer
  {

    typedef typename IntegerTraits<TInteger>::SignedVersion Integer;
    typedef typename IntegerTraits<Integer>::ParamType IntegerParamType;

    typedef typename IntegerTraits<TInteger>::UnsignedVersion UnsignedInteger;
    typedef typename IntegerTraits<UnsignedInteger>::ParamType UnsignedIntegerParamType;

    BOOST_CONCEPT_ASSERT((CInteger<Integer>));
    BOOST_CONCEPT_ASSERT((CUnsignedInteger<UnsignedInteger>));    


  public:    
    /**
     * Initializes the modulo computer with the value [m].
     * @param m any non-zero integer.
     */
    ModuloComputer( const UnsignedIntegerParamType m );
    
    /**
     * Increment the value [i] modulo.
     * @param i any value between 0 and [k] (excluded).
     * @see k
     */
    void increment(  UnsignedIntegerParamType i ) const;

    /**
     * Decrement the value [i] modulo.
     * @param i any value between 0 and [k] (excluded).
     * @see k
     */
    void decrement(  UnsignedIntegerParamType  i ) const;

    /**
     * @param i any value between 0 and [k] (excluded).
     * @return the incremented value of [i] modulo [k].
     * @see k
     */
    UnsignedInteger  next(  const UnsignedIntegerParamType i ) const;

    /**
     * @param i any value between 0 and [k] (excluded).
     * @return the decremented value of [i] modulo [k].
     * @see k
     */
    UnsignedInteger previous(  const UnsignedIntegerParamType i ) const;

    /**
     * @param i any integer value.
     * @return the value of [i] modulo [k].
     * @see k
     */
    UnsignedInteger cast( const IntegerParamType i ) const;
    
    /**
     * Less comparator modulo. Be careful, modulo comparisons have no
     * sense when the absolute difference of the values are around k / 2.
     *
     * @param i any value between 0 and [k] (excluded).
     * @param j any value between 0 and [k] (excluded).
     * @return 'true' if [i] strictly precedes [j] in a window 'floor([k]/2)'.
     * @see k
     */
    bool less( const UnsignedIntegerParamType i, const UnsignedIntegerParamType j ) const;

    /**
     * Performs j - i modulo, assuming less(i,j) is true.
     *
     * @param j any value between 0 and [k] (excluded).
     * @param i any value between 0 and [k] (excluded).
     * @return the value j - i, always positive. 
     * @see k
     */
    UnsignedInteger posDiff( const UnsignedIntegerParamType j, const UnsignedIntegerParamType i ) const;
    
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

    // ------------------------- Private Datas --------------------------------
  private:
    /**
     * Modulo of all computations.
     */
    UnsignedInteger k;

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    ModuloComputer();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    ModuloComputer ( const ModuloComputer & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    ModuloComputer & operator= ( const ModuloComputer & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class ModuloComputer

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/math/arithmetic/ModuloComputer.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ModuloComputer_h

#undef ModuloComputer_RECURSES
#endif // else defined(ModuloComputer_RECURSES)
