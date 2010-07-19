#pragma once

/**
 * @file GrayscaleColorMap.h
 * @author Sebastien Fourey (\c Sebastien.Fourey@greyc.ensicaen.fr )
 * Groupe de Recherche en Informatique, Image, Automatique et Instrumentation de Caen - GREYC (CNRS, UMR 6072), ENSICAEN, France
 *
 * @date 2010/07/19
 *
 * Header file for module GrayscaleColorMap.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GrayscaleColorMap_RECURSES)
#error Recursive header files inclusion detected in GrayscaleColorMap.h
#else // defined(GrayscaleColorMap_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GrayscaleColorMap_RECURSES

#if !defined GrayscaleColorMap_h
/** Prevents repeated inclusion of headers. */
#define GrayscaleColorMap_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "Board/Board.h"
#include "Board/Color.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class GrayscaleColorMap
  /**
   * Description of template class 'GrayscaleColorMap' <p>
   * \brief Aim: This class template may be used to (linearly) convert scalar values
   * in a given range into gray levels.
   * 
   * The GrayscaleColorMap can be used either as a functor object
   * (the value range is given at the object's construction) which converts a value 
   * into a LibBoard::Color structure, or it can be used through a static method
   * taking both the range and the value as parameters.
   *
   * The code below shows a possible use of this class.
   * @code
   * #include "Board/Color.h"
   * #include "GrayscaleColorMap.h"
   * // ...
   * {
   *   GrayscaleColorMap<float> grayShade(0.0f,1.0f);
   *   LibBoard::Color white = grayShade(1.0f);
   *   LibBoard::Color gray1 = grayShade(0.5f);
   *   // Or, equivalently:
   *   LibBoard::Color gray2 = GrayscaleColorMap<float>::getColor(0.0f,1.0f,0.5f);
   * }
   * @endcode
   *
   * @tparam ValueType The type of the range values.
   */
  template <typename ValueType>
  class GrayscaleColorMap
  {
    // ----------------------- Standard services ------------------------------
  public:

    /** 
     * Constructor.
     * 
     * @param min The lower bound of the value range.
     * @param max The upper bound of the value range.
     */
    GrayscaleColorMap( const ValueType & min,
		       const ValueType & max );


    /** 
     * Computes the gray level associated with a value in a given range.
     * 
     * @param value A value within the value range.
     * @return A gray level (as a Color) which linearly depends on the 
     * position of [value] within the current range.
     */
    LibBoard::Color operator()( const ValueType & value ) const;
      
    /**
     * Destructor.
     */
    ~GrayscaleColorMap();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    GrayscaleColorMap ( const GrayscaleColorMap & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    GrayscaleColorMap & operator= ( const GrayscaleColorMap & other );

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

    /** 
     * Returns the lower bound of the value range.
     *
     * @return The lower bound of the value range.
     */
    const ValueType & min() const;

    /** 
     * Returns the upper bound of the value range.
     *
     * @return The upper bound of the value range.
     */
    const ValueType & max() const;

    // ----------------------- Static methods ---------------------------------


    /** 
     * Computes the gray level associated with a value in a given range.
     * 
     * @param min The lower bound of the value range.  
     * @param max The upper bound of the value range.
     * @param value A value within the value range.
     * @return A gray level (as a Color) which linearly depends on the 
     * position of [value] within the range [min]..[max]. 
     */
    static LibBoard::Color getColor( const ValueType & min,
				     const ValueType & max,
				     const ValueType & value );
    
    // ------------------------- Protected Datas ------------------------------
  private:

    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:

    ValueType myMin;		/**< The lower bound of the value range.  */
    ValueType myMax;            /**< The lower bound of the value range.  */

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    GrayscaleColorMap();

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class GrayscaleColorMap


  /**
   * Overloads 'operator<<' for displaying objects of class 'GrayscaleColorMap'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'GrayscaleColorMap' to write.
   * @return the output stream after the writing.
   */
  template <typename ValueType>
  std::ostream&
  operator<< ( std::ostream & out, const GrayscaleColorMap<ValueType> & object );
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/colormaps/GrayscaleColorMap.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GrayscaleColorMap_h

#undef GrayscaleColorMap_RECURSES
#endif // else defined(GrayscaleColorMap_RECURSES)
