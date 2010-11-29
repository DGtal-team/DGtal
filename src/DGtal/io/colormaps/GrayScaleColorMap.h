/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

#pragma once

/**
 * @file GrayScaleColorMap.h
 * @author Sebastien Fourey (\c Sebastien.Fourey@greyc.ensicaen.fr )
 * Groupe de Recherche en Informatique, Image, Automatique et Instrumentation de Caen - GREYC (CNRS, UMR 6072), ENSICAEN, France
 *
 * @date 2010/07/19
 *
 * Header file for module GrayScaleColorMap.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GrayScaleColorMap_RECURSES)
#error Recursive header files inclusion detected in GrayscaleColorMap.h
#else // defined(GrayScaleColorMap_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GrayScaleColorMap_RECURSES

#if !defined GrayScaleColorMap_h
/** Prevents repeated inclusion of headers. */
#define GrayScaleColorMap_h

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
   * @tparam PValueType The type of the range values.
   */
  template <typename PValueType>
  class GrayscaleColorMap
  {
    
  public:
    
    typedef PValueType ValueType;

    // ----------------------- Standard services ------------------------------
  public:

    /** 
     * Constructor.
     * 
     * @param min The lower bound of the value range.
     * @param max The upper bound of the value range.
     */
    GrayscaleColorMap( const PValueType & min,
		       const PValueType & max );


    /** 
     * Computes the gray level associated with a value in a given range.
     * 
     * @param value A value within the value range.
     * @return A gray level (as a Color) which linearly depends on the 
     * position of [value] within the current range.
     */
    LibBoard::Color operator()( const PValueType & value ) const;
      
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
    const PValueType & min() const;

    /** 
     * Returns the upper bound of the value range.
     *
     * @return The upper bound of the value range.
     */
    const PValueType & max() const;

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
    static LibBoard::Color getColor( const PValueType & min,
				     const PValueType & max,
				     const PValueType & value );
    
    // ------------------------- Protected Datas ------------------------------
  private:

    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:

    PValueType myMin;		/**< The lower bound of the value range.  */
    PValueType myMax;            /**< The lower bound of the value range.  */

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
  template <typename PValueType>
  std::ostream&
  operator<< ( std::ostream & out, const GrayscaleColorMap<PValueType> & object );
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/colormaps/GrayscaleColorMap.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GrayscaleColorMap_h

#undef GrayScaleColorMap_RECURSES
#endif // else defined(GrayScaleColorMap_RECURSES)
