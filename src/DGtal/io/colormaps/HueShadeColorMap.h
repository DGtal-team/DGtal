#pragma once

/**
 * @file HueShadeColorMap.h
 * @author Sebastien Fourey (\c Sebastien.Fourey@greyc.ensicaen.fr )
 * Groupe de Recherche en Informatique, Image, Automatique et Instrumentation de Caen - GREYC (CNRS, UMR 6072), ENSICAEN, France
 *
 * @date 2010/07/19
 *
 * Header file for module HueShadeColorMap.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(HueShadeColorMap_RECURSES)
#error Recursive header files inclusion detected in HueShadeColorMap.h
#else // defined(HueShadeColorMap_RECURSES)
/** Prevents recursive inclusion of headers. */
#define HueShadeColorMap_RECURSES

#if !defined HueShadeColorMap_h
/** Prevents repeated inclusion of headers. */
#define HueShadeColorMap_h

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
  // template class HueShadeColorMap
  /**
   * Description of template class 'HueShadeColorMap' <p>
   * \brief Aim: This class template may be used to (linearly) convert scalar values
   * in a given range into a color in a hue shade colormap, maybe aka rainbow color map.
   * 
   * The HueShadeColorMap can be used either as a functor object
   * (the value range is given at the object's construction) which converts a value 
   * into a LibBoard::Color structure, or it can be used through a static method
   * taking both the range and the value as parameters.
   *
   * The code below shows a possible use of this class.
   * @code
   * #include "Board/Color.h"
   * #include "HueShadeColorMap.h"
   * // ...
   * {
   *   HueShadeColorMap<float> hueShade(0.0f,1.0f);
   *   LibBoard::Color red = hueShade(1.0f);
   *   LibBoard::Color lightBlue1 = hueShade(0.5f);
   *   // Or, equivalently:
   *   LibBoard::Color lightBlue2 = HueShadeColorMap<float>::getColor(0.0f,1.0f,0.5f);
   * }
   * @endcode
   *
   * @tparam ValueType The type of the range values.
   */
  template <typename ValueType>
  class HueShadeColorMap
  {
    // ----------------------- Standard services ------------------------------
  public:

    /** 
     * Constructor.
     * 
     * @param min The lower bound of the value range.
     * @param max The upper bound of the value range.
     */
    HueShadeColorMap( const ValueType & min,
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
    ~HueShadeColorMap();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    HueShadeColorMap ( const HueShadeColorMap & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    HueShadeColorMap & operator= ( const HueShadeColorMap & other );

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
    HueShadeColorMap();

    // ------------------------- Internals ------------------------------------
  private:

    /** 
     * Converts a color from the HSV (Hue,Saturation,Value) space to the RGB space.
     * 
     * @param r The red component (out).
     * @param g The green component (out).
     * @param b The blue component (out).
     * @param h The hue of the color in [0..360)
     * @param s The saturation of the color in [0..1].
     * @param v The value of the color in [0..1].
     */
    static void HSVtoRGB(double &r, double &g, double &b,
			 double h, const double s, const double v);


  }; // end of class HueShadeColorMap


  /**
   * Overloads 'operator<<' for displaying objects of class 'HueShadeColorMap'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'HueShadeColorMap' to write.
   * @return the output stream after the writing.
   */
  template <typename ValueType>
  std::ostream&
  operator<< ( std::ostream & out, const HueShadeColorMap<ValueType> & object );
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/colormaps/HueShadeColorMap.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined HueShadeColorMap_h

#undef HueShadeColorMap_RECURSES
#endif // else defined(HueShadeColorMap_RECURSES)
