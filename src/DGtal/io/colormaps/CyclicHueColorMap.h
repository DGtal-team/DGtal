#pragma once

/**
 * @file CyclicHueColorMap.h
 * @author Sebastien Fourey (\c Sebastien.Fourey@greyc.ensicaen.fr )
 * Groupe de Recherche en Informatique, Image, Automatique et Instrumentation de Caen - GREYC (CNRS, UMR 6072), ENSICAEN, France
 *
 * @date 2010/07/19
 *
 * Header file for module CyclicHueColorMap.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CyclicHueColorMap_RECURSES)
#error Recursive header files inclusion detected in CyclicHueColorMap.h
#else // defined(CyclicHueColorMap_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CyclicHueColorMap_RECURSES

#if !defined CyclicHueColorMap_h
/** Prevents repeated inclusion of headers. */
#define CyclicHueColorMap_h

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
  // template class CyclicHueColorMap
  /**
   * Description of template class 'CyclicHueColorMap' <p>
   *
   * @brief Aim: This class template may be used to (linearly) convert scalar
   * values in a given range into a color in a \em cyclic hue shade
   * colormap, maybe aka rainbow color map. This color map is suitable, for
   * example, to colorize distance functions.
   * 
   * The CyclicHueColorMap can be used either as a functor object (the value
   * range is given at the object's construction) which converts a value  into a
   * LibBoard::Color structure, or it can be used through a static method taking
   * both the range and the value as parameters.
   *
   * @tparam PValueType The type of the range values.
   */
  template <typename PValueType>
  class CyclicHueColorMap
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
     * @param cycles The number of cycles in the colormap.
     */
    CyclicHueColorMap( const PValueType & min,
		       const PValueType & max,
		       const unsigned int cycles = 5 );
    
    /** 
     * Computes the color associated with a value in a given range.
     * 
     * @param value A value within the value range.
     * @return A color whose hue linearly depends on the 
     * position of [value] within the current range.
     */
    LibBoard::Color operator()( const PValueType & value ) const;
      
    /**
     * Destructor.
     */
    ~CyclicHueColorMap();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    CyclicHueColorMap ( const CyclicHueColorMap & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    CyclicHueColorMap & operator= ( const CyclicHueColorMap & other );

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
     * Computes the color associated with a value in a given range.
     * 
     * @param cycles The number of (rainbow) cycles.
     * @param min The lower bound of the value range.  
     * @param max The upper bound of the value range.
     * @param value A value within the value range.
     * @return A color whose hue linearly depends on the 
     * position of [value] within the range [min]..[max]. 
     */
    static LibBoard::Color getColor( const unsigned int cycles,
				     const PValueType & min,
				     const PValueType & max,
				     const PValueType & value );
    
    // ------------------------- Protected Datas ------------------------------
  private:

    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:

    PValueType myMin;		/**< The lower bound of the value range.  */
    PValueType myMax;           /**< The lower bound of the value range.  */
    unsigned int myCycles;	/**< The number of cycles in the color map. */
    
    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    CyclicHueColorMap();

    // ------------------------- Internals ------------------------------------
  private:

    /** 
     * Converts a color from the HSV (Hue,Saturation,Value) space to the RGB
     * space.
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


  }; // end of class CyclicHueColorMap


  /**
   * Overloads 'operator<<' for displaying objects of class 'CyclicHueColorMap'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'CyclicHueColorMap' to write.
   * @return the output stream after the writing.
   */
  template <typename PValueType>
    std::ostream&
    operator<< ( std::ostream & out, const CyclicHueColorMap<PValueType> & object );
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/colormaps/CyclicHueColorMap.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CyclicHueColorMap_h

#undef CyclicHueColorMap_RECURSES
#endif // else defined(CyclicHueColorMap_RECURSES)
