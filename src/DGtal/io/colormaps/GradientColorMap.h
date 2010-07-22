#pragma once

/**
 * @file GradientColorMap.h
 * @author Sebastien Fourey (\c Sebastien.Fourey@greyc.ensicaen.fr )
 * Groupe de Recherche en Informatique, Image, Automatique et Instrumentation de Caen - GREYC (CNRS, UMR 6072), ENSICAEN, France
 *
 * @date 2010/07/19
 *
 * Header file for module GradientColorMap.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GradientColorMap_RECURSES)
#error Recursive header files inclusion detected in GradientColorMap.h
#else // defined(GradientColorMap_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GradientColorMap_RECURSES

#if !defined GradientColorMap_h
/** Prevents repeated inclusion of headers. */
#define GradientColorMap_h

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
  // template class GradientColorMap
  /**
   * Description of template class 'GradientColorMap' <p>
   * \brief Aim: This class template may be used to (linearly) convert scalar
   * values in a given range into a color in a gradient between two colors. 
   * 
   * The GradientColorMap can be used either as a functor object
   * (the value range is given at the object's construction, together with the
   * reference color) which converts a value into a LibBoard::Color structure,
   * or it can be used through a static method taking both the range and the
   * value as parameters.
   *
   * The code below shows a possible use of this class.
   * @code
   * #include "Board/Color.h"
   * #include "GradientColorMap.h"
   * // ...
   * {
   *   GradientColorMap<float> gradient(0.0f,1.0f,
   *                                    LibBoard::Color::Blue,
   *                                    LibBoard::Color::Red );
   *   LibBoard::Color c1 = blueShade(0.33f);
   *   LibBoard::Color c2 = blueShade(0.75f);
   *   // Or, equivalently:
   *   LibBoard::Color c3 = GradientColorMap<float>::getColor(LibBoard::Color::Blue,
   *                                                          LibBoard::Color::Red,
   *                                                          0.0f,1.0f,
   *                                                          0.75f);
   * }
   * @endcode
   * @todo Fix the GradientColorMap::getColor method so that it works!
   *
   * @tparam ValueType The type of the range values.
   */
  template <typename PValueType>
  class GradientColorMap
  {

  public:
    
    typedef PValueType ValueType;

    enum Preset { Spring, Summer, Autumn, Winter, Cool, Copper };


    // ----------------------- Standard services ------------------------------
  public:

    /** 
     * Constructor.
     * 
     * @param min The lower bound of the value range.
     * @param max The upper bound of the value range.
     * @param firstColor The color of the lower bound.
     * @param lastColor The color of the upper bound.
     */
    GradientColorMap( const PValueType & min,
		      const PValueType & max,
		      const LibBoard::Color & firstColor,
		      const LibBoard::Color & lastColor );

    /** 
     * Constructor.
     * 
     * @param min The lower bound of the value range.
     * @param max The upper bound of the value range.
     * @param preset A preset gradient.
     */
    GradientColorMap( const PValueType & min,
		      const PValueType & max,
		      const Preset & preset );

    
    /** 
     * Computes the color associated with a value in a given range.
     * 
     * @param value A value within the value range.
     * @return A color whose color linearly depends on the 
     * position of [value] within the current range.
     */
    LibBoard::Color operator()( const PValueType & value ) const;
      
    /**
     * Destructor.
     */
    ~GradientColorMap();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    GradientColorMap ( const GradientColorMap & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    GradientColorMap & operator= ( const GradientColorMap & other );

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
     * @param color The color associated with the upper bound. 
     * @param min The lower bound of the value range.  
     * @param max The upper bound of the value range.
     * @param value A value within the value range.
     * @return A color whose color linearly depends on the 
     * position of [value] within the range [min]..[max]. 
     */
    static LibBoard::Color getColor( const LibBoard::Color firstColor,
				     const LibBoard::Color lastColor,
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
    LibBoard::Color myFirstColor;	/**< The color of the lower bound value. */
    LibBoard::Color myLastColor;	/**< The color of the upper bound value. */
    
    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    GradientColorMap();

    // ------------------------- Internals ------------------------------------
  private:


  }; // end of class GradientColorMap

  /**
   * Overloads 'operator<<' for displaying objects of class 'GradientColorMap'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'GradientColorMap' to write.
   * @return the output stream after the writing.
   */
  template <typename PValueType>
  std::ostream&
  operator<< ( std::ostream & out, const GradientColorMap<PValueType> & object );
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/colormaps/GradientColorMap.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GradientColorMap_h

#undef GradientColorMap_RECURSES
#endif // else defined(GradientColorMap_RECURSES)
