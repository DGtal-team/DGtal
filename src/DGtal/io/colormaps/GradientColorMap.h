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
   * values in a given range into a color in a gradient defined by two
   * or more colors.
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
   *   Board b;
   *
   *   GradientColorMap<float> gradient( 0.0, 1000.0, Color::White, Color::Red );
   *   b.setPenColor( gradient( 230.0 ) ); // Somewhere between white and red.
   *
   *   GradientColorMap<int> grad3( 0, 500 );
   *   grad3.addColor( Color::Blue );
   *   grad3.addColor( Color::White );
   *   grad3.addColor( Color::Red );
   *   
   *   b.setPenColor( grad3( 100 ) ); // Between Blue and white.
   *   b.setPenColor( grad3( 300 ) ); // Between white and red.
   * }
   * @endcode
   *
   * @tparam ValueType The type of the range values.
   */
  template <typename PValueType>
  class GradientColorMap
  {

  public:
    
    typedef PValueType ValueType;

    enum Preset { Spring, Summer, Autumn, Winter, Cool, Copper, Hot, Jet };


    // ----------------------- Standard services ------------------------------
  public:

    /** 
     * Constructor.
     * 
     * @param min The lower bound of the value range.
     * @param max The upper bound of the value range.
     */
    GradientColorMap( const PValueType & min,
		       const PValueType & max );

    /** 
     * Constructor for a gradient between two colors.
     * 
     * @param min The lower bound of the value range.
     * @param max The upper bound of the value range.
     * @param firstColor The "left" color of the gradient.
     * @param lastColor  The "right" color of the gradient.
     */
    GradientColorMap( const ValueType & min,
		      const ValueType & max,
		      const LibBoard::Color & firstColor,
		      const LibBoard::Color & lastColor );
    /** 
     * Constructor.
     * 
     * @param min The lower bound of the value range.
     * @param max The upper bound of the value range.
     * @param preset A preset identifier.
     */
    GradientColorMap( const PValueType & min,
		       const PValueType & max,
		       const Preset & preset );

    
    /** 
     * Computes the color associated with a value in a given range.
     * 
     * @param value A value within the value range.
     * @return A color whose brightness linearly depends on the 
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
     * Clears the list of colors.
     * 
     */
    void clearColors();

    /** 
     * Adds a color to the list of color steps.
     * 
     * @param color A color.
     */
    void addColor( const LibBoard::Color & color );

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
     * @param colors The gradients boundary colors.
     * @param min The lower bound of the value range.  
     * @param max The upper bound of the value range.
     * @param value A value within the value range.
     * @return A color whose color linearly depends on the 
     * position of [value] within the range [min]..[max]. 
     */
    static LibBoard::Color getColor( const std::vector<LibBoard::Color> & colors,
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
    std::vector<LibBoard::Color> myColors;	/**< The gradients boundary colors. */
    
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
