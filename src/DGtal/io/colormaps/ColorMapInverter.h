#pragma once

/**
 * @file ColorMapInverter.h
 * @author Sebastien Fourey (\c Sebastien.Fourey@greyc.ensicaen.fr )
 * Groupe de Recherche en Informatique, Image, Automatique et Instrumentation de Caen - GREYC (CNRS, UMR 6072), ENSICAEN, France
 *
 * @date 2010/07/19
 *
 * Header file for module ColorMapInverter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ColorMapInverter_RECURSES)
#error Recursive header files inclusion detected in ColorMapInverter.h
#else // defined(ColorMapInverter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ColorMapInverter_RECURSES

#if !defined ColorMapInverter_h
/** Prevents repeated inclusion of headers. */
#define ColorMapInverter_h

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
  // template class ColorMapInverter
  /**
   * Description of template class 'ColorMapInverter' <p>
   * \brief Aim: This class template may be used to reverse an existing color map.
   * 
   * A ColorMapInverter can be used either as a functor object which converts a
   * value  into a LibBoard::Color structure, or it can be used through a static
   * method taking both the range and the value as parameters. In both cases, an
   * existing color map is used, whose range is simply considered from max to min.
   *
   * The code below shows a possible use of this class.
   * @code
   * #include "Board/Color.h"
   * #include "GrayScaleColorMap.h"
   * #include "ColorMapInverter.h"
   * // ...
   * {
   *   typedef GrayScaleColorMap<float> GrayMap;
   *   GrayMap grayShade(0.0f,1000.0f)
   *   ColorMapInverter<GrayMap> iGrayShade(grayShade);
   *   LibBoard::Color white = grayShade(1.0f);
   *   LibBoard::Color black = iGrayShade(1.0f);
   * }
   * @endcode
   *
   * @tparam PValueType The type of the range values.
   */
  template <typename PColorMap>
  class ColorMapInverter
  {

  public:
    
    typedef typename PColorMap::ValueType ValueType;

    // ----------------------- Standard services ------------------------------
  public:

    /** 
     * Constructor.
     * 
     * @param aColorMap A color map to be inverted.
     */
    ColorMapInverter( const PColorMap & aColorMap );

    /** 
     * Computes the colorassociated with a value in a given range.
     * 
     * @param value A value within the value range.
     * @return A color which linearly and reversely depends on the 
     * position of [value] within the range of the color map used.
     */
    LibBoard::Color operator()( const ValueType & value ) const;
      
    /**
     * Destructor.
     */
    ~ColorMapInverter();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    ColorMapInverter ( const ColorMapInverter & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    ColorMapInverter & operator= ( const ColorMapInverter & other );

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
     * Computes the color associated with a value in a given range. The range
     * is the one of the associated color map, but considered from max to min.
     * 
     * @param min The lower bound of the value range.  
     * @param max The upper bound of the value range.
     * @param value A value within the value range.
     * @return A color which linearly depends on the 
     * position of [value] within the range [min]..[max]. 
     */
    static LibBoard::Color getColor( const PColorMap & aColorMap,
				     const ValueType & aValue );
    
    // ------------------------- Protected Datas ------------------------------
  private:

    // ------------------------- Private Datas --------------------------------
  private:

    PColorMap myColorMap;

    // ------------------------- Hidden services ------------------------------
  protected:

    ValueType myMin;		/**< The lower bound of the value range.  */
    ValueType myMax;            /**< The lower bound of the value range.  */

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    ColorMapInverter();

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class ColorMapInverter


  /**
   * Overloads 'operator<<' for displaying objects of class 'ColorMapInverter'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ColorMapInverter' to write.
   * @return the output stream after the writing.
   */
  template <typename ValueType>
  std::ostream&
  operator<< ( std::ostream & out, const ColorMapInverter<ValueType> & object );
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/colormaps/ColorMapInverter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ColorMapInverter_h

#undef ColorMapInverter_RECURSES
#endif // else defined(ColorMapInverter_RECURSES)
