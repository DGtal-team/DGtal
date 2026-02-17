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
 * @file Color.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/07/17
 *
 * Header file for module Color.cpp
 *
 * This file is part of the DGtal library.
 * Backport from Color of LibBoard library :http://libboard.sourceforge.net
 */

#if defined(Color_RECURSES)
#error Recursive header files inclusion detected in Color.h
#else // defined(Color_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Color_RECURSES

#if !defined Color_h
/** Prevents repeated inclusion of headers. */
#define Color_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <DGtal/base/BasicTypes.h>
#include <iostream>
#include <algorithm>
#include <array>
#include <string>
//////////////////////////////////////////////////////////////////////////////

#ifndef secured_sprintf
#if defined( WIN32 )
#define secured_sprintf sprintf_s
#else
#include <stdio.h>
#define secured_sprintf snprintf
#endif // defined( WIN32 )
#endif

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class Color
  /**
   * Description of class 'Color' <p>
   *
   * @brief Structure representing an RGB triple with alpha component.
   *
   * @note if compilation flag COLOR_WITH_ALPHA_ARITH is set, then the
   * arithmetical operations on colors also consider the
   * alpha-channel. Otherwise, the alpha channel is not changed when
   * summing up to colors for instance.
   *
   */
  class Color
  {
    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~Color() = default;

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Constructor.
     *
     * @param aRgb an unsigned int representing the color.
     * @param aAlpha color transparency (default value =255);
     */

    Color( const unsigned int aRgb,
           unsigned char aAlpha = 255 );

    /**
     * Copy Constructor.
     *
     * @param aColor the color to copy.
     */

    Color( const Color &aColor ) = default;

    /**
     * Constructor from R, G, B and Alpha parameter.
     *
     * @param aRedValue  red component
     * @param aGreenValue green component
     * @param aBlueValue blue component
     * @param aAlphaValue color transparency.
     */
    Color( const unsigned char aRedValue,
           const unsigned char  aGreenValue,
           const unsigned char  aBlueValue,
           const unsigned char aAlphaValue = 255 )
      : myRed(aRedValue),myGreen(aGreenValue),myBlue(aBlueValue),myAlpha(aAlphaValue) { }


    /**
     * Constructor from gray scale value.
     *
     * @param aGrayValue the color gray value.
     * @param aAlphaValue color transparency (default value =255);.
     */

    Color( unsigned char aGrayValue,
           unsigned char aAlphaValue = 255 )
      : myRed(aGrayValue),myGreen(aGrayValue), myBlue(aGrayValue), myAlpha(aAlphaValue) { }


    /**
     * Default Constructor.
     *
     */

    Color( )
      : myRed(0),myGreen(0),myBlue(0), myAlpha(255)
    {
    }


    void red( const unsigned char aRedValue );

    void green( const unsigned char aGreenValue );

    void blue( const unsigned char aBlueValue );

    void alpha( const unsigned char aAlphaValue );

    unsigned char red() const ;

    unsigned char green() const ;

    unsigned char blue() const ;

    unsigned char alpha() const ;


    double r() const ;

    double g() const ;

    double b() const ;

    double a() const ;


    Color& setRGBi( const unsigned char aRedValue,
                   const unsigned char aGreenValue,
                   const unsigned char aBlueValue,
                   const unsigned char aAlphaValue = 255);

    /**
     * Set the color parameter from an unsigned integer coding each canal.
     *
     * @param aRGBA an unsigned integer on 32 bits(DGtal::unit32_t)
     * representing the color coded with 4 bits on each components R, G, B
     * and Alpha value.
     * @return a reference on the itself.
     *
     */
    Color& setRGBA( DGtal::uint32_t aRGBA );


    /// Set the color from HSV values
    /// @param h hue
    /// @param s saturation
    /// @param v value
    /// @return the color
    Color& setFromHSV( const double h, const double s, const double v)
    {
      double r,g,b;
      Color::HSVtoRGB(r,g,b,h,s,v);
      return this->setRGBf((float)r,(float)g,(float)b);
    }
    /// @return the HSV values of a DGtal::Color (array of three doubles).
    ///
    std::array<double, 3> getHSV() const
    {
      double h,s,v;
      Color::RGBtoHSV(h,s,v, this->red(), this->green(), this->blue());
      return {h,s,v};
    }

    /**
     * @return the unsigned integer ( DGtal::uint32_t ) coding  each
     * R, G, B canal on 8 bits starting from least significant bit.
     **/

    DGtal::uint32_t getRGB() const;

    /**
     * @return the unsigned integer ( DGtal::uint32_t ) coding  each
     * R, G, B, A canal on 8 bits starting from least significant bit.
     **/

    DGtal::uint32_t getRGBA() const;




    bool valid() const;


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



    Color & setRGBf( float red,
		     float green,
		     float blue,
		     float alpha = 1.0 );

    bool operator==( const Color & aColor ) const;

    bool operator!=( const Color & aColor ) const;

    bool operator<( const Color & aColor ) const;

    bool operator>( const Color & aColor ) const;

    bool operator<=( const Color & aColor ) const;

    bool operator>=( const Color & aColor ) const;


    /**
     * Addition operator with assignment.
     *
     * @note returned components are clamped to [0,255] interval.
     *
     * @param v is the Color that gets added to @a *this.
     * @return a reference on 'this'.
     */
    Color & operator+= ( const Color & v )
    {
      this->myRed = clamp((int)this->myRed + (int)v.myRed);
      this->myBlue =  clamp((int)this->myBlue + (int)v.myBlue);
      this->myGreen =  clamp((int)this->myGreen + (int)v.myGreen);
#ifdef COLOR_WITH_ALPHA_ARITH
      this->myAlpha =  clamp((int)this->myAlpha + (int)v.myAlpha);
#endif
      return *this;
    }

    /**
     * Addition operator.
     *
     * @note returned components are clamped to [0,255] interval.
     *
     *
     * @param v is the Color that gets added to @a *this.
     * @return a new Point that is the addition of 'this' to [v].
     */
    Color operator+ ( const Color & v ) const
    {
      Color c;
      c.myRed = clamp((int)this->myRed + (int)v.myRed);
      c.myBlue =clamp((int)this->myBlue + (int)v.myBlue);
      c.myGreen = clamp((int)this->myGreen + (int)v.myGreen);
#ifdef COLOR_WITH_ALPHA_ARITH
      c.myAlpha = clamp((int)this->myAlpha + (int)v.myAlpha);
#else
      c.myAlpha = this->myAlpha ;
#endif
  return c;
    }

    /**
     * Subtraction operator with assignment.
     *
     * @note returned components are clamped to [0,255] interval.
     *
     *
     * @param v is the Point that gets subtracted to  *this.
     * @return a reference on 'this'.
     */
    Color & operator-= ( const Color & v )
    {
      this->myRed = clamp((int)this->myRed - (int)v.myRed);
      this->myBlue = clamp((int)this->myBlue - (int)v.myBlue);
      this->myGreen = clamp((int)this->myGreen - (int)v.myGreen);
#ifdef COLOR_WITH_ALPHA_ARITH
      this->myAlpha = clamp((int)this->myAlpha - (int)v.myAlpha);
#endif
      return *this;
    }

    /**
     * Subtraction operator.
     *
     * @note returned components are clamped to [0,255] interval.
     *
     * @param v is the Color that gets substacted to @a *this.
     * @return a new Point that is the subtraction 'this'-[v].
     */
    Color operator- ( const Color & v ) const
    {
      Color c;
      c.myRed = clamp((int)this->myRed - (int)v.myRed);
      c.myBlue = clamp((int)this->myBlue - (int)v.myBlue);
      c.myGreen = clamp((int)this->myGreen - (int)v.myGreen);
#ifdef COLOR_WITH_ALPHA_ARITH
      c.myAlpha = clamp((int)this->myAlpha - (int)v.myAlpha);
#else
      c.myAlpha = this->myAlpha ;
#endif
      return c;
    }

    /**
     * Multiplication by a scalar (component-wise)
     *
     * @note returned components are clamped to [0,255] interval.
     *
     *
     * @param coeff the scalar
     *
     * @return the scaled color
     */
    Color &operator *= ( const double coeff)
    {
      this->myRed = clamp((double)this->myRed*coeff);
      this->myBlue = clamp((double)this->myBlue*coeff);
      this->myGreen = clamp((double)this->myGreen*coeff);
#ifdef COLOR_WITH_ALPHA_ARITH
      this->myAlpha =  clamp((double)this->myAlpha*coeff);
#endif
      return *this;
    }

    /**
     * Multiplication by a scalar (component-wise)
     *
     * @note returned components are clamped to [0,255] interval.
     *
     * @param coeff the scalar.
     * @return a scaled color
     */
    Color operator * ( const double coeff) const
    {
      Color c;
      c.myRed = clamp((double)this->myRed*coeff);
      c.myBlue = clamp((double)this->myBlue*coeff);
      c.myGreen = clamp((double)this->myGreen*coeff);
#ifdef COLOR_WITH_ALPHA_ARITH
      c.myAlpha =  clamp((double)this->myAlpha*coeff);
#else
      c.myAlpha  = this->myAlpha;
#endif
      return c;
    }

    /**
     * Assignment Operator
     *
     * @param pv the object to copy.
     * @return a reference on 'this'.
     */
    Color & operator= ( const Color & pv ) = default;

    void flushPostscript( std::ostream & ) const;

    std::string svg() const;

    /**
     * Return a an SVG parameter string for the opacity value.
     *
     * @param aPrefix A prefix string to be appended to the returned
     * string if not empty.
     *
     * @return An empty string if alpha == 255, otherwise the string \<prefix\>-opacity="<alpha-value>".
     */
    std::string svgAlpha( const char * aPrefix ) const;

    std::string postscript() const;

    /**
     * Return a string representation of the color usable in TikZ commands.
     * Use the corresponding named color (or a mixture of a named color and black)
     * for predefined colors. Use a mixture of red, green and blue for general
     * colors.
     *
     * @return a string representation of the color usable in TikZ commands.
     */
    std::string tikz() const;

    static const Color None;
    static const Color Black;
    static const Color Gray;
    static const Color White;
    static const Color Red;
    static const Color Green;
    static const Color Lime;
    static const Color Blue;
    static const Color Cyan;
    static const Color Magenta;
    static const Color Yellow;
    static const Color Silver;
    static const Color Purple;
    static const Color Navy;
    static const Color Aqua;

    // ------------------------- Protected Data ------------------------------
  private:

    // ------------------------- Private Data --------------------------------
  private:
    unsigned char myRed;      /**< The red component. */
    unsigned char myGreen;    /**< The green component. */
    unsigned char myBlue;      /**< The blue component. */
    unsigned char myAlpha;    /**< The opacity. */
    // ------------------------- Hidden services ------------------------------
  protected:


  private:

    /**
     * Clamp an int to [0,255]
     *
     * @param [in] value the value to clamp
     *
     * @return the clamped value
     */
    unsigned char clamp(const double value)  const
    {
      return static_cast<unsigned char>(std::max( std::min(value, 255.0), 0.0));
    }


    // ----------------------- Static methods ---------------------------------
  public:
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

    /**
     * Converts a color from the RGB space to the HSV (Hue,Saturation,Value) space.
     *
     * @param h (out) The hue of the color in [0..360)
     * @param s (out) The saturation of the color in [0..1].
     * @param v (out) The value of the color in [0..1].
     * @param r The red component.
     * @param g The green component.
     * @param b The blue component.
     */
    static void RGBtoHSV( double & h, double & s, double & v,
        const unsigned char r,
        const unsigned char g,
        const unsigned char b );


    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class Color



  /**
     External multiplication operator with a scalar number

     @param coeff is the factor \a Color is multiplied by.
     @param aColor is the vector that is multiplied by the factor \a coef.

     @return a new Vector that is the multiplication of \a aVector by
     \a coeff.
  */
  Color
  operator*( const double coeff,
	     const Color &aColor );



  /**
   * Overloads 'operator<<' for displaying objects of class 'Color'.
   * @param out the output stream where the object is written.
   * @param aColor the object of class 'Color' to write.
   * @return the output stream after the writing.
   */
  std::ostream&
  operator<< ( std::ostream & out, const Color & aColor );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods
#include "DGtal/io/Color.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Color_h

#undef Color_RECURSES
#endif // else defined(Color_RECURSES)
