/* -*- mode: c++ -*- */
/**
 * @file   Color.h
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 20 avril 2011
 * 
 * @brief  
 */

#ifndef _BOARDCAIRO_COLOR_H_
#define _BOARDCAIRO_COLOR_H_

namespace LibBoard {

/**
 * The Color class.
 * @brief Structure representing an RGB triple.
 */
class Color {
  
public:
  
  Color( unsigned char redValue, unsigned char greenValue, unsigned char blueValue,
	 unsigned char alphaValue = 255 )
    : _red(redValue),_green(greenValue),_blue(blueValue),_alpha(alphaValue) { }
    
  Color( const bool validColor = true )
    : _red(-1),_green(-1),_blue(-1),_alpha(255)
  { 
    if ( validColor ) {
      _red = _green = _blue = 0;
    }
  }
  
  inline void red( unsigned char red );
  inline void green( unsigned char green );
  inline void blue( unsigned char blue );
  inline void alpha( unsigned char alpha );

  inline unsigned char red() const;
  inline unsigned char green() const;
  inline unsigned char blue() const;
  inline unsigned char alpha() const;
  
  inline Color & setRGBi( const unsigned char red,
			  const unsigned char green,
			  const unsigned char blue,
			  const unsigned char alpha = 255 );
  
  bool operator==( const Color & other ) const;
  
  bool operator!=( const Color & other ) const;
  
public:
  
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
  
private:
  
  int _red;			/**< The red component. */
  int _green;			/**< The green component. */
  int _blue;			/**< The blue component. */
  int _alpha;			/**< The opacity. */
  
};

inline void
Color::red( unsigned char redValue )
{
  _red = redValue;
}

inline void
Color::green( unsigned char greenValue )
{
  _green = greenValue;
}

inline void 
Color::blue( unsigned char blueValue )
{
  _blue = blueValue;
}

inline void 
Color::alpha( unsigned char alphaValue )
{
  _alpha = alphaValue;
}

inline unsigned char Color::red() const { return _red; } 
inline unsigned char Color::green() const { return _green; } 
inline unsigned char Color::blue() const { return _blue; } 
inline unsigned char Color::alpha() const { return _alpha; }

inline Color &
Color::setRGBi( const unsigned char redValue,
		const unsigned char greenValue,
		const unsigned char blueValue,
		const unsigned char alphaValue ) {
  _red = redValue;
  _green = greenValue;
  _blue = blueValue;
  _alpha = alphaValue;
  return *this;
}

} // namespace

/*
 * Inline methods
 */
//#include "Color.ih"

#endif
