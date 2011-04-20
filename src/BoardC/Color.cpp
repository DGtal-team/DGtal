/* -*- mode: c++ -*- */
/**
 * @file   Color.cpp
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 20 avril 2011
 * 
 * @brief  
 */

#include "Color.h"

namespace LibBoard {

const Color Color::None(false);
const Color Color::Black((unsigned char)0,(unsigned char)0,(unsigned char)0);
const Color Color::Gray((unsigned char)128,(unsigned char)128,(unsigned char)128);
const Color Color::White((unsigned char)255,(unsigned char)255,(unsigned char)255);
const Color Color::Red((unsigned char)255,(unsigned char)0,(unsigned char)0);
const Color Color::Green((unsigned char)0,(unsigned char)128,(unsigned char)0);
const Color Color::Lime((unsigned char)0,(unsigned char)255,(unsigned char)0);
const Color Color::Blue((unsigned char)0,(unsigned char)0,(unsigned char)255);
const Color Color::Cyan((unsigned char)0,(unsigned char)255,(unsigned char)255);
const Color Color::Magenta((unsigned char)255,(unsigned char)0,(unsigned char)255);
const Color Color::Yellow((unsigned char)255,(unsigned char)255,(unsigned char)0);
const Color Color::Silver((unsigned char)190,(unsigned char)190,(unsigned char)190);
const Color Color::Purple((unsigned char)128,(unsigned char)128,(unsigned char)128);
const Color Color::Navy((unsigned char)0,(unsigned char)0,(unsigned char)128);
const Color Color::Aqua((unsigned char)0,(unsigned char)255,(unsigned char)255);

bool
Color::operator==( const Color & other ) const
{
  return _red == other._red 
    && _green == other._green
    && _blue == other._blue
    && _alpha == other._alpha;
}

bool
Color::operator!=( const Color & other ) const
{
  return _red != other._red  
    || _green != other._green
    || _blue != other._blue
    || _alpha != other._alpha;
}

} // namespace
