/* -*- mode: c++ -*- */
/**
 * @file   Color.cpp
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Sat Aug 18 2007
 * 
 * @brief  
 * @copyright
 * This source code is part of the Board project, a C++ library whose
 * purpose is to allow simple drawings in EPS, FIG or SVG files.
 * Copyright (C) 2007 Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb/>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. 
 * This source code is part of the Board project, a C++ library whose
 * purpose is to allow simple drawings in EPS, FIG or SVG files.
 * Copyright (C) 2007 Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb/>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. 
 */
#include "board/Color.h"
#include <cmath>
using std::string;

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

Color::Color( const unsigned int rgb, unsigned char alpha )
 :_alpha( alpha )
{
  _red = ( rgb & 0xFF0000u ) >> 16;
  _green = ( rgb & 0xFF00u ) >> 8;
  _blue = rgb & 0xFF;
}

Color &
Color::setRGBf( float red,
		float green,
		float blue,
		float alpha  ) {
  if ( red > 1.0f ) red = 1.0f;
  if ( red < 0.0f ) red = 0.0f;
  _red = static_cast<unsigned char>( 255 * red );
  if ( green > 1.0f ) green = 1.0f;
  if ( green < 0.0f ) green = 0.0f;
  _green = static_cast<unsigned char>( 255 * green );
  if ( blue > 1.0f ) blue = 1.0f;
  if ( blue < 0.0f ) blue = 0.0f;
  _blue = static_cast<unsigned char>( 255 * blue );
  if ( alpha > 1.0f ) alpha = 1.0f;
  if ( alpha < 0.0f ) alpha = 0.0f;
  _alpha = static_cast<unsigned char>( 255 * alpha );
  return *this;
}

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

bool
Color::operator<( const Color & other ) const
{
  if ( _red < other._red )
    return true;
  if ( _red == other._red ) {
    if ( _green < other._green )
      return true;
    if ( _green == other._green ) { 
      if ( _blue < other._blue )
	return true;
      if ( _blue == other._blue )
	return _alpha < other._alpha;
    }
  }
  return false;
}

void
Color::flushPostscript( std::ostream & stream ) const
{
  stream << (_red/255.0) << " "
	 << (_green/255.0) << " "
	 << (_blue/255.0) << " srgb\n";
}

string
Color::postscript() const
{
  char buffer[255];
  sprintf( buffer, "%.4f %.4f %.4f", _red/255.0, _green/255.0, _blue/255.0 );
  return buffer;
}

string
Color::svg() const
{
  char buffer[255];
  if ( *this == Color::None ) return "none";
  sprintf( buffer, "rgb(%d,%d,%d)", _red, _green, _blue );
  return buffer;
}

string
Color::svgAlpha( const char * prefix ) const
{
  char buffer[255];
  if ( _alpha == 255 || *this == Color::None ) return "";
  sprintf( buffer, " %s-opacity=\"%f\"", prefix, _alpha/255.0f );
  return buffer;
}

} // namespace LibBoard
