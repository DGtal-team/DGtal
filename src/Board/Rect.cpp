/* -*- mode: c++ -*- */
/**
 * @file   Rect.cpp
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
#include "board/Rect.h"

namespace LibBoard {

Rect
operator||( const Rect & rectA, const Rect & rectB )
{
  Rect rect;
  rect.top = ( rectA.top > rectB.top ) ? rectA.top : rectB.top;
  rect.left = (rectA.left < rectB.left) ? rectA.left : rectB.left;
  if ( rectA.left + rectA.width > rectB.left + rectB.width )
    rect.width = rectA.left + rectA.width - rect.left;
  else
    rect.width = rectB.left + rectB.width - rect.left;
  if ( rectA.top - rectA.height < rectB.top - rectB.height )
    rect.height = rect.top - ( rectA.top - rectA.height );
  else
    rect.height = rect.top - ( rectB.top - rectB.height );
  return rect;
}

Rect
operator&&( const Rect & rectA, const Rect & rectB )
{
  Rect rect;
  rect.top = ( rectA.top < rectB.top ) ? rectA.top : rectB.top;
  rect.left = (rectA.left > rectB.left) ? rectA.left : rectB.left;
  if ( rectA.left + rectA.width < rectB.left + rectB.width )
    rect.width = rectA.left + rectA.width - rect.left;
  else
    rect.width = rectB.left + rectB.width - rect.left;
  if ( rectA.top - rectA.height > rectB.top - rectB.height )
    rect.height = rect.top - ( rectA.top - rectA.height );
  else
    rect.height = rect.top - ( rectB.top - rectB.height );
  if ( rect.height < 0 ) rect.height = 0;
  if ( rect.width < 0 ) rect.width = 0;
  return rect;
}

} // namespace LibBoard

std::ostream &
operator<<( std::ostream & out, const LibBoard::Rect & rect )
{
  out << "Rect(" 
      << rect.left << "," << rect.top
      << "+" << rect.width << "x" << rect.height << ")";
  return out;
}
