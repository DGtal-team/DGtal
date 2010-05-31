/* -*- mode: c++ -*- */
/**
 * @file   Path.h
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Aug 2009
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
 */
#ifndef _BOARD_PATH_H_
#define _BOARD_PATH_H_

#include "board/Point.h"
#include "board/Rect.h"
#include "board/Transforms.h"
#include <vector>
#include <iostream>

namespace LibBoard {



/**
 * The path structure.
 * @brief A path, according to Postscript and SVG definition.
 */
struct Path { 

  Path() : _closed( false ) { }

  Path( const std::vector<Point> & points, bool closed )
    : _points( points ), _closed( closed ) { }

  Path( bool closed ) : _closed( closed ) { }    
  
  inline void clear();

  inline bool closed() const;

  inline bool empty() const;

  inline unsigned int size() const;

  inline void setClosed( bool closed  );
  
  /** 
   * Barycenter of the path
   * @return 
   */
  Point center() const;

  /** 
   * Add a point at the end of the path.
   * 
   * @param p 
   * 
   * @return 
   */
  Path & operator<<( const Point & p );

  /** 
   * 
   * 
   * 
   * @return 
   */
  Path & pop_back();

  /** 
   * Returns the n-th point of the polyline.
   * 
   * @param i 
   * 
   * @return 
   */
  Point & operator[]( const unsigned int n ) {
    return _points[ n ];
  }

  /** 
   * Returns the n-th point of the polyline.
   * 
   * @param i 
   * 
   * @return 
   */
  const Point & operator[]( const unsigned int n ) const {
    return _points[ n ];
  }

  /** 
   * 
   * 
   * @param angle 
   * @param center 
   * 
   * @return 
   */
  Path & rotate( double angle, const Point & center );

  /** 
   * 
   * 
   * @param angle 
   * @param center 
   * 
   * @return 
   */
  Path rotated( double angle, const Point & center ) const;

  /** 
   * 
   * 
   * @param angle 
   * 
   * @return 
   */
  Path & rotate( double angle );
  
  /** 
   * 
   * 
   * @param angle 
   * 
   * @return 
   */
  Path rotated( double angle ) const;

  /** 
   * 
   * 
   * @param dx 
   * @param dy 
   * 
   * @return 
   */
  Path & translate( double dx, double dy );
 
  /** 
   * 
   * 
   * @param dx 
   * @param dy 
   * 
   * @return 
   */
  Path translated( double dx, double dy ) const;

  /** 
   * 
   * 
   * @param sx 
   * @param sy 
   * 
   * @return 
   */
  Path & scale( double sx, double sy );

  /** 
   * 
   * 
   * @param s 
   * 
   * @return 
   */
  Path & scale( double s );
  
  /** 
   * 
   * 
   * @param sx 
   * @param sy 
   * 
   * @return 
   */
  Path scaled( double sx, double sy )  const;

  Path scaled( double s )  const;

  /** 
   * Scales all the points.
   * 
   * @param s The scaling factor.
   */
  void scaleAll( double s );

  void flushPostscript( std::ostream & stream,
			const TransformEPS & transform ) const;
  
  void flushFIG( std::ostream & stream,
		 const TransformFIG & transform ) const;
  
  void flushSVGPoints( std::ostream & stream,
		       const TransformSVG & transform ) const;

  void flushSVGCommands( std::ostream & stream,
			 const TransformSVG & transform ) const;

  Rect boundingBox() const;

protected:
  std::vector<Point> _points;
  bool _closed;
};
  
void
Path::clear()
{
  _points.clear();
}

bool
Path::closed() const
{
  return _closed;
}

bool
Path::empty() const
{
  return _points.empty();
}

unsigned int
Path::size() const
{
  return _points.size();
}

void
Path::setClosed( bool closed )
{
  _closed = closed;
}

} // namespace LibBoard  

#endif /* _PATH_H_ */

