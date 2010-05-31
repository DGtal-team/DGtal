/* -*- mode: c++ -*- */
/**
 * @file   ShapeList.h
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Sat Aug 18 2007
 * 
 * @brief  Classes ShapeList and Group
 *
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
#ifndef _BOARD_SHAPELIST_H_
#define _BOARD_SHAPELIST_H_

#include "board/Shapes.h"
#include "board/Tools.h"

namespace LibBoard {

/**
 * The ShapeList structure.
 * @brief A group of shapes
 */
struct ShapeList : public Shape {
  
  ShapeList( int depth = -1 )
    : Shape( Color::None, Color::None, 1.0, ButtCap, MiterJoin, depth ),
      _nextDepth( std::numeric_limits<int>::max() - 1 )
  { }

  ShapeList( const ShapeList & other );

  ~ShapeList();

  /** 
   * Returns the generic name of the shape (e.g., Circle, Rectangle, etc.)
   * 
   * @return 
   */
  const std::string & name() const;
  
  ShapeList & clear();

  Point center() const;

  Shape & rotate( double angle, const Point & center );

  ShapeList rotated( double angle, const Point & center );

  Shape & rotate( double angle );

  ShapeList rotated( double angle );

  Shape & translate( double dx, double dy );

  ShapeList translated( double dx, double dy );

  Shape & scale( double sx, double sy );

  Shape & scale( double s );

  ShapeList scaled( double sx, double sy );

  ShapeList scaled( double s );

  /** 
   * Scales all the values (positions, dimensions, etc.) associated
   * with the shape.
   * 
   * @param s The scaling factor.
   */
  void scaleAll( double s );
  
  void flushPostscript( std::ostream & stream,
			const TransformEPS & transform ) const;
  
  void flushFIG( std::ostream & stream,
		 const TransformFIG & transform,
		 std::map<Color,int> & colormap ) const;

  void flushSVG( std::ostream & stream,
		 const TransformSVG & transform ) const;

  Rect boundingBox() const;
  
  virtual int minDepth() const;

  virtual int maxDepth() const;

  void shiftDepth( int shift );

  Shape * clone() const;
  
  ShapeList & operator=( const ShapeList & other );

  /** 
   * Adds a shape to the shape list. If the shape has no given depth
   * or is a compound shape (ShapeList) then it is placed on top of
   * the shapes stack. Otherwise, the shape depth is left unchanged.
   * 
   * @param shape 
   * 
   * @return 
   */
  ShapeList & operator<<( const Shape & shape ); 

  /** 
   * Adds a shape to the list of shape, always preserving the shape's depth.
   * 
   * @param shape 
   * 
   * @return 
   */
  ShapeList & operator+=( const Shape & shape ); 
  
  /** 
   * Insert the shape at a given depth. If the shape is ShapeList or a Board, 
   * then all shapes above it will be shifted.
   * 
   * @param shape 
   * @param depth 
   * 
   * @return 
   */
  ShapeList & insert( const Shape & shape, int depth );

  /** 
   * Duplicates the last inserted shape.
   * 
   * @param copies The number of copies.
   */
  ShapeList & dup( unsigned int copies  = 1 );
  
  /** 
   * Return the last inserted shape with its actual type, if specified (otherwise, a Shape &).
   * 
   * @param position The position. 0 is the last inserted shape, 1 is the one before, etc.
   * @return A reference to the addressed shape.
   */
  template<typename T>
  T & last( const unsigned int position = 0 );

  Shape & last( const unsigned int position = 0 );

  /** 
   * Convenience function that simply calls last(0).
   * 
   * @param position 
   * 
   * @return 
   */
  Shape & top();

private:
  static const std::string _name; /**< The generic name of the shape. */
  
protected:

  void addShape( const Shape & shape, double scaleFactor );

  std::vector<Shape*> _shapes;	/**< The vector of shapes. */
  int _nextDepth;		/**< The depth of the next figure to be added.  */

  /** 
   * Free the memory used by the shapes in the shape vector.
   */
  void free();
};

/**
 * The Group structure.
 * @brief A group of shapes. A group is basically a ShapeList except that
 * when rendered in either an SVG of a FIG file, it is a true compound element.
 */
struct Group : public ShapeList {
  
  Group( int depth = -1 )
    : ShapeList( depth ), _clippingPath( true /* closed path */ ) { }
  
  Group( const Group & other ) 
    : ShapeList( other ), _clippingPath( other._clippingPath ) { }

  ~Group() { };
  
  /** 
   * Returns the generic name of the shape (e.g., Circle, Rectangle, etc.)
   * 
   * @return 
   */
  const std::string & name() const;

  Shape & rotate( double angle, const Point & center );

  Shape & rotate( double angle );
  
  Group rotated( double angle, const Point & center );
  
  Group rotated( double angle );
  
  Shape & translate( double dx, double dy );
  
  Group translated( double dx, double dy );
  
  Shape & scale( double sx, double sy );
  
  Shape & scale( double s );
  
  Group scaled( double sx, double sy );
  
  Group scaled( double s );
  
  /** 
   * Define a clipping rectangle for the group.
   * 
   * @param x 
   * @param y 
   * @param width 
   * @param height 
   */
  void setClippingRectangle(  float x, float y, 
			      float width, float height );

  /** 
   * Define a clipping path for the group.
   * 
   * @param points A path.
   */
  void setClippingPath(  const std::vector<Point> & points  );

  /** 
   * Define a clipping path for the group.
   * 
   * @param points A path.
   */
  void setClippingPath( const Path & path );


  void flushPostscript( std::ostream & stream,
			const TransformEPS & transform ) const;
  
  void flushFIG( std::ostream & stream,
		 const TransformFIG & transform,
		 std::map<Color,int> & colormap ) const;

  void flushSVG( std::ostream & stream,
		 const TransformSVG & transform ) const;

  Group & operator=( const Group & other );

  Shape * clone() const;

  Rect boundingBox() const;

private:
  static const std::string _name; /**< The generic name of the shape. */
  Path _clippingPath;
  static unsigned int _clippingCount;
};

template<typename T>
T &
ShapeList::last( const unsigned int position )
{
  if ( position < _shapes.size() ) {
    std::vector<Shape*>::reverse_iterator it = _shapes.rbegin() + position;
    return dynamic_cast<T&>( *(*it) );
} else {
  error << "Trying to access an element that does not exist (" 
	<< position << "/" << _shapes.size() << ").\n";
  throw -1;
 }
}

} // namespace LibBoard

#endif /* _SHAPELIST_H_ */

