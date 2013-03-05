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
 * @file DigitalShapesDecorator.h
 * @author Jeremy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2012/08/28
 *
 * This file is part of the DGtal library.
 */

#if defined(DigitalShapesDecorator_RECURSES)
#error Recursive header files inclusion detected in DigitalShapesDecorator.h
#else // defined(DigitalShapesDecorator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalShapesDecorator_RECURSES

#if !defined DigitalShapesDecorator_h
/** Prevents repeated inclusion of headers. */
#define DigitalShapesDecorator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/shapes/CDigitalBoundedShape.h"
#include "DGtal/shapes/CDigitalOrientedShape.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// template class DigitalShapesUnion
/**
 * Description of template class 'DigitalShapesUnion' <p>
 * \brief Aim: Union between two models of CDigitalBoundedShape and CDigitalOrientedShape
 *
 * @tparam ShapeA type of the first shape. Must be a model of CDigitalBoundedShape and CDigitalOrientedShape
 * @tparam ShapeB type of the second shape. Must be a model of CDigitalBoundedShape and CDigitalOrientedShape
 */
template <typename ShapeA, typename ShapeB>
class DigitalShapesUnion
{
  // ----------------------- Standard services ------------------------------
public:
  BOOST_CONCEPT_ASSERT (( CDigitalBoundedShape< ShapeA > ));
  BOOST_CONCEPT_ASSERT (( CDigitalOrientedShape< ShapeA > ));
  BOOST_CONCEPT_ASSERT (( CDigitalBoundedShape< ShapeB > ));
  BOOST_CONCEPT_ASSERT (( CDigitalOrientedShape< ShapeB > ));

  typedef typename ShapeA::Space Space;
  typedef typename ShapeA::Point Point;

  /**
    * Constructor.
    *
    * @param a a model of CDigitalBoundedShape and CDigitalOrientedShape
    * @param b a model of CDigitalBoundedShape and CDigitalOrientedShape
    */
  DigitalShapesUnion( const ShapeA & a, const ShapeB & b )
    : myShapeA(a),
      myShapeB(b)
  {
    Point shapeALowerBoundary = myShapeA.getLowerBound();
    Point shapeBLowerBoundary = myShapeB.getLowerBound();
    Point shapeAUpperBoundary = myShapeA.getUpperBound();
    Point shapeBUpperBoundary = myShapeB.getUpperBound();
    for ( unsigned int i = 0; i < myLowerBound.size(); ++i )
    {
      myLowerBound[i] = std::min( shapeALowerBoundary[i], shapeBLowerBoundary[i] );
      myUpperBound[i] = std::max( shapeAUpperBoundary[i], shapeBUpperBoundary[i] );
    }
  }


  /**
   * @param p any point in the digital plane.
   *
   * @return 'true' if the point is inside the shape, 'false' if it
   * is strictly outside.
   */
  bool operator()( const Point & p ) const
  {
    return myShapeA( p ) || myShapeB( p );
  }

  /**
   * @return the lower bound of the shape bounding box.
   *
   */
  Point getLowerBound() const
  {
    return myLowerBound;
  }

  /**
   * @return the upper bound of the shape bounding box.
   *
   */
  Point getUpperBound() const
  {
    return myUpperBound;
  }

  /**
   * Return the orientation of a point with respect to a shape.
   *
   * @param p input point
   *
   * @return the orientation of the point (<0 means inside, ...)
   */
  Orientation orientation( const Point & p) const
  {
      if (  myShapeA.orientation( p ) == INSIDE ||  myShapeB.orientation( p ) == INSIDE )
      {
          return INSIDE;
      }
      else if ( myShapeA.orientation( p ) == ON ||  myShapeB.orientation( p ) == ON )
      {
          return ON;
      }
      return OUTSIDE;
  }

  /**
   * Destructor.
   */
  ~DigitalShapesUnion(){}

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

  // ------------------------- Hidden services ------------------------------
protected:

  /**
   * Constructor.
   * Forbidden by default (protected to avoid g++ warnings).
   */
  DigitalShapesUnion();

private:

  /**
   * Copy constructor.
   * @param other the object to clone.
   * Forbidden by default.
   */
  DigitalShapesUnion ( const DigitalShapesUnion & other );

  /**
   * Assignment.
   * @param other the object to copy.
   * @return a reference on 'this'.
   * Forbidden by default.
   */
  DigitalShapesUnion & operator= ( const DigitalShapesUnion & other );

  // ------------------------- Internals ------------------------------------
private:
  const ShapeA & myShapeA;
  const ShapeB & myShapeB;

  Point myLowerBound;
  Point myUpperBound;

}; // end of class DigitalShapesUnion

/////////////////////////////////////////////////////////////////////////////
// template class DigitalShapesIntersection
/**
 * Description of template class 'DigitalShapesIntersection' <p>
 * \brief Aim: Intersection between two models of CDigitalBoundedShape and CDigitalOrientedShape
 *
 * @tparam ShapeA type of the first shape. Must be a model of CDigitalBoundedShape and CDigitalOrientedShape
 * @tparam ShapeB type of the second shape. Must be a model of CDigitalBoundedShape and CDigitalOrientedShape
 */
template <typename ShapeA, typename ShapeB>
class DigitalShapesIntersection
{
  // ----------------------- Standard services ------------------------------
public:
  BOOST_CONCEPT_ASSERT (( CDigitalBoundedShape< ShapeA > ));
  BOOST_CONCEPT_ASSERT (( CDigitalOrientedShape< ShapeA > ));
  BOOST_CONCEPT_ASSERT (( CDigitalBoundedShape< ShapeB > ));
  BOOST_CONCEPT_ASSERT (( CDigitalOrientedShape< ShapeB > ));

  typedef typename ShapeA::Space Space;
  typedef typename ShapeA::Point Point;

  /**
    * Constructor.
    *
    * @param a a model of CDigitalBoundedShape and CDigitalOrientedShape
    * @param b a model of CDigitalBoundedShape and CDigitalOrientedShape
    */
  DigitalShapesIntersection( const ShapeA & a, const ShapeB & b )
    : myShapeA(a),
      myShapeB(b)
  {
    Point shapeALowerBoundary = myShapeA.getLowerBound();
    Point shapeBLowerBoundary = myShapeB.getLowerBound();
    Point shapeAUpperBoundary = myShapeA.getUpperBound();
    Point shapeBUpperBoundary = myShapeB.getUpperBound();
    for ( unsigned int i = 0; i < myLowerBound.size(); ++i )
    {
      myLowerBound[i] = std::min( shapeALowerBoundary[i], shapeBLowerBoundary[i] );
      myUpperBound[i] = std::max( shapeAUpperBoundary[i], shapeBUpperBoundary[i] );
    }
  }

  /**
   * @param p any point in the digital plane.
   *
   * @return 'true' if the point is inside the shape, 'false' if it
   * is strictly outside.
   */
  bool operator()( const Point & p ) const
  {
    return myShapeA( p ) && myShapeB( p );
  }

  /**
   * @return the lower bound of the shape bounding box.
   *
   */
  Point getLowerBound() const
  {
    return myLowerBound;
  }

  /**
   * @return the upper bound of the shape bounding box.
   *
   */
  Point getUpperBound() const
  {
    return myUpperBound;
  }

  /**
   * Return the orientation of a point with respect to a shape.
   *
   * @param p input point
   *
   * @return the orientation of the point (<0 means inside, ...)
   */
  Orientation orientation( const Point & p) const
  {
      if ( myShapeA.orientation( p ) == ON )
      {
        if ( myShapeB.orientation( p ) == INSIDE || myShapeB.orientation( p ) == ON )
        {
          return ON;
        }
      }
      else if ( myShapeB.orientation( p ) == ON )
      {
        if ( myShapeA.orientation( p ) == INSIDE )
        {
          return ON;
        }
      }
      else if ( myShapeA.orientation( p ) == INSIDE && myShapeB.orientation( p ) == INSIDE )
      {
        return INSIDE;
      }

      return OUTSIDE;
  }


  /**
   * Destructor.
   */
  ~DigitalShapesIntersection(){}

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

  // ------------------------- Hidden services ------------------------------
protected:

  /**
   * Constructor.
   * Forbidden by default (protected to avoid g++ warnings).
   */
  DigitalShapesIntersection();

private:

  /**
   * Copy constructor.
   * @param other the object to clone.
   * Forbidden by default.
   */
  DigitalShapesIntersection ( const DigitalShapesIntersection & other );

  /**
   * Assignment.
   * @param other the object to copy.
   * @return a reference on 'this'.
   * Forbidden by default.
   */
  DigitalShapesIntersection & operator= ( const DigitalShapesIntersection & other );

  // ------------------------- Internals ------------------------------------
private:
  const ShapeA & myShapeA;
  const ShapeB & myShapeB;

  Point myLowerBound;
  Point myUpperBound;

}; // end of class DigitalShapesIntersection

/////////////////////////////////////////////////////////////////////////////
// template class DigitalShapesMinus
/**
 * Description of template class 'DigitalShapesMinus' <p>
 * \brief Aim: Minus between two models of CDigitalBoundedShape and CDigitalOrientedShape
 *
 * @tparam ShapeA type of the first shape. Must be a model of CDigitalBoundedShape and CDigitalOrientedShape
 * @tparam ShapeB type of the second shape. Must be a model of CDigitalBoundedShape and CDigitalOrientedShape
 */
template <typename ShapeA, typename ShapeB>
class DigitalShapesMinus
{
  // ----------------------- Standard services ------------------------------
public:
  BOOST_CONCEPT_ASSERT (( CDigitalBoundedShape< ShapeA > ));
  BOOST_CONCEPT_ASSERT (( CDigitalOrientedShape< ShapeA > ));
  BOOST_CONCEPT_ASSERT (( CDigitalBoundedShape< ShapeB > ));
  BOOST_CONCEPT_ASSERT (( CDigitalOrientedShape< ShapeB > ));

  typedef typename ShapeA::Space Space;
  typedef typename ShapeA::Point Point;

  /**
    * Constructor.
    *
    * @param a a model of CDigitalBoundedShape and CDigitalOrientedShape
    * @param b a model of CDigitalBoundedShape and CDigitalOrientedShape
    */
  DigitalShapesMinus( const ShapeA & a, const ShapeB & b )
    : myShapeA(a),
      myShapeB(b)
  {
    Point shapeALowerBoundary = myShapeA.getLowerBound();
    Point shapeBLowerBoundary = myShapeB.getLowerBound();
    Point shapeAUpperBoundary = myShapeA.getUpperBound();
    Point shapeBUpperBoundary = myShapeB.getUpperBound();
    for ( unsigned int i = 0; i < myLowerBound.size(); ++i )
    {
      myLowerBound[i] = std::min( shapeALowerBoundary[i], shapeBLowerBoundary[i] );
      myUpperBound[i] = std::max( shapeAUpperBoundary[i], shapeBUpperBoundary[i] );
    }
  }


  /**
   * @param p any point in the digital plane.
   *
   * @return 'true' if the point is inside the shape, 'false' if it
   * is strictly outside.
   */
  bool operator()( const Point & p ) const
  {
    return myShapeA( p ) && !myShapeB.orientation(p) == INSIDE;
  }

  /**
   * @return the lower bound of the shape bounding box.
   *
   */
  Point getLowerBound() const
  {
    return myLowerBound;
  }

  /**
   * @return the upper bound of the shape bounding box.
   *
   */
  Point getUpperBound() const
  {
    return myUpperBound;
  }

  /**
   * Return the orientation of a point with respect to a shape.
   *
   * @param p input point
   *
   * @return the orientation of the point (<0 means inside, ...)
   */
  Orientation orientation( const Point & p) const
  {
      if ( myShapeA.orientation( p ) == INSIDE )
      {
        if ( myShapeB.orientation( p ) == ON )
        {
          return ON;
        }
        else if ( myShapeB.orientation( p ) == INSIDE )
        {
          return OUTSIDE;
        }

        return INSIDE;
      }
      else if ( myShapeA.orientation( p ) == ON )
      {
        if ( myShapeB.orientation( p ) == INSIDE )
        {
          return OUTSIDE;
        }

        return ON;
      }

      return OUTSIDE;
  }


  /**
   * Destructor.
   */
  ~DigitalShapesMinus(){}

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

  // ------------------------- Hidden services ------------------------------
protected:

  /**
   * Constructor.
   * Forbidden by default (protected to avoid g++ warnings).
   */
  DigitalShapesMinus();

private:

  /**
   * Copy constructor.
   * @param other the object to clone.
   * Forbidden by default.
   */
  DigitalShapesMinus ( const DigitalShapesMinus & other );

  /**
   * Assignment.
   * @param other the object to copy.
   * @return a reference on 'this'.
   * Forbidden by default.
   */
  DigitalShapesMinus & operator= ( const DigitalShapesMinus & other );

  // ------------------------- Internals ------------------------------------
private:
  const ShapeA & myShapeA;
  const ShapeB & myShapeB;

  Point myLowerBound;
  Point myUpperBound;

}; // end of class DigitalShapesMinus

  /**
   * Overloads 'operator<<' for displaying objects of class 'DigitalShapesDecorator'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DigitalShapesDecorator' to write.
   * @return the output stream after the writing.
   */
  template <typename ShapeA, typename ShapeB>
  std::ostream&
  operator<< ( std::ostream & out, const DigitalShapesUnion< ShapeA, ShapeB > & object );

  template <typename ShapeA, typename ShapeB>
  std::ostream&
  operator<< ( std::ostream & out, const DigitalShapesIntersection< ShapeA, ShapeB > & object );

  template <typename ShapeA, typename ShapeB>
  std::ostream&
  operator<< ( std::ostream & out, const DigitalShapesMinus< ShapeA, ShapeB > & object );

} // namespace DGtal



#endif // !defined DigitalShapesDecorator_h

#undef DigitalShapesDecorator_RECURSES
#endif // else defined(DigitalShapesDecorator_RECURSES)
