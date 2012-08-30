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
 * @file EuclideanShapesAdapter.h
 * @author Jeremy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/08/28
 *
 * Header file for module EuclideanShapesAdapter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(EuclideanShapesAdapter_RECURSES)
#error Recursive header files inclusion detected in EuclideanShapesAdapter.h
#else // defined(EuclideanShapesAdapter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define EuclideanShapesAdapter_RECURSES

#if !defined EuclideanShapesAdapter_h
/** Prevents repeated inclusion of headers. */
#define EuclideanShapesAdapter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/shapes/CEuclideanBoundedShape.h"
#include "DGtal/shapes/CEuclideanOrientedShape.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// template class EuclideanShapesAdapter
/**
 * Description of template class 'EuclideanShapesAdapter' <p>
 * \brief Aim:
 */

  /////////////////////////////////////////////////////////////////////////////
  // template class EuclideanShapesUnion
  /**
   * Description of template class 'EuclideanShapesUnion' <p>
   * \brief Aim:
   */
  template <typename ShapeA, typename ShapeB>
  class EuclideanShapesUnion
  {
    // ----------------------- Standard services ------------------------------
  public:
    ///@todo BOOST_CONCEPT_ASSERT (( CEuclideanBoundedShape<ShapeA> ));
    ///@todo BOOST_CONCEPT_ASSERT (( CEuclideanOrientedShape<ShapeA> ));
    typedef typename ShapeA::Space Space;
    typedef typename Space::Point Point;
    typedef typename Space::RealPoint RealPoint;

    EuclideanShapesUnion( const ShapeA & a, const ShapeB & b )
      : myShapeA(a),
        myShapeB(b)
    {
      RealPoint shapeALowerBoundary = myShapeA.getLowerBound();
      RealPoint shapeBLowerBoundary = myShapeB.getLowerBound();
      RealPoint shapeAUpperBoundary = myShapeA.getUpperBound();
      RealPoint shapeBUpperBoundary = myShapeB.getUpperBound();
      for ( unsigned int i = 0; i < myLowerBound.size(); ++i )
      {
        myLowerBound[i] = std::min( shapeALowerBoundary[i], shapeBLowerBoundary[i] );
        myUpperBound[i] = std::max( shapeAUpperBoundary[i], shapeBUpperBoundary[i] );
      }
      //myCenter = myShapeA.center()
    }

    /**
     * @param p any point in the plane.
     *
     * @return 'true' if the point is inside the shape, 'false' if it
     * is strictly outside.
     */
    bool isInside( const RealPoint & p ) const
    {
      return myShapeA.isInside( p ) || myShapeB.isInside( p );
    }

    /**
     * @param p any point in the digital plane.
     *
     * @return 'true' if the point is inside the shape, 'false' if it
     * is strictly outside.
     */
    bool isInside( const Point & p ) const
    {
      return myShapeA.isInside( p ) || myShapeB.isInside( p );
    }

    /**
     * @return the lower bound of the shape bounding box.
     *
     */
    RealPoint getLowerBound() const
    {
      return myLowerBound;
    }

    /**
     * @return the upper bound of the shape bounding box.
     *
     */
    RealPoint getUpperBound() const
    {
      return myUpperBound;
    }

    /**
     * Return the orienatation of a point with respect to a shape.
     *
     * @param p input point
     *
     * @return the orientation of the point (<0 means inside, ...)
     */
    Orientation orientation( const RealPoint & p) const
    {
      if (( myShapeA.orientation( p ) == ON && myShapeB.orientation( p ) == INSIDE )
          || ( myShapeA.orientation( p ) == INSIDE && myShapeB.orientation( p ) == ON ))
      {
        return INSIDE;
      }
      else if ( myShapeA.orientation( p ) == ON && myShapeB.orientation( p ) == ON ) //discutable
      {
        return INSIDE;
      }
      else if ( myShapeA.isInside( p ))
      {
        return myShapeA.orientation( p );
      }
      else if ( myShapeB.isInside( p ))
      {
        return myShapeB.orientation( p );
      }
      else
      {
        return OUTSIDE;
      }
    }

    /**
     * Destructor.
     */
    ~EuclideanShapesUnion(){}

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
    EuclideanShapesUnion();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    EuclideanShapesUnion ( const EuclideanShapesUnion & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    EuclideanShapesUnion & operator= ( const EuclideanShapesUnion & other );

    // ------------------------- Internals ------------------------------------
  private:
    const ShapeA & myShapeA;
    const ShapeB & myShapeB;

    RealPoint myLowerBound;
    RealPoint myUpperBound;
    //RealPoint myCenter;

  }; // end of class EuclideanShapesUnion

  /////////////////////////////////////////////////////////////////////////////
  // template class EuclideanShapesIntersection
  /**
   * Description of template class 'EuclideanShapesIntersection' <p>
   * \brief Aim:
   */
  template <typename ShapeA, typename ShapeB>
  class EuclideanShapesIntersection
  {
    // ----------------------- Standard services ------------------------------
  public:
    ///@todo BOOST_CONCEPT_ASSERT (( CEuclideanBoundedShape<ShapeA> ));
    ///@todo BOOST_CONCEPT_ASSERT (( CEuclideanOrientedShape<ShapeA> ));
    typedef typename ShapeA::Space Space;
    typedef typename Space::Point Point;
    typedef typename Space::RealPoint RealPoint;

    EuclideanShapesIntersection( const ShapeA & a, const ShapeB & b )
      : myShapeA(a),
        myShapeB(b)
    {
      RealPoint shapeALowerBoundary = myShapeA.getLowerBound();
      RealPoint shapeBLowerBoundary = myShapeB.getLowerBound();
      RealPoint shapeAUpperBoundary = myShapeA.getUpperBound();
      RealPoint shapeBUpperBoundary = myShapeB.getUpperBound();
      for ( unsigned int i = 0; i < myLowerBound.size(); ++i )
      {
        myLowerBound[i] = std::min( shapeALowerBoundary[i], shapeBLowerBoundary[i] );
        myUpperBound[i] = std::max( shapeAUpperBoundary[i], shapeBUpperBoundary[i] );
      }
    }

    /**
     * @param p any point in the plane.
     *
     * @return 'true' if the point is inside the shape, 'false' if it
     * is strictly outside.
     */
    bool isInside( const RealPoint & p ) const
    {
      return myShapeA.isInside( p ) && myShapeB.isInside( p );
    }

    /**
     * @param p any point in the digital plane.
     *
     * @return 'true' if the point is inside the shape, 'false' if it
     * is strictly outside.
     */
    bool isInside( const Point & p ) const
    {
      return myShapeA.isInside( p ) && myShapeB.isInside( p );
    }

    /**
     * @return the lower bound of the shape bounding box.
     *
     */
    RealPoint getLowerBound() const
    {
      return myLowerBound;
    }

    /**
     * @return the upper bound of the shape bounding box.
     *
     */
    RealPoint getUpperBound() const
    {
      return myUpperBound;
    }

    /**
     * Return the orienatation of a point with respect to a shape.
     *
     * @param p input point
     *
     * @return the orientation of the point (<0 means inside, ...)
     */
    Orientation orientation( const RealPoint & p) const
    {
      if (( myShapeA.orientation( p ) == ON && myShapeB.orientation( p ) == INSIDE )
          || ( myShapeA.orientation( p ) == INSIDE && myShapeB.orientation( p ) == ON ))
      {
        return ON;
      }
      else if ( myShapeA.orientation( p ) == ON && myShapeB.orientation( p ) == ON ) //discutable
      {
        return ON;
      }
      else if ( myShapeA.orientation( p ) == INSIDE && myShapeB.orientation( p ) == INSIDE )
      {
        return INSIDE;
      }
      else
      {
        return OUTSIDE;
      }
    }


    /**
     * Destructor.
     */
    ~EuclideanShapesIntersection(){}

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
    EuclideanShapesIntersection();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    EuclideanShapesIntersection ( const EuclideanShapesIntersection & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    EuclideanShapesIntersection & operator= ( const EuclideanShapesIntersection & other );

    // ------------------------- Internals ------------------------------------
  private:
    const ShapeA & myShapeA;
    const ShapeB & myShapeB;

    RealPoint myLowerBound;
    RealPoint myUpperBound;
    RealPoint myCenter;

  }; // end of class EuclideanShapesIntersection

  /////////////////////////////////////////////////////////////////////////////
  // template class EuclideanShapesMinus
  /**
   * Description of template class 'EuclideanShapesMinus' <p>
   * \brief Aim:
   */
  template <typename ShapeA, typename ShapeB>
  class EuclideanShapesMinus
  {
    // ----------------------- Standard services ------------------------------
  public:
    ///@todo BOOST_CONCEPT_ASSERT (( CEuclideanBoundedShape<ShapeA> ));
    ///@todo BOOST_CONCEPT_ASSERT (( CEuclideanOrientedShape<ShapeA> ));
    typedef typename ShapeA::Space Space;
    typedef typename Space::Point Point;
    typedef typename Space::RealPoint RealPoint;

    EuclideanShapesMinus( const ShapeA & a, const ShapeB & b )
      : myShapeA(a),
        myShapeB(b)
    {
      RealPoint shapeALowerBoundary = myShapeA.getLowerBound();
      RealPoint shapeBLowerBoundary = myShapeB.getLowerBound();
      RealPoint shapeAUpperBoundary = myShapeA.getUpperBound();
      RealPoint shapeBUpperBoundary = myShapeB.getUpperBound();
      for ( unsigned int i = 0; i < myLowerBound.size(); ++i )
      {
        myLowerBound[i] = std::min( shapeALowerBoundary[i], shapeBLowerBoundary[i] );
        myUpperBound[i] = std::max( shapeAUpperBoundary[i], shapeBUpperBoundary[i] );
      }
    }

    /**
     * @param p any point in the plane.
     *
     * @return 'true' if the point is inside the shape, 'false' if it
     * is strictly outside.
     */
    bool isInside( const RealPoint & p ) const
    {
      return myShapeA.isInside( p ) && !myShapeB.isInside( p );
    }

    /**
     * @param p any point in the digital plane.
     *
     * @return 'true' if the point is inside the shape, 'false' if it
     * is strictly outside.
     */
    bool isInside( const Point & p ) const
    {
      return myShapeA.isInside( p ) && !myShapeB.isInside( p );
    }

    /**
     * @return the lower bound of the shape bounding box.
     *
     */
    RealPoint getLowerBound() const
    {
      return myLowerBound;
    }

    /**
     * @return the upper bound of the shape bounding box.
     *
     */
    RealPoint getUpperBound() const
    {
      return myUpperBound;
    }

    /**
     * Return the orienatation of a point with respect to a shape.
     *
     * @param p input point
     *
     * @return the orientation of the point (<0 means inside, ...)
     */
    Orientation orientation( const RealPoint & p) const
    {
      if ( myShapeA.orientation( p ) == INSIDE && myShapeB.orientation( p ) == ON )
      {
        return ON;
      }
      else if ( myShapeA.orientation( p ) == INSIDE && myShapeB.orientation( p ) == INSIDE )
      {
        return OUTSIDE;
      }
      else if ( myShapeA.orientation( p ) == INSIDE )
      {
        return INSIDE;
      }
      else
      {
        return OUTSIDE;
      }
    }


    /**
     * Destructor.
     */
    ~EuclideanShapesMinus(){}

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
    EuclideanShapesMinus();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    EuclideanShapesMinus ( const EuclideanShapesMinus & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    EuclideanShapesMinus & operator= ( const EuclideanShapesMinus & other );

    // ------------------------- Internals ------------------------------------
  private:
    const ShapeA & myShapeA;
    const ShapeB & myShapeB;

    RealPoint myLowerBound;
    RealPoint myUpperBound;
    //RealPoint myCenter;

  }; // end of class EuclideanShapesMinus





  /**
   * Overloads 'operator<<' for displaying objects of class 'EuclideanShapesAdapter'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'EuclideanShapesAdapter' to write.
   * @return the output stream after the writing.
   */
  template <typename ShapeA, typename ShapeB>
  std::ostream&
  operator<< ( std::ostream & out, const EuclideanShapesUnion<ShapeA, ShapeB> & object );

  template <typename ShapeA, typename ShapeB>
  std::ostream&
  operator<< ( std::ostream & out, const EuclideanShapesIntersection<ShapeA, ShapeB> & object );

  template <typename ShapeA, typename ShapeB>
  std::ostream&
  operator<< ( std::ostream & out, const EuclideanShapesMinus<ShapeA, ShapeB> & object );

} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined EuclideanShapesAdapter_h

#undef EuclideanShapesAdapter_RECURSES
#endif // else defined(EuclideanShapesAdapter_RECURSES)
