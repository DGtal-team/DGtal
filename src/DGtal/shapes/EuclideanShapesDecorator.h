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
 * @file EuclideanShapesDecorator.h
 * @author Jeremy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2012/08/28
 *
 * This file is part of the DGtal library.
 */

#if defined(EuclideanShapesDecorator_RECURSES)
#error Recursive header files inclusion detected in EuclideanShapesDecorator.h
#else // defined(EuclideanShapesDecorator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define EuclideanShapesDecorator_RECURSES

#if !defined EuclideanShapesDecorator_h
/** Prevents repeated inclusion of headers. */
#define EuclideanShapesDecorator_h

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
// template class EuclideanShapesDecorator
/**
 * Description of template class 'EuclideanShapesDecorator' <p>
 * \brief Aim: Union between two models of CEuclideanBoundedShape and CEuclideanOrientedShape
 *
 * @tparam ShapeA type of the first shape. Must be a model of CEuclideanBoundedShape and CEuclideanOrientedShape
 * @tparam ShapeB type of the second shape. Must be a model of CEuclideanBoundedShape and CEuclideanOrientedShape
 */

  template <typename ShapeA, typename ShapeB>
  class EuclideanShapesUnion
  {
    // ----------------------- Standard services ------------------------------
  public:
    BOOST_CONCEPT_ASSERT (( CEuclideanBoundedShape< ShapeA > ));
    BOOST_CONCEPT_ASSERT (( CEuclideanOrientedShape< ShapeA > ));
    BOOST_CONCEPT_ASSERT (( CEuclideanBoundedShape< ShapeB > ));
    BOOST_CONCEPT_ASSERT (( CEuclideanOrientedShape< ShapeB > ));

    typedef typename ShapeA::Space Space;
    typedef typename ShapeA::RealPoint RealPoint;

    /**
      * Constructor.
      *
      * @param a a model of CEuclideanBoundedShape and CEuclideanOrientedShape
      * @param b a model of CEuclideanBoundedShape and CEuclideanOrientedShape
      */
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
     * Return the orientation of a point with respect to a shape.
     *
     * @param p input point
     *
     * @return the orientation of the point (<0 means inside, ...)
     */
    Orientation orientation( const RealPoint & p) const
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

  }; // end of class EuclideanShapesUnion

  /////////////////////////////////////////////////////////////////////////////
  // template class EuclideanShapesIntersection
  /**
   * Description of template class 'EuclideanShapesIntersection' <p>
   * \brief Aim: Intersection between two models of CEuclideanBoundedShape and CEuclideanOrientedShape
   *
   * @tparam ShapeA type of the first shape. Must be a model of CEuclideanBoundedShape and CEuclideanOrientedShape
   * @tparam ShapeB type of the second shape. Must be a model of CEuclideanBoundedShape and CEuclideanOrientedShape
   */
  template <typename ShapeA, typename ShapeB>
  class EuclideanShapesIntersection
  {
    // ----------------------- Standard services ------------------------------
  public:
    BOOST_CONCEPT_ASSERT (( CEuclideanBoundedShape< ShapeA > ));
    BOOST_CONCEPT_ASSERT (( CEuclideanOrientedShape< ShapeA > ));
    BOOST_CONCEPT_ASSERT (( CEuclideanBoundedShape< ShapeB > ));
    BOOST_CONCEPT_ASSERT (( CEuclideanOrientedShape< ShapeB > ));

    typedef typename ShapeA::Space Space;
    typedef typename ShapeA::RealPoint RealPoint;

    /**
      * Constructor.
      *
      * @param a a model of CEuclideanBoundedShape and CEuclideanOrientedShape
      * @param b a model of CEuclideanBoundedShape and CEuclideanOrientedShape
      */
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
     * Return the orientation of a point with respect to a shape.
     *
     * @param p input point
     *
     * @return the orientation of the point (<0 means inside, ...)
     */
    Orientation orientation( const RealPoint & p) const
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

  }; // end of class EuclideanShapesIntersection

  /////////////////////////////////////////////////////////////////////////////
  // template class EuclideanShapesMinus
  /**
   * Description of template class 'EuclideanShapesMinus' <p>
   * \brief Aim: Minus between two models of CEuclideanBoundedShape and CEuclideanOrientedShape
   *
   * @tparam ShapeA type of the first shape. Must be a model of CEuclideanBoundedShape and CEuclideanOrientedShape
   * @tparam ShapeB type of the second shape. Must be a model of CEuclideanBoundedShape and CEuclideanOrientedShape
   */
  template <typename ShapeA, typename ShapeB>
  class EuclideanShapesMinus
  {
    // ----------------------- Standard services ------------------------------
  public:
    BOOST_CONCEPT_ASSERT (( CEuclideanBoundedShape< ShapeA > ));
    BOOST_CONCEPT_ASSERT (( CEuclideanOrientedShape< ShapeA > ));
    BOOST_CONCEPT_ASSERT (( CEuclideanBoundedShape< ShapeB > ));
    BOOST_CONCEPT_ASSERT (( CEuclideanOrientedShape< ShapeB > ));

    typedef typename ShapeA::Space Space;
    typedef typename ShapeA::RealPoint RealPoint;

    /**
      * Constructor.
      *
      * @param a a model of CEuclideanBoundedShape and CEuclideanOrientedShape
      * @param b a model of CEuclideanBoundedShape and CEuclideanOrientedShape
      */
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
      return myShapeA.isInside( p ) && !myShapeB.orientation(p) == INSIDE;
    }

    /**
     * @return the lower bound of the shape bounding box.
     *
     */
    RealPoint getLowerBound() const
    {
      return myLowerBound + RealPoint(-1,-1);
    }

    /**
     * @return the upper bound of the shape bounding box.
     *
     */
    RealPoint getUpperBound() const
    {
      return myUpperBound + RealPoint(1,1);
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

  }; // end of class EuclideanShapesMinus





  /**
   * Overloads 'operator<<' for displaying objects of class 'EuclideanShapesDecorator'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'EuclideanShapesDecorator' to write.
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

#endif // !defined EuclideanShapesDecorator_h

#undef EuclideanShapesDecorator_RECURSES
#endif // else defined(EuclideanShapesDecorator_RECURSES)
