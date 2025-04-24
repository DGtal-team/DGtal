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
#include "DGtal/base/ConstAlias.h"

#include "DGtal/shapes/CEuclideanBoundedShape.h"
#include "DGtal/shapes/CEuclideanOrientedShape.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class EuclideanShapesCSG
  /**
   * Description of template class 'EuclideanShapesCSG' <p>
   * \brief Aim: Constructive Solid Geometry (CSG) between models of CEuclideanBoundedShape and CEuclideanOrientedShape
   * Use CSG operation (union, intersection, minus) from a shape of Type ShapeA with one (or more) shapes of Type ShapeB.
   * Can combine differents operations.
   * Limitations: Since we don't have a class derived by all shapes, operations can be done by only one type of shapes.
   * Use CSG of CSG to go beyond this limitation.
   *
   * @tparam ShapeA type of a first shape. Must be a model of CEuclideanBoundedShape and CEuclideanOrientedShape
   * @tparam ShapeB type of a second shape. Must be a model of CEuclideanBoundedShape and CEuclideanOrientedShape
   */
  template <typename ShapeA, typename ShapeB>
  class EuclideanShapesCSG
  {
  protected:
    enum e_operator
    {
      e_plus,
      e_intersection,
      e_minus
    };

  public:
    BOOST_CONCEPT_ASSERT (( concepts::CEuclideanBoundedShape< ShapeA > ));
    BOOST_CONCEPT_ASSERT (( concepts::CEuclideanOrientedShape< ShapeA > ));

    typedef typename ShapeA::Space Space;
    typedef typename ShapeA::RealPoint RealPoint;

    /**
      * Default constructor. EuclideanShapesCSG will be not valid without setParams(ShapeA).
      *
      */
    EuclideanShapesCSG( )
      : bIsValid(false)
    {}

    /**
      * Copy constructor.
      *
      * @param[in] other a EuclideanShapesCSG to copy
      */
    EuclideanShapesCSG ( const EuclideanShapesCSG & other )
      : myShapeA(other.myShapeA), v_shapes(other.v_shapes),
        myLowerBound(other.myLowerBound), myUpperBound(other.myUpperBound),
        bIsValid(other.bIsValid)
    {}

    /**
      * Constructor. EuclideanShapesCSG will be valid.
      *
      * @param[in] a a model of CEuclideanBoundedShape and CEuclideanOrientedShape
      */
    EuclideanShapesCSG( ConstAlias<ShapeA> a )
      : myShapeA( a )
    {
      myLowerBound = myShapeA->getLowerBound();
      myUpperBound = myShapeA->getUpperBound();

      bIsValid = true;
    }

    /**
      * Copy operator.
      *
      * @param[in] other a EuclideanShapesCSG to copy
      *
      * @return this
      */
    EuclideanShapesCSG & operator= ( const EuclideanShapesCSG & other )
    {
      myShapeA = other.myShapeA;
      v_shapes = other.v_shapes;

      myLowerBound = other.myLowerBound;
      myUpperBound = other.myUpperBound;

      bIsValid = other.bIsValid;

      return *this;
    }

    /**
      * Add a (unique) ShapeA for the CSG computation. EuclideanShapesCSG will be valid after. If a ShapeA was already set, the previous one will be override.
      *
      * @param[in] a a ShapeA, model of CEuclideanBoundedShape and CEuclideanOrientedShape
      */
    void setParams( ConstAlias<ShapeA> a )
    {
      myShapeA = a;

      myLowerBound = myShapeA->getLowerBound();
      myUpperBound = myShapeA->getUpperBound();

      bIsValid = true;
    }

    /**
      * Union between a shape (ShapeA, gived at construction) and b (ShapeB). If an operation was already set, the
      * union will be between the CSG shape and b (ShapeB).
      *
      * @param[in] b a ShapeB, model of CEuclideanBoundedShape and CEuclideanOrientedShape
      */
    void plus( ConstAlias<ShapeB> b )
    {
      BOOST_CONCEPT_ASSERT (( concepts::CEuclideanBoundedShape< ShapeB > ));
      BOOST_CONCEPT_ASSERT (( concepts::CEuclideanOrientedShape< ShapeB > ));

      FATAL_ERROR_MSG( isValid(), "Operation invalid. Maybe you don't set a ShapeA object." );

      std::pair<e_operator, CountedConstPtrOrConstPtr< ShapeB > > shape( e_plus, b );

      for(Dimension i =0; i < Space::dimension; ++i)
      {
        myLowerBound[i] = std::min(myLowerBound[i], b->getLowerBound()[i]);
        myUpperBound[i] = std::max(myUpperBound[i], b->getUpperBound()[i]);
      }

      v_shapes.push_back(shape); 
    }

    /**
      * Intersection between a (ShapeA) and b (ShapeB). If an operation was already set, the
      * intersection will be between the CSG shape and b (ShapeB).
      *
      * @param[in] b a ShapeB, model of CEuclideanBoundedShape and CEuclideanOrientedShape
      */
    void intersection( ConstAlias<ShapeB> b )
    {
      BOOST_CONCEPT_ASSERT (( concepts::CEuclideanBoundedShape< ShapeB > ));
      BOOST_CONCEPT_ASSERT (( concepts::CEuclideanOrientedShape< ShapeB > ));

      FATAL_ERROR_MSG( isValid(), "Operation invalid. Maybe you don't set a ShapeA object." );

      std::pair<e_operator, CountedConstPtrOrConstPtr< ShapeB > > shape( e_intersection, b );

      for(Dimension i=0; i < Space::dimension; ++i)
      {
        myLowerBound[i] = std::max(myLowerBound[i], b->getLowerBound()[i]);
        myUpperBound[i] = std::min(myUpperBound[i], b->getUpperBound()[i]);
      }

      v_shapes.push_back(shape); 
    }

    /**
      * Minus between a (ShapeA) and b (ShapeB). If an operation was already set, the
      * minus will be between the CSG shape and b (ShapeB).
      *
      * @param[in] b a ShapeB, model of CEuclideanBoundedShape and CEuclideanOrientedShape
      */
    void minus( ConstAlias<ShapeB> b )
    {
      BOOST_CONCEPT_ASSERT (( concepts::CEuclideanBoundedShape< ShapeB > ));
      BOOST_CONCEPT_ASSERT (( concepts::CEuclideanOrientedShape< ShapeB > ));

      FATAL_ERROR_MSG( isValid(), "Operation invalid. Maybe you don't set a ShapeA object." );

      std::pair<e_operator, CountedConstPtrOrConstPtr< ShapeB > > shape( e_minus, b );

      v_shapes.push_back(shape);
    }

    /**
     * @return the lower bound of the shape bounding box.
     *
     */
    RealPoint getLowerBound() const
    {
      FATAL_ERROR_MSG( isValid(), "Operation invalid. Maybe you don't set a ShapeA object." );

      return myLowerBound;
    }

    /**
     * @return the upper bound of the shape bounding box.
     *
     */
    RealPoint getUpperBound() const
    {
      FATAL_ERROR_MSG( isValid(), "Operation invalid. Maybe you don't set a ShapeA object." );

      return myUpperBound;
    }

    /**
     * Return the orientation of a point with respect to a shape. Resolve all operations done
     * with operators in the order they are set.
     *
     * @param[in] p input point
     *
     * @return the orientation of the point (0 = INSIDE, 1 = ON, 2 = OUTSIDE)
     */
    Orientation orientation( const RealPoint & p ) const
    {
      FATAL_ERROR_MSG( isValid(), "Operation invalid. Maybe you don't set a ShapeA object." );

      Orientation orient = myShapeA->orientation( p );

      for(unsigned int i = 0; i < v_shapes.size(); ++i)
      {
        if( v_shapes[i].first == e_minus )
        {
          if (( v_shapes[i].second->orientation( p ) == INSIDE ) || ( v_shapes[i].second->orientation( p ) == ON ))
          {
            orient = OUTSIDE;
          }
        }
        else if( v_shapes[i].first == e_intersection )
        {
          if (( orient == ON ) && ( v_shapes[i].second->orientation( p ) != OUTSIDE ))
          {
            orient = ON;
          }
          else if (( v_shapes[i].second->orientation( p ) == ON ) && ( orient != OUTSIDE ))
          {
            orient = ON;
          }
          else if (( orient == INSIDE ) && ( v_shapes[i].second->orientation( p ) == INSIDE ))
          {
            orient = INSIDE;
          }
          else
          {
            orient = OUTSIDE;
          }
        }
        else /// e_plus
        {
          if (( orient == INSIDE ) || ( v_shapes[i].second->orientation( p ) == INSIDE ))
          {
            orient = INSIDE;
          }
          else if (( orient == ON ) || ( v_shapes[i].second->orientation( p ) == ON ))
          {
            orient = ON;
          }
          else
          {
            orient = OUTSIDE;
          }
        }
      }

      return orient;
    }

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
    bool isValid() const
    {
      return bIsValid;
    }

    // ------------------------- Internals ------------------------------------
  private:

    /// Base Shape.
    CountedConstPtrOrConstPtr< ShapeA > myShapeA;

    /// Vector of all operations (ordered) of ShapeB.
    std::vector< std::pair<e_operator,  CountedConstPtrOrConstPtr< ShapeB > > > v_shapes;

    /// Domain lower bound.
    RealPoint myLowerBound;

    /// Domain upper bound.
    RealPoint myUpperBound;

    /// if the CSG is valid.
    bool bIsValid;

  };

} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined EuclideanShapesDecorator_h

#undef EuclideanShapesDecorator_RECURSES
#endif // else defined(EuclideanShapesDecorator_RECURSES)
