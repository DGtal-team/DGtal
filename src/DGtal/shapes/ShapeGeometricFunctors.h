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
 * @file ShapeGeometricFunctors.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2014/02/14
 *
 * Header file for module ShapeGeometricFunctors.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ShapeGeometricFunctors_RECURSES)
#error Recursive header files inclusion detected in ShapeGeometricFunctors.h
#else // defined(ShapeGeometricFunctors_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ShapeGeometricFunctors_RECURSES

#if !defined ShapeGeometricFunctors_h
/** Prevents repeated inclusion of headers. */
#define ShapeGeometricFunctors_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  namespace functors
  {

  namespace ShapeGeometricFunctors
  {

    /**
     * Description of template class 'ShapePositionFunctor' <p> \brief
     * Aim: A functor RealPoint -> Quantity that returns the position of
     * the point itself.
     *
     * @tparam TShape the type of the shape where geometric estimation
     * are made.
     */
    template <typename TShape>
    struct ShapePositionFunctor {
      typedef TShape Shape;
      typedef typename Shape::RealPoint RealPoint;
      typedef RealPoint Argument;
      typedef RealPoint Quantity;
      typedef Quantity Value;

      /**
       * Constructor. A shape may also be attached at construction.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      ShapePositionFunctor( ConstAlias<Shape> aShape = 0 ) : myShape( aShape ) {}

      /**
       * Attach a shape.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      void attach( ConstAlias<Shape> aShape )
      {
        myShape = aShape;
      }

      /**
         Map operator RealPoint -> RealPoint giving the position.
         @param p any point on the shape.
         @return the point p itself
      */
      Quantity operator()( const RealPoint & p ) const
      {
        return p;
      }

    private:
      /// The shape of interest.
      CountedConstPtrOrConstPtr<Shape> myShape;
    };

    /**
     * Description of template class 'ShapeNormalVectorFunctor' <p> \brief
     * Aim: A functor RealPoint -> Quantity that returns the normal
     * vector at given point.
     *
     * @tparam TShape the type of the shape where geometric estimation
     * are made. It must have methods \a gradient.
     */
    template <typename TShape>
    struct ShapeNormalVectorFunctor {
      typedef TShape Shape;
      typedef typename Shape::RealPoint RealPoint;
      typedef typename Shape::RealVector RealVector;
      typedef RealPoint Argument;
      typedef typename RealVector::Component Scalar;
      typedef RealVector Quantity;
      typedef Quantity Value;

      /**
       * Constructor. A shape may also be attached at construction.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      ShapeNormalVectorFunctor( ConstAlias<Shape> aShape = 0 ) : myShape( aShape ) {}

      /**
       * Attach a shape.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      void attach( ConstAlias<Shape> aShape )
      {
        myShape = aShape;
      }

      /**
         Map operator RealPoint -> RealVector giving the normal vector.
         @param p any point on the shape.
         @return the normal at point p (as the normalized gradient).
      */
      Quantity operator()( const RealPoint & p ) const
      {
        RealVector v = myShape->gradient( p );
        Scalar norm = v.norm();
        return ( norm != 0 ) ? v / norm : v;
      }

    private:
      /// The shape of interest.
      CountedConstPtrOrConstPtr<Shape> myShape;
    };


    /**
     * Description of template class 'ShapeMeanCurvatureFunctor' <p>
     * \brief Aim: A functor RealPoint -> Quantity that returns the
     * mean curvature at given point.
     *
     * @tparam TShape the type of the shape where geometric estimation
     * are made. It must have method \a meanCurvature.
     */
    template <typename TShape>
    struct ShapeMeanCurvatureFunctor {
      typedef TShape Shape;
      typedef typename Shape::RealPoint RealPoint;
      typedef typename Shape::RealVector RealVector;
      typedef typename RealVector::Component Scalar;
      typedef RealPoint Argument;
      typedef Scalar Quantity;
      typedef Quantity Value;

      /**
       * Constructor. A shape may also be attached at construction.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      ShapeMeanCurvatureFunctor( ConstAlias<Shape> aShape = 0 ) : myShape( aShape ) {}

      /**
       * Attach a shape.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      void attach( ConstAlias<Shape> aShape )
      {
        myShape = aShape;
      }

      /**
         Map operator RealPoint -> Quantity giving the mean curvature.
         @param p any point on the shape.
         @return the mean curvature at point p.
      */
      Quantity operator()( const RealPoint & p ) const
      {
        return myShape->meanCurvature( p );
      }

    private:
      /// The shape of interest.
      CountedConstPtrOrConstPtr<Shape> myShape;
    };

    /**
     * Description of template class 'ShapeGaussianCurvatureFunctor' <p>
     * \brief Aim: A functor RealPoint -> Quantity that returns the
     * gaussian curvature at given point.
     *
     * @tparam TShape the type of the shape where geometric estimation
     * are made. It must have method \a gaussianCurvature.
     */
    template <typename TShape>
    struct ShapeGaussianCurvatureFunctor {
      typedef TShape Shape;
      typedef typename Shape::RealPoint RealPoint;
      typedef typename Shape::RealVector RealVector;
      typedef typename RealVector::Component Scalar;
      typedef RealPoint Argument;
      typedef Scalar Quantity;
      typedef Quantity Value;

      /**
       * Constructor. A shape may also be attached at construction.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      ShapeGaussianCurvatureFunctor( ConstAlias<Shape> aShape = 0 ) : myShape( aShape ) {}

      /**
       * Attach a shape.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      void attach( ConstAlias<Shape> aShape )
      {
        myShape = aShape;
      }

      /**
         Map operator RealPoint -> Quantity giving the Gaussian curvature.
         @param p any point on the shape.
         @return the Gaussian curvature at point p.
      */
      Quantity operator()( const RealPoint & p ) const
      {
        return myShape->gaussianCurvature( p );
      }

    private:
      /// The shape of interest.
      CountedConstPtrOrConstPtr<Shape> myShape;
    };

    /**
     * Description of template class 'ShapeFirstPrincipalCurvatureFunctor' <p>
     * \brief Aim: A functor RealPoint -> Quantity that returns the
     * first principal curvature at given point (i.e. smallest principal curvature).
     *
     * @tparam TShape the type of the shape where geometric estimation
     * are made. It must have method \a principalCurvatures.
     */
    template <typename TShape>
    struct ShapeFirstPrincipalCurvatureFunctor {
      typedef TShape Shape;
      typedef typename Shape::RealPoint RealPoint;
      typedef typename Shape::RealVector RealVector;
      typedef typename RealVector::Component Scalar;
      typedef RealPoint Argument;
      typedef Scalar Quantity;
      typedef Quantity Value;

      /**
       * Constructor. A shape may also be attached at construction.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      ShapeFirstPrincipalCurvatureFunctor( ConstAlias<Shape> aShape = 0 )
	: myShape( aShape ) {}

      /**
       * Attach a shape.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      void attach( ConstAlias<Shape> aShape )
      {
        myShape = aShape;
      }

      /**
         Map operator RealPoint -> Quantity giving the first principal curvature.
         @param p any point on the shape.
         @return the first principal curvature at point p (smallest curvature).
      */
      Quantity operator()( const RealPoint & p ) const
      {
	Quantity k1, k2;
	myShape->principalCurvatures( p, k1, k2 );
        return k1;
      }

    private:
      /// The shape of interest.
      CountedConstPtrOrConstPtr<Shape> myShape;
    }; // struct ShapeFirstPrincipalCurvatureFunctor

    /**
     * Description of template class 'ShapeSecondPrincipalCurvatureFunctor' <p>
     * \brief Aim: A functor RealPoint -> Quantity that returns the
     * second principal curvature at given point (i.e. greatest principal curvature).
     *
     * @tparam TShape the type of the shape where geometric estimation
     * are made. It must have method \a principalCurvatures.
     */
    template <typename TShape>
    struct ShapeSecondPrincipalCurvatureFunctor {
      typedef TShape Shape;
      typedef typename Shape::RealPoint RealPoint;
      typedef typename Shape::RealVector RealVector;
      typedef typename RealVector::Component Scalar;
      typedef RealPoint Argument;
      typedef Scalar Quantity;
      typedef Quantity Value;

      /**
       * Constructor. A shape may also be attached at construction.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      ShapeSecondPrincipalCurvatureFunctor( ConstAlias<Shape> aShape = 0 )
	: myShape( aShape ) {}

      /**
       * Attach a shape.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      void attach( ConstAlias<Shape> aShape )
      {
        myShape = aShape;
      }

      /**
         Map operator RealPoint -> Scalar giving the second principal curvature.
         @param p any point on the shape.
         @return the second principal curvature at point p (greatest curvature).
      */
      Quantity operator()( const RealPoint & p ) const
      {
	Quantity k1, k2;
	myShape->principalCurvatures( p, k1, k2 );
        return k2;
      }

    private:
      /// The shape of interest.
      CountedConstPtrOrConstPtr<Shape> myShape;
    }; // struct ShapeSecondPrincipalCurvatureFunctor

    /**
     * Description of template class
     * 'ShapeFirstPrincipalDirectionFunctor' <p> \brief Aim: A functor
     * RealPoint -> RealVector that returns the first principal
     * direction at given point (i.e. direction of smallest principal
     * curvature).
     *
     * @tparam TShape the type of the shape where geometric estimation
     * are made. It must have method \a principalDirections.
     */
    template <typename TShape>
    struct ShapeFirstPrincipalDirectionFunctor {
      typedef TShape Shape;
      typedef typename Shape::RealPoint RealPoint;
      typedef typename Shape::RealVector RealVector;
      typedef typename RealVector::Component Scalar;
      typedef RealPoint Argument;
      typedef RealVector Quantity;
      typedef Quantity Value;

      /**
       * Constructor. A shape may also be attached at construction.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      ShapeFirstPrincipalDirectionFunctor( ConstAlias<Shape> aShape = 0 )
	: myShape( aShape ) {}

      /**
       * Attach a shape.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      void attach( ConstAlias<Shape> aShape )
      {
        myShape = aShape;
      }

      /**
         Map operator RealPoint -> Scalar giving the first principal direction (i.e. direction of first/smallest principal curvature).
         @param p any point on the shape.
         @return the first principal direction at point p.
      */
      Quantity operator()( const RealPoint & p ) const
      {
	Quantity d1, d2;
	myShape->principalDirections( p, d1, d2 );
        return d1;
      }

    private:
      /// The shape of interest.
      CountedConstPtrOrConstPtr<Shape> myShape;
    }; // struct ShapeFirstPrincipalDirectionFunctor

    /**
     * Description of template class
     * 'ShapeSecondPrincipalDirectionFunctor' <p> \brief Aim: A functor
     * RealPoint -> RealVector that returns the second principal
     * direction at given point (i.e. direction of second/greatest principal
     * curvature).
     *
     * @tparam TShape the type of the shape where geometric estimation
     * are made. It must have method \a principalDirections.
     */
    template <typename TShape>
    struct ShapeSecondPrincipalDirectionFunctor {
      typedef TShape Shape;
      typedef typename Shape::RealPoint RealPoint;
      typedef typename Shape::RealVector RealVector;
      typedef typename RealVector::Component Scalar;
      typedef RealPoint Argument;
      typedef RealVector Quantity;
      typedef Quantity Value;

      /**
       * Constructor. A shape may also be attached at construction.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      ShapeSecondPrincipalDirectionFunctor( ConstAlias<Shape> aShape = 0 )
	: myShape( aShape ) {}

      /**
       * Attach a shape.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      void attach( ConstAlias<Shape> aShape )
      {
        myShape = aShape;
      }

      /**
         Map operator RealPoint -> Scalar giving the second principal direction (i.e. direction of second/greatest principal curvature).
         @param p any point on the shape.
         @return the second principal direction at point p.
      */
      Quantity operator()( const RealPoint & p ) const
      {
	Quantity d1, d2;
	myShape->principalDirections( p, d1, d2 );
        return d2;
      }

    private:
      /// The shape of interest.
      CountedConstPtrOrConstPtr<Shape> myShape;
    }; // struct ShapeSecondPrincipalDirectionFunctor

    /**
     * Description of template class
     * 'ShapePrincipalCurvaturesAndDirectionsFunctor' <p> \brief Aim:
     * A functor RealPoint -> (Scalar,Scalar,RealVector,RealVector
     * that returns the principal curvatures and the principal
     * directions as a tuple at given point (k1,k2,d1,d2).
     *
     * @tparam TShape the type of the shape where geometric estimation
     * are made. It must have method \a principalDirections.
     */
    template <typename TShape>
    struct ShapePrincipalCurvaturesAndDirectionsFunctor {
      typedef TShape Shape;
      typedef typename Shape::RealPoint RealPoint;
      typedef typename Shape::RealVector RealVector;
      typedef typename RealVector::Component Scalar;
      typedef RealPoint Argument;
      typedef std::tuple<Scalar,Scalar,RealVector,RealVector> Quantity;
      typedef Quantity Value;

      /**
       * Constructor. A shape may also be attached at construction.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      ShapePrincipalCurvaturesAndDirectionsFunctor( ConstAlias<Shape> aShape = 0 )
	: myShape( aShape ) {}

      /**
       * Attach a shape.
       *
       * @param aShape the shape of interest. The alias can be secured
       * if a some counted pointer is handed.
       */
      void attach( ConstAlias<Shape> aShape )
      {
        myShape = aShape;
      }

      /**
         Map operator RealPoint ->
         (Scalar,Scalar,RealVector,RealVector) giving the principal
         curvatures and directions (k1,k2,d1,d2).
         @param p any point on the shape.
         @return the principal curvatures and directions (k1,k2,d1,d2)
         at point p.
      */
      Quantity operator()( const RealPoint & p ) const
      {
	Scalar k1, k2;
	RealVector d1, d2;
	myShape->principalCurvatures( p, k1, k2 );
	myShape->principalDirections( p, d1, d2 );
        return std::make_tuple( k1, k2, d1, d2 );
      }

    private:
      /// The shape of interest.
      CountedConstPtrOrConstPtr<Shape> myShape;
    }; // struct ShapePrincipalCurvaturesAndDirectionsFunctor


  } // namespace ShapeGeometricFunctors
  } // namespace functors
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/shapes/ShapeGeometricFunctors.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ShapeGeometricFunctors_h

#undef ShapeGeometricFunctors_RECURSES
#endif // else defined(ShapeGeometricFunctors_RECURSES)
