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
 * @file ParametricShapeArcLengthFunctor.h
 * @brief Estimates the arc length of a paramtric curve.
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/06/30
 *
 * Header file for module ParametricShapeArcLengthFunctor.cpp
 *
 * This file is part of the DGtal library.
 *
 * @see testLengthEstimators.cpp, testTrueLocalEstimator.cpp
 */

#if defined(ParametricShapeArcLengthFunctor_RECURSES)
#error Recursive header files inclusion detected in ParametricShapeArcLengthFunctor.h
#else // defined(ParametricShapeArcLengthFunctor_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ParametricShapeArcLengthFunctor_RECURSES

#if !defined ParametricShapeArcLengthFunctor_h
/** Prevents repeated inclusion of headers. */
#define ParametricShapeArcLengthFunctor_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ParametricShapeArcLengthFunctor
  /**
   * Description of template class 'ParametricShapeArcLengthFunctor' <p>
   * \brief Aim: implements a functor that estimates the arc length of a
   * paramtric curve.
   *
   *
   * @tparam TParametricShape a model of parametric shape.
   */
  template <typename TParametricShape>
  class ParametricShapeArcLengthFunctor
  {

    // ----------------------- Standard services ------------------------------
  public:

    ///Type of parametric shape.
    typedef TParametricShape ParametricShape;

    ///Type of const iterator on points.
    typedef typename TParametricShape::RealPoint RealPoint;

    ///Type of the functor output.
    typedef double Quantity;

    /**
     * Constructor.
     */
    ParametricShapeArcLengthFunctor() = delete;


    /**
     * Constructor.
     * @param aShape the input shape.
     */
    ParametricShapeArcLengthFunctor(const ParametricShape &aShape): myShape(aShape) {};


    /**
     * Destructor.
     */
    ~ParametricShapeArcLengthFunctor() = default;


    // ----------------------- Interface --------------------------------------
  public:
    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    ParametricShapeArcLengthFunctor & operator= ( const ParametricShapeArcLengthFunctor & other ) = delete;

    /**
     * Compute the arc length between two points.
     *
     * @param aFirstPoint the first point
     * @param aSecondPoint the second point
     * @return the estimated arc length
     */
    Quantity operator()(const RealPoint &aFirstPoint,const RealPoint &aSecondPoint) const
    {
      //determining nbSamples from the bounding box size of the shape
      RealPoint v = myShape.getUpperBound() - myShape.getLowerBound();
      double n = v.norm(RealPoint::L_infty);
      unsigned int nbSamples = (unsigned int) ceil( n*100 );

      //computes the angles
      double t = myShape.parameter( aFirstPoint );
      double t2 = myShape.parameter( aSecondPoint );
      return myShape.arclength( t, t2, nbSamples );

    }

    /**
     * Compute the total length
     *
     * @return the estimated length
     */
    Quantity operator()() const
    {
      //determining nbSamples from the bounding box size of the shape
      RealPoint v = myShape.getUpperBound() - myShape.getLowerBound();
      double n = v.norm(RealPoint::L_infty);
      unsigned int nbSamples = (unsigned int) ceil( n*100 );

      return myShape.arclength( 0,2*M_PI, nbSamples );
    }

    // ------------------------- Private Data --------------------------------
  private:

    ///Reference of the implicit shape.
    const ParametricShape& myShape;

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class ParametricShapeArcLengthFunctor

} // namespace DGtal

                                                                        //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ParametricShapeArcLengthFunctor_h

#undef ParametricShapeArcLengthFunctor_RECURSES
#endif // else defined(ParametricShapeArcLengthFunctor_RECURSES)
