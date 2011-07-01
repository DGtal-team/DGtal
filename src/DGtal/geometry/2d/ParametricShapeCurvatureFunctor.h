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
 * @file ParametricShapeCurvatureFunctor.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/06/30
 *
 * Header file for module ParametricShapeCurvatureFunctor.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ParametricShapeCurvatureFunctor_RECURSES)
#error Recursive header files inclusion detected in ParametricShapeCurvatureFunctor.h
#else // defined(ParametricShapeCurvatureFunctor_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ParametricShapeCurvatureFunctor_RECURSES

#if !defined ParametricShapeCurvatureFunctor_h
/** Prevents repeated inclusion of headers. */
#define ParametricShapeCurvatureFunctor_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ParametricShapeCurvatureFunctor
  /**
   * Description of template class 'ParametricShapeCurvatureFunctor' <p>
   * \brief Aim: implements a functor that computes the curvature
   * at a given point of a parametric shape.
   *
   * @note @f$\kappa(t) = \frac{x'y''-y'x''}{(x'^2+y'^2)^{3/2}}@f$
   *
   *
   * @tparam TParametricShape a model of parametric shape.
   * @tparam TConstIteratorOnPoints a model of CConstIteratorOnPoints.
   */
  template <typename TParametricShape, typename TConstIteratorOnPoints>
  class ParametricShapeCurvatureFunctor
  {
    
    // ----------------------- Standard services ------------------------------
  public:

    ///Type of parametric shape.
    typedef TParametricShape ParametricShape;

    ///Type of const iterator on points.
    typedef TConstIteratorOnPoints ConstIteratorOnPoints;

    ///Type of the functor output.
    typedef double Quantity;

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    ParametricShapeCurvatureFunctor(){}


    /**
     * Constructor.
     */
    ParametricShapeCurvatureFunctor(ParametricShape *aShape): myShape(aShape) {};
    
    
    /**
     * Destructor.
     */
    ~ParametricShapeCurvatureFunctor(){}
    

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    ParametricShapeCurvatureFunctor & operator= ( const ParametricShapeCurvatureFunctor & other )
    {
      myShape = other.myShape;
      return *this;
    }

   
    /** 
     * Compute the curvature at a point specified by the
     * ConstIteratorOnPoints.
     *
     * 
     * @param it the point at which the curvature is computed. 
     * @return the tangent at (*it).
     */
    Quantity operator()(const ConstIteratorOnPoints &it)
    {
      ASSERT(myShape);
      typename ParametricShape::RealPoint2D p = *it,  tangent, second;
      double t = myShape->parameter( p );
     
      return myShape->curvature( t );
    }
    

    // ------------------------- Private Datas --------------------------------
  private:
    
    ///Copy of the implicit shape.
    ParametricShape *myShape;
    
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of class ParametricShapeCurvatureFunctor
  
} // namespace DGtal

                                                                        //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ParametricShapeCurvatureFunctor_h

#undef ParametricShapeCurvatureFunctor_RECURSES
#endif // else defined(ParametricShapeCurvatureFunctor_RECURSES)
