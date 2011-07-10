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
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/06/30
 *
 * Header file for module ParametricShapeArcLengthFunctor.cpp
 *
 * This file is part of the DGtal library.
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
   * @tparam TConstIteratorOnPoints a model of CConstIteratorOnPoints.
   */
  template <typename TParametricShape, typename TConstIteratorOnPoints>
  class ParametricShapeArcLengthFunctor
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
    ParametricShapeArcLengthFunctor(){}


    /**
     * Constructor.
     */
    ParametricShapeArcLengthFunctor(ParametricShape *aShape): myShape(aShape) {};
    
    
    /**
     * Destructor.
     */
    ~ParametricShapeArcLengthFunctor(){}
    

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    ParametricShapeArcLengthFunctor & operator= ( const ParametricShapeArcLengthFunctor & other )
    {
      myShape = other.myShape;
      return *this;
    }

   
    /** .
     * Compute the length between two iterator.
     * 
     * @param itb begin point
     * @param ite end point
     * @param nbSamples number of samples used to approximate the
     * length (default= 1000 )
     * @return the estimated arc length.c
     */
    Quantity operator()(const ConstIteratorOnPoints &itb,const ConstIteratorOnPoints &ite, 
			const bool& isClosed = false, const unsigned int nbSamples = 10000)
    {
//TODO determining nbSamples from the bounding box size of the shape
      ASSERT(myShape);
      if (isClosed) {
        return myShape->arclength (0,2*M_PI,nbSamples); 
      } else {
        typename ParametricShape::RealPoint2D p = *itb;
        ConstIteratorOnPoints i = itb;
        ConstIteratorOnPoints j(i); ++j; 
        for ( ; j != ite; ++i, ++j);
        typename ParametricShape::RealPoint2D p2 = *i;  
       
        double t = myShape->parameter( p );
        double t2 = myShape->parameter( p2 );
        return myShape->arclength (t,t2,nbSamples); 
      }

    }
    

    // ------------------------- Private Datas --------------------------------
  private:
    
    ///Copy of the implicit shape.
    ParametricShape *myShape;
    
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of class ParametricShapeArcLengthFunctor
  
} // namespace DGtal

                                                                        //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ParametricShapeArcLengthFunctor_h

#undef ParametricShapeArcLengthFunctor_RECURSES
#endif // else defined(ParametricShapeArcLengthFunctor_RECURSES)
