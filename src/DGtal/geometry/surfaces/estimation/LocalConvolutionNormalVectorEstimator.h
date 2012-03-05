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
 * @file LocalConvolutionNormalVectorEstimator.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/03/03
 *
 * This file is part of the DGtal library.
 *
 * @see  testLocalConvolutionNormalVectorEstimator.cpp
 */

#if defined(LocalConvolutionNormalVectorEstimator_RECURSES)
#error Recursive header files inclusion detected in LocalConvolutionNormalVectorEstimator.h
#else // defined(LocalConvolutionNormalVectorEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define LocalConvolutionNormalVectorEstimator_RECURSES

#if !defined LocalConvolutionNormalVectorEstimator_h
/** Prevents repeated inclusion of headers. */
#define LocalConvolutionNormalVectorEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/topology/BreadthFirstVisitor.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/topology/BreadthFirstVisitor.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class LocalConvolutionNormalVectorEstimator
  /**
   * Description of template class 'LocalConvolutionNormalVectorEstimator' <p>
   * \brief Aim: Computes the normal vector at a surface element by
   * summation of elementary normal vector to adjacent surfel.
   *
   * To each $n-1$ signed surfel, an elementary outward normal vector can be
   * defined. At a given surfel, this estimator will compute the
   * weighted sum of elementary normal vector of neighboring surfel
   * using a breadth-first propagation around the given surfel.
   *
   * The neighboring is parametrized by a given topological radius @e R.
   * The weight function maps [O,R] to a continuous weights.
   *
   * @tparam TDigitalSurface type of digital surface on which we would
   * like to compute vector field..
   * @tparam TKernelFunctor type of Functor used to represent
   * convolution kernel functor. ma
   */
  template <typename TDigitalSurface, typename TKernelFunctor, typename TCellEmbedder>
  class LocalConvolutionNormalVectorEstimator
  {

    // ----------------------- Types ------------------------------
  public:

    typedef TCellEmbedder CellEmbedder;
    typedef TDigitalSurface DigitalSurface;
    typedef TKernelFunctor KernelFunctor;
    typedef typename TDigitalSurface::ConstIterator ConstIterator;
    typedef typename TDigitalSurface::KSpace::Space::RealVector Quantity;
    

    // ----------------------- Standard services ------------------------------
  public:

     /**
     * Constructor.
     * @param h grid size (must be >0).
     * @param itb, begin iterator
     * @param ite, end iterator
     */
    LocalConvolutionNormalVectorEstimator(const DigitalSurface & aSurface,
                                          const KernelFunctor & aFunctor,
					  const CellEmbedder & anEmbedder);
    
    /**
     * Destructor.
     */
    ~LocalConvolutionNormalVectorEstimator() {};

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Initialisation.
     * @param h grid size (must be >0).
      */
    void init(const double h, 
              const unsigned int radius);
    
    /**
     * @return the estimated quantity at *it
     */
    Quantity eval(const ConstIterator& it);
    
    /**
     * @return the estimated quantity
     * from itb till ite (exculded)
     */
    template <typename OutputIterator>
    OutputIterator eval(const ConstIterator& itb, 
                        const ConstIterator& ite, 
                        OutputIterator result); 
    
    
    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
  protected:
    /**
     * Default constructor.
     */
    LocalConvolutionNormalVectorEstimator() 
    {
      myFlagIsInit = false;
    }
   
 
    // ------------------------- Private Datas --------------------------------
  private:

    ///Grid size
    double myH; 
    
    ///True if the init() has been called.
    bool myFlagIsInit;
    
    ///Parametric quantity functor
    unsigned int myRadius;
    
    ///Copy of the kernel convolution functor.
    const KernelFunctor & myKernelFunctor;

    ///Copy to the digitale surface
    const DigitalSurface & mySurface;

    ///Copy of the cell embedder
    const CellEmbedder & myEmbedder;



    // ------------------------- Hidden services ------------------------------
  private:
    
    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    LocalConvolutionNormalVectorEstimator ( const LocalConvolutionNormalVectorEstimator & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    LocalConvolutionNormalVectorEstimator & operator= ( const LocalConvolutionNormalVectorEstimator & other );


  }; // end of class LocalConvolutionNormalVectorEstimator

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/LocalConvolutionNormalVectorEstimator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined LocalConvolutionNormalVectorEstimator_h

#undef LocalConvolutionNormalVectorEstimator_RECURSES
#endif // else defined(LocalConvolutionNormalVectorEstimator_RECURSES)
