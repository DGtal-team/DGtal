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
 * @file BasicConvolutionKernels.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/03/06
 *
 * Header file for module BasicConvolutionKernels.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(BasicConvolutionKernels_RECURSES)
#error Recursive header files inclusion detected in BasicConvolutionKernels.h
#else // defined(BasicConvolutionKernels_RECURSES)
/** Prevents recursive inclusion of headers. */
#define BasicConvolutionKernels_RECURSES

#if !defined BasicConvolutionKernels_h
/** Prevents repeated inclusion of headers. */
#define BasicConvolutionKernels_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ConstantConvolutionKernel
  /**
   * Description of template class 'ConstantConvolutionKernel' <p>
   * \brief Aim: implement a trivial constant convolution kernel which
   * returns 1 for each vector.
   *
   *   @tparam TVector type for displacement vectors.
   */
  template <typename TVector>
  class ConstantConvolutionKernel
  {
    // ----------------------- Standard services ------------------------------
  public:
    
    typedef TVector Vector;

    inline
    double operator()(const Vector &/*aDisplacment*/) const
    {
      return 1.0;
    } 
  };
  
  /////////////////////////////////////////////////////////////////////////////
  // template class ConstantConvolutionKernel
  /**
   * Description of template class 'GaussianConvolutionKernel' <p>
   * \brief Aim: implement a Gaussian centered convolution kernel.
   *
   *   @tparam TVector type for displacement vectors.
   */
  template <typename TVector>
  class GaussianConvolutionKernel
  {
    // ----------------------- Standard services ------------------------------
  public:
      
    typedef TVector Vector;
      
    GaussianConvolutionKernel(const double sigma): mySigma(sigma)
    {}
      
    inline
    double operator()(const Vector &aDisplacment) const
    {
      return 1.0/(mySigma* sqrt(2.0*M_PI))*
	exp(  -aDisplacment.norm()*aDisplacment.norm()/(2.0*mySigma));
    } 
     
    double mySigma;
    
  }; 




} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////

#endif // !defined BasicConvolutionKernels_h

#undef BasicConvolutionKernels_RECURSES
#endif // else defined(BasicConvolutionKernels_RECURSES)
