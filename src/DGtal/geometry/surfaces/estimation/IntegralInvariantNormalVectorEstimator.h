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
 * @file IntegralInvariantNormalVectorEstimator.h
 * @author Jeremy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2014/04/24
 *
 * Header file for module IntegralInvariantNormalVectorEstimator.ih
 *
 * This file is part of the DGtal library.
 */

#if defined(IntegralInvariantNormalVectorEstimator_RECURSES)
#error Recursive header files inclusion detected in IntegralInvariantNormalVectorEstimator.h
#else // defined(IntegralInvariantNormalVectorEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IntegralInvariantNormalVectorEstimator_RECURSES

#if !defined IntegralInvariantNormalVectorEstimator_h
/** Prevents repeated inclusion of headers. */
#define IntegralInvariantNormalVectorEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/shapes/Shapes.h"

#include "DGtal/geometry/surfaces/DigitalSurfaceConvolver.h"
#include "DGtal/shapes/EuclideanShapesDecorator.h"

#include "DGtal/shapes/implicit/ImplicitBall.h"
#include "DGtal/kernel/CCellFunctor.h"
//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// template class IntegralInvariantNormalVectorEstimator
/**
* Description of template class 'IntegralInvariantNormalVectorEstimator' <p>
* \brief Aim: This class implement an Integral Invariant normal vector estimator.
*
* @see related article:
*       Coeurjolly, D.; Lachaud, J.O; Levallois, J., (2013). Integral based Curvature
*       Estimators in Digital Geometry. DGCI 2013. Retrieved from
*       https://liris.cnrs.fr/publis/?id=5866
*
* The algorithm we propose uses volume of a kernel (2D: Ball2D, 3D:
* Ball3D) to approximate the normal vector.  To compute the volume, we
* convolve a kernel around the surface and then count the number of
* cells belonging the shape.  Theorical multigrid convergence is
* proved, with a convergence speed of O(h^1/3) with hypothesis about
* the shape geometry and the convolution kernel radius.  Experimental
* results showed a multigrid convergence.
*
* Some optimization is available when we give a range of 0-adjacent surfels to the estimator.
*
* @tparam TKSpace space in which the shape is defined.
* @tparam TShapeSpelFunctor TFunctor a model of a functor for the shape ( f(x) ).
*
* @see exampleIntegralInvariantCurvature2D.cpp testIntegralInvariantMeanCurvature3D.cpp testIntegralInvariantCurvature2D.cpp
*/
template <typename TKSpace, typename TShapeSpelFunctor>
class IntegralInvariantNormalVectorEstimator
{
public:
  typedef IntegralInvariantNormalVectorEstimator< TKSpace, TShapeSpelFunctor> Self;
  typedef TKSpace KSpace;
  typedef TShapeSpelFunctor ShapeSpelFunctor;
  typedef typename KSpace::Space Space;
  typedef HyperRectDomain<Space> Domain;
  typedef typename Space::RealPoint RealPoint;
  typedef typename DigitalSetSelector<Domain,  BIG_DS + HIGH_VAR_DS>::Type DigitalSet;
  typedef typename KSpace::SCell Spel;
  typedef typename KSpace::SurfelSet SurfelSet;
  typedef typename SurfelSet::const_iterator ConstIteratorKernel;

  typedef double Quantity;
  typedef int Value;

  typedef ConstValueCellFunctor<Value, Spel> KernelSpelFunctor;
  typedef ImplicitBall<Space> KernelSupport;
  typedef EuclideanShapesMinus< KernelSupport, KernelSupport > EuclideanMinus;
  typedef GaussDigitizer< Space, KernelSupport > DigitalShapeKernel;
  typedef GaussDigitizer< Space, EuclideanMinus > DigitalShape;

  typedef DigitalSurfaceConvolver<ShapeSpelFunctor, KernelSpelFunctor, 
                                  KSpace, DigitalShapeKernel> Convolver;
  typedef typename Convolver::PairIterators PairIterators;

  BOOST_CONCEPT_ASSERT (( CCellFunctor< ShapeSpelFunctor > ));

  // ----------------------- Standard services ------------------------------
public:
  /**
  * Default constructor. The object is invalid. The user needs to call
  * setParams and attach.
  */
  IntegralInvariantNormalVectorEstimator();
  /**
  * Constructor.
  *
  * @param[in] K the cellular grid space in which the shape is defined.
  * @param[in] aShapeSpelFunctor the shape of interest. The alias can be secured
  * if a some counted pointer is handed.
  */
  IntegralInvariantNormalVectorEstimator ( ConstAlias< KSpace > K, 
                                           ConstAlias< ShapeSpelFunctor > aShapeSpelFunctor );

  /**
  * Destructor.
  */
  ~IntegralInvariantNormalVectorEstimator();

  /**
  * Copy constructor.
  * @param other the object to clone.
  */
  IntegralInvariantNormalVectorEstimator ( const Self& other );

  /**
  * Assignment.
  * @param other the object to copy.
  * @return a reference on 'this'.
  */
  Self& operator= ( const Self& other );

  // ----------------------- Interface --------------------------------------
public:

  /// @return the grid step.
  Scalar h() const;

  /**
  * Attach a shape, defined as a functor spel -> boolean
  *
  * @param[in] K the cellular grid space in which the shape is defined.
  * @param aShapeSpelFunctor the shape of interest. The alias can be secured
  * if a some counted pointer is handed.
  */
  void attach( ConstAlias< KSpace > K, 
               ConstAlias<ShapeSpelFunctor> aShapeSpelFunctor );

  /**
  * Set specific parameters: the radius of the ball.
  *
  * @param[in] re Euclidean radius of the kernel support
  */
  void setParams( const double re );
  
  /**
  * Initialise the estimator with a specific Euclidean kernel radius re, and grid step _h.
  *
  * @param[in] _h precision of the grid
  * @param[in] re Euclidean radius of the kernel support
  */
  void init ( const double _h );

  /**
  * -- Normal vector -- 
  *
  * Compute the integral invariant normal vector
  * at surfel *it of a shape. Not so easy, since II is a symmetric
  * matrix, only directions of eigenvectors are pertinent. Another
  * computation is necessary to obtain the orientation.
  *
  * @tparam SurfelIterator type of Iterator on a Surfel
  *
  * @param[in] it iterator of a surfel (from a shape) we want compute the integral invariant mean curvature.
  *
  * @return quantity (normal vector) at surfel *it
  */
  template< typename SurfelIterator >
  Quantity eval ( const SurfelIterator & it ) const;


  /**
  * -- Normal vector -- 
  *
  * Compute the integral invariant normal vector from a range of surfels [itb,ite)
  * of a shape. Not so easy, since II is a symmetric
  * matrix, only directions of eigenvectors are pertinent. Another
  * computation is necessary to obtain the orientation.
  * Return the result on an OutputIterator (param).
  *
  * @tparam OutputIterator type of Iterator of an array of Quantity
  * @tparam SurfelConstIterator type of Iterator on a Surfel
  *
  * @param[in] itb iterator defining the start of the range of surfels where the normal vector is computed.
  * @param[in] ite iterator defining the end of the range of surfels where the normal vector is computed.
  * @param[in] output iterator of results of the computation.
  * @return the updated output iterator after all outputs.
  */
  template <typename OutputIterator, typename SurfelConstIterator>
  OutputIterator eval( SurfelConstIterator itb,
                       SurfelConstIterator ite,
                       OutputIterator result ) const;

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

  // ------------------------- Private Datas --------------------------------
private:

  const KernelSpelFunctor myKernelFunctor;///< Kernel functor (on Spel)
  std::vector< PairIterators > myKernels;   ///< array of begin/end iterator of shifting masks.
  std::vector< DigitalSet * > myKernelsSet; ///< Array of shifting masks. Size = 9 for each shifting (0-adjacent and full kernel included)
  CowPtr<KernelSupport> myKernel;           ///< Euclidean kernel
  CowPtr<DigitalShapeKernel> myDigKernel;   ///< Digital kernel
  CowPtr<Convolver> myConvolver;          ///< Convolver
  double myH;                             ///< precision of the grid
  double myRadius;                        ///< "digital" radius of the kernel (buy may be non integer).

private:


}; // end of class IntegralInvariantNormalVectorEstimator

  /**
  * Overloads 'operator<<' for displaying objects of class 'IntegralInvariantNormalVectorEstimator'.
  * @param out the output stream where the object is written.
  * @param object the object of class 'IntegralInvariantNormalVectorEstimator' to write.
  * @return the output stream after the writing.
  */
  template <typename TKSpace, typename TShapeSpelFunctor>
  std::ostream&
  operator<< ( std::ostream & out, 
               const IntegralInvariantNormalVectorEstimator<TKSpace, TShapeSpelFunctor> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantNormalVectorEstimator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IntegralInvariantNormalVectorEstimator_h

#undef IntegralInvariantNormalVectorEstimator_RECURSES
#endif // else defined(IntegralInvariantNormalVectorEstimator_RECURSES)
