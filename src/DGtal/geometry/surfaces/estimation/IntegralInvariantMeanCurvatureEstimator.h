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
 * @file IntegralInvariantMeanCurvatureEstimator.h
 * @author Jeremy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2012/04/19
 *
 * Header file for module IntegralInvariantMeanCurvatureEstimator.ih
 *
 * This file is part of the DGtal library.
 */

#if defined(IntegralInvariantMeanCurvatureEstimator_RECURSES)
#error Recursive header files inclusion detected in IntegralInvariantMeanCurvatureEstimator.h
#else // defined(IntegralInvariantMeanCurvatureEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IntegralInvariantMeanCurvatureEstimator_RECURSES

#if !defined IntegralInvariantMeanCurvatureEstimator_h
/** Prevents repeated inclusion of headers. */
#define IntegralInvariantMeanCurvatureEstimator_h

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

template< typename Quantity >
class MeanCurvatureFunctor3 : std::unary_function <double,double>
{
public:

  MeanCurvatureFunctor3(){}

  void init( const double & h, const double & r )
  {
    d8_3r = 8.0 / ( 3.0 * r );
    double r2 = r * r;
    d_4_PIr4 = 4.0 / ( M_PI * r2 * r2 );
    dh3 = h * h * h;
  }

  Quantity operator()(const Quantity & aInput) const
  {
    Quantity cp_quantity = aInput;
    cp_quantity *= dh3;

    return d8_3r - d_4_PIr4 * cp_quantity;
  }

private:
  Quantity dh3;
  Quantity d8_3r;
  Quantity d_4_PIr4;

};

template< typename Quantity >
class MeanCurvatureFunctor2 : std::unary_function <double,double>
{
public:

  MeanCurvatureFunctor2(){}

  void init( const double & h, const double & r )
  {
    d1_r2 = 1.0 / ( r * r );
    dPI_2 = M_PI / 2.0;
    d3_r = 3.0 / r;
    dh2 = h * h;
  }

  Quantity operator()(const Quantity & aInput) const
  {
    Quantity cp_quantity = aInput;
    cp_quantity *= dh2;

    return d3_r * ( dPI_2 - d1_r2 * cp_quantity );
  }

private:
  Quantity dh2;
  Quantity d3_r;
  Quantity dPI_2;
  Quantity d1_r2;

};

/////////////////////////////////////////////////////////////////////////////
// template class IntegralInvariantMeanCurvatureEstimator
/**
* Description of template class 'IntegralInvariantMeanCurvatureEstimator' <p>
* \brief Aim: This class implement a Integral Invariant mean curvature estimation.
*
* @see related article:
*       Coeurjolly, D.; Lachaud, J.O; Levallois, J., (2013). Integral based Curvature
*       Estimators in Digital Geometry. DGCI 2013. Retrieved from
*       https://liris.cnrs.fr/publis/?id=5866
*
* The algorithm we propose uses volume of a kernel (2D: Ball2D, 3D: Ball3D) to approximate mean curvature.
* To compute the volume, we convolve a kernel around the surface and counting the number of cells belonging the shape.
* Theorical multigrid convergence is proved, with a convergence speed of O(h^1/3) with hypothesis about the shape geometry
* and the convolution kernel radius.
* Experimental results showed a multigrid convergence.
*
* Some optimization is available when we set a range of 0-adjacent surfels to the estimator.
*
* @tparam TKSpace space in which the shape is defined.
* @tparam TShapeFunctor TFunctor a model of a functor for the shape ( f(x) ).
* @tparam dimension dimension of the shape. Let default value to use the correct specialization.
*
* @see exampleIntegralInvariantCurvature2D.cpp testIntegralInvariantMeanCurvature3D.cpp testIntegralInvariantCurvature2D.cpp
*/
template <typename TKSpace, typename TShapeFunctor, Dimension dimension = TKSpace::dimension>
class IntegralInvariantMeanCurvatureEstimator
{
public:
  typedef TKSpace KSpace;
  typedef typename Z2i::Domain Domain;
  typedef typename KSpace::Space::RealPoint RealPoint;
  typedef typename Z2i::DigitalSet DigitalSet;
  typedef typename KSpace::SCell Spel;
  typedef typename KSpace::SurfelSet SurfelSet;
  typedef typename SurfelSet::const_iterator ConstIteratorKernel;

  typedef double Quantity;
  typedef int Value;

  typedef TShapeFunctor ShapeSpelFunctor;
  typedef ConstValueCellFunctor<Value, Spel> KernelSpelFunctor;
  typedef ImplicitBall<Z2i::Space> KernelSupport;
  typedef EuclideanShapesMinus< KernelSupport, KernelSupport > EuclideanMinus;
  typedef GaussDigitizer< Z2i::Space, KernelSupport > DigitalShapeKernel;
  typedef GaussDigitizer< Z2i::Space, EuclideanMinus > DigitalShape;

  typedef MeanCurvatureFunctor2< Quantity > ValuesFunctor;

  typedef DigitalSurfaceConvolver<ShapeSpelFunctor, KernelSpelFunctor, KSpace, DigitalShapeKernel> Convolver;
  typedef typename Convolver::PairIterators PairIterators;

  BOOST_CONCEPT_ASSERT (( CCellFunctor< ShapeSpelFunctor > ));

  // ----------------------- Standard services ------------------------------
public:
  /**
  * Constructor.
  *
  * @param[in] space space in which the shape is defined.
  * @param[in] f functor on spel of the shape.
  */
  IntegralInvariantMeanCurvatureEstimator ( ConstAlias< KSpace > space, ConstAlias< ShapeSpelFunctor > f );

  /**
  * Destructor.
  */
  ~IntegralInvariantMeanCurvatureEstimator()
  {
    for( unsigned int i = 0; i < kernelsSet.size(); ++i )
    {
      delete kernelsSet[ i ];
    }
    kernelsSet.clear();
    delete kernel;
    delete digKernel;
  }

  // ----------------------- Interface --------------------------------------
public:

  /**
  * Initialise the estimator with a specific Euclidean kernel radius re, and grid step _h.
  *
  * @param[in] _h precision of the grid
  * @param[in] re Euclidean radius of the kernel support
  */
  void init ( const double _h, const double re );

  /**
  * -- Mean curvature --
  * Compute the integral invariant mean curvature at surfel *it of a shape.
  *
  * @tparam SurfelIterator type of Iterator on a Surfel
  *
  * @param[in] it iterator of a surfel (from a shape) we want compute the integral invariant mean curvature.
  *
  * @return quantity (mean curvature) at surfel *it
  */
  template< typename SurfelIterator >
  Quantity eval ( const SurfelIterator & it ) const;


  /**
  * -- Mean curvature --
  * Compute the integral invariant mean curvature from two surfels (from *itb to *ite (exclude) ) of a shape.
  * Return the result on an OutputIterator (param).
  *
  * @tparam SurfelIterator type of Iterator on a Surfel
  * @tparam OutputIterator type of Iterator of an array of Quantity
  *
  * @param[in] ite iterator of the begin surfel on the shape we want compute the integral invariant Gaussian curvature.
  * @param[in] itb iterator of the end surfel (excluded) on the shape we want compute the integral invariant Gaussiaan curvature.
  * @param[out] result iterator of results of the computation.
  */
  template< typename SurfelIterator, typename OutputIterator >
  void eval ( const SurfelIterator & itb,
              const SurfelIterator & ite,
              OutputIterator & result ) const;

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

  std::vector< PairIterators > kernels; ///< array of begin/end iterator of shifting masks.

  std::vector< DigitalSet * > kernelsSet; ///< Array of shifting masks. Size = 9 for each shifting (0-adjacent and full kernel included)

  KernelSupport * kernel; ///< Euclidean kernel

  DigitalShapeKernel * digKernel; ///< Digital kernel

  const KernelSpelFunctor myKernelFunctor; ///< Kernel functor (on Spel)

  Convolver myConvolver; ///< Convolver

  double h; ///< precision of the grid

  double radius; ///< Euclidean radius of the kernel

  ValuesFunctor meanFunctor; ///< Functor to transform covarianceMatrix to Quantity

private:

  /**
  * Copy constructor.
  * @param other the object to clone.
  * Forbidden by default.
  */
  IntegralInvariantMeanCurvatureEstimator ( const IntegralInvariantMeanCurvatureEstimator & other );

  /**
  * Assignment.
  * @param other the object to copy.
  * @return a reference on 'this'.
  * Forbidden by default.
  */
  IntegralInvariantMeanCurvatureEstimator & operator= ( const IntegralInvariantMeanCurvatureEstimator & other );

}; // end of class IntegralInvariantMeanCurvatureEstimator

/**
* Specialization for dimension = 2
*/
template <typename TKSpace, typename TShapeFunctor>
class IntegralInvariantMeanCurvatureEstimator<TKSpace, TShapeFunctor, 2>
{
public:
  typedef TKSpace KSpace;
  typedef typename Z2i::Domain Domain;
  typedef typename KSpace::Space::RealPoint RealPoint;
  typedef typename Z2i::DigitalSet DigitalSet;
  typedef typename KSpace::SCell Spel;
  typedef typename KSpace::SurfelSet SurfelSet;
  typedef typename SurfelSet::const_iterator ConstIteratorKernel;

  typedef double Quantity;
  typedef int Value;

  typedef TShapeFunctor ShapeSpelFunctor;
  typedef ConstValueCellFunctor<Value, Spel> KernelSpelFunctor;
  typedef ImplicitBall<Z2i::Space> KernelSupport;
  typedef EuclideanShapesMinus< KernelSupport, KernelSupport > EuclideanMinus;
  typedef GaussDigitizer< Z2i::Space, KernelSupport > DigitalShapeKernel;
  typedef GaussDigitizer< Z2i::Space, EuclideanMinus > DigitalShape;

  typedef MeanCurvatureFunctor2< Quantity > ValuesFunctor;

  typedef DigitalSurfaceConvolver<ShapeSpelFunctor, KernelSpelFunctor, KSpace, DigitalShapeKernel> Convolver;
  typedef typename Convolver::PairIterators PairIterators;

  BOOST_CONCEPT_ASSERT (( CCellFunctor< ShapeSpelFunctor > ));

  // ----------------------- Standard services ------------------------------
public:
  /**
  * Constructor.
  *
  * @param[in] space space in which the shape is defined.
  * @param[in] f functor on spel of the shape.
  */
  IntegralInvariantMeanCurvatureEstimator ( ConstAlias< KSpace > space, ConstAlias< ShapeSpelFunctor > f );

  /**
  * Destructor.
  */
  ~IntegralInvariantMeanCurvatureEstimator()
  {
    for( unsigned int i = 0; i < kernelsSet.size(); ++i )
    {
      delete kernelsSet[ i ];
    }
    kernelsSet.clear();
    delete kernel;
    delete digKernel;
  }

  // ----------------------- Interface --------------------------------------
public:

  /**
  * Initialise the estimator with a specific Euclidean kernel radius re, and grid step _h.
  *
  * @param[in] _h precision of the grid
  * @param[in] re Euclidean radius of the kernel support
  */
  void init ( const double _h, const double re );

  /**
  * -- Mean curvature --
  * Compute the integral invariant mean curvature at surfel *it of a shape.
  *
  * @tparam SurfelIterator type of Iterator on a Surfel
  *
  * @param[in] it iterator of a surfel (from a shape) we want compute the integral invariant mean curvature.
  *
  * @return quantity (mean curvature) at surfel *it
  */
  template< typename SurfelIterator >
  Quantity eval ( const SurfelIterator & it ) const;


  /**
  * -- Mean curvature --
  * Compute the integral invariant mean curvature from two surfels (from *itb to *ite (exclude) ) of a shape.
  * Return the result on an OutputIterator (param).
  *
  * @tparam SurfelIterator type of Iterator on a Surfel
  * @tparam OutputIterator type of Iterator of an array of Quantity
  *
  * @param[in] ite iterator of the begin surfel on the shape we want compute the integral invariant Gaussian curvature.
  * @param[in] itb iterator of the end surfel (excluded) on the shape we want compute the integral invariant Gaussiaan curvature.
  * @param[out] result iterator of results of the computation.
  */
  template< typename SurfelIterator, typename OutputIterator >
  void eval ( const SurfelIterator & itb,
              const SurfelIterator & ite,
              OutputIterator & result ) const;

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

  std::vector< PairIterators > kernels; ///< array of begin/end iterator of shifting masks.

  std::vector< DigitalSet * > kernelsSet; ///< Array of shifting masks. Size = 9 for each shifting (0-adjacent and full kernel included)

  KernelSupport * kernel; ///< Euclidean kernel

  DigitalShapeKernel * digKernel; ///< Digital kernel

  const KernelSpelFunctor myKernelFunctor; ///< Kernel functor (on Spel)

  Convolver myConvolver; ///< Convolver

  double h; ///< precision of the grid

  double radius; ///< Euclidean radius of the kernel

  ValuesFunctor meanFunctor; ///< Functor to transform covarianceMatrix to Quantity

private:

  /**
  * Copy constructor.
  * @param other the object to clone.
  * Forbidden by default.
  */
  IntegralInvariantMeanCurvatureEstimator ( const IntegralInvariantMeanCurvatureEstimator & other );

  /**
  * Assignment.
  * @param other the object to copy.
  * @return a reference on 'this'.
  * Forbidden by default.
  */
  IntegralInvariantMeanCurvatureEstimator & operator= ( const IntegralInvariantMeanCurvatureEstimator & other );
}; // end of class IntegralInvariantMeanCurvatureEstimator for dimension = 2

/**
* Specialization for dimension = 3
*/
template <typename TKSpace, typename TShapeFunctor>
class IntegralInvariantMeanCurvatureEstimator<TKSpace, TShapeFunctor, 3>
{
public:
  typedef TKSpace KSpace;
  typedef typename Z3i::Domain Domain;
  typedef typename KSpace::Space::RealPoint RealPoint;
  typedef typename Z3i::DigitalSet DigitalSet;
  typedef typename KSpace::SCell Spel;
  typedef typename KSpace::SurfelSet SurfelSet;
  typedef typename SurfelSet::const_iterator ConstIteratorKernel;

  typedef double Quantity;
  typedef int Value;

  typedef TShapeFunctor ShapeSpelFunctor;
  typedef ConstValueCellFunctor<Value, Spel> KernelSpelFunctor;
  typedef ImplicitBall<Z3i::Space> KernelSupport;
  typedef EuclideanShapesMinus< KernelSupport, KernelSupport > EuclideanMinus;
  typedef GaussDigitizer< Z3i::Space, KernelSupport > DigitalShapeKernel;
  typedef GaussDigitizer< Z3i::Space, EuclideanMinus > DigitalShape;

  typedef MeanCurvatureFunctor3< Quantity > ValuesFunctor;

  typedef DigitalSurfaceConvolver<ShapeSpelFunctor, KernelSpelFunctor, KSpace, DigitalShapeKernel> Convolver;
  typedef typename Convolver::PairIterators PairIterators;

  BOOST_CONCEPT_ASSERT (( CCellFunctor< ShapeSpelFunctor > ));

  // ----------------------- Standard services ------------------------------
public:
  /**
  * Constructor.
  *
  * @param[in] space space in which the shape is defined.
  * @param[in] f functor on spel of the shape.
  */
  IntegralInvariantMeanCurvatureEstimator ( ConstAlias< KSpace > space, ConstAlias< ShapeSpelFunctor > f );

  /**
  * Destructor.
  */
  ~IntegralInvariantMeanCurvatureEstimator()
  {
    for( unsigned int i = 0; i < kernelsSet.size(); ++i )
    {
      delete kernelsSet[ i ];
    }
    kernelsSet.clear();
    delete kernel;
    delete digKernel;
  }

  // ----------------------- Interface --------------------------------------
public:

  /**
  * Initialise the estimator with a specific Euclidean kernel radius re, and grid step _h.
  *
  * @param[in] _h precision of the grid
  * @param[in] re Euclidean radius of the kernel support
  */
  void init ( const double _h, const double re );

  /**
  * -- Mean curvature --
  * Compute the integral invariant mean curvature at surfel *it of a shape.
  *
  * @tparam SurfelIterator type of Iterator on a Surfel
  *
  * @param[in] it iterator of a surfel (from a shape) we want compute the integral invariant mean curvature.
  *
  * @return quantity (mean curvature) at surfel *it
  */
  template< typename SurfelIterator >
  Quantity eval ( const SurfelIterator & it ) const;

  /**
  * -- Mean curvature --
  * Compute the integral invariant mean curvature from two surfels (from *itb to *ite (exclude) ) of a shape.
  * Return the result on an OutputIterator (param).
  *
  * @tparam SurfelIterator type of Iterator on a Surfel
  * @tparam OutputIterator type of Iterator of an array of Quantity
  *
  * @param[in] ite iterator of the begin surfel on the shape we want compute the integral invariant Gaussian curvature.
  * @param[in] itb iterator of the end surfel (excluded) on the shape we want compute the integral invariant Gaussiaan curvature.
  * @param[out] result iterator of results of the computation.
  */
  template< typename SurfelIterator, typename OutputIterator >
  void eval ( const SurfelIterator & itb,
              const SurfelIterator & ite,
              OutputIterator & result ) const;

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

  std::vector< PairIterators > kernels; ///< array of begin/end iterator of shifting masks.

  std::vector< DigitalSet * > kernelsSet; ///< Array of shifting masks. Size = 9 for each shifting (0-adjacent and full kernel included)

  KernelSupport * kernel; ///< Euclidean kernel

  DigitalShapeKernel * digKernel; ///< Digital kernel

  const KernelSpelFunctor myKernelFunctor; ///< Kernel functor (on Spel)

  Convolver myConvolver; ///< Convolver

  double h; ///< precision of the grid

  double radius; ///< Euclidean radius of the kernel

  ValuesFunctor meanFunctor; ///< Functor to transform covarianceMatrix to Quantity

private:

  /**
  * Copy constructor.
  * @param other the object to clone.
  * Forbidden by default.
  */
  IntegralInvariantMeanCurvatureEstimator ( const IntegralInvariantMeanCurvatureEstimator & other );

  /**
  * Assignment.
  * @param other the object to copy.
  * @return a reference on 'this'.
  * Forbidden by default.
  */
  IntegralInvariantMeanCurvatureEstimator & operator= ( const IntegralInvariantMeanCurvatureEstimator & other );

}; // end of specialization for dimension = 3









/**
* Overloads 'operator<<' for displaying objects of class 'IntegralInvariantMeanCurvatureEstimator'.
* @param out the output stream where the object is written.
* @param object the object of class 'IntegralInvariantMeanCurvatureEstimator' to write.
* @return the output stream after the writing.
*/
template <typename TKS, typename TSF, Dimension dimension>
std::ostream&
operator<< ( std::ostream & out, const IntegralInvariantMeanCurvatureEstimator<TKS, TSF, dimension> & object );

template <typename TKS, typename TSF>
std::ostream&
operator<< ( std::ostream & out, const IntegralInvariantMeanCurvatureEstimator<TKS, TSF, 2> & object );

template <typename TKS, typename TSF>
std::ostream&
operator<< ( std::ostream & out, const IntegralInvariantMeanCurvatureEstimator<TKS, TSF, 3> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantMeanCurvatureEstimator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IntegralInvariantMeanCurvatureEstimator_h

#undef IntegralInvariantMeanCurvatureEstimator_RECURSES
#endif // else defined(IntegralInvariantMeanCurvatureEstimator_RECURSES)
