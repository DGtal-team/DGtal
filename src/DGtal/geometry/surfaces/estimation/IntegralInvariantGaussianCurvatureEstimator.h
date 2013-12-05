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
 * @file IntegralInvariantGaussianCurvatureEstimator.h
 * @author Jeremy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2012/07/10
 *
 * Header file for module IntegralInvariantGaussianCurvatureEstimator.ih
 *
 * @brief Compute Gaussian curvature on border of shapes of n-dimension, based on integral invariant.
 *
 * @see related article:
 *       Coeurjolly, D.; Lachaud, J.O; Levallois, J., (2013). Integral based Curvature
 *       Estimators in Digital Geometry. DGCI 2013. Retrieved from
 *       https://liris.cnrs.fr/publis/?id=5866
 *
 * This file is part of the DGtal library.
 */

#if defined(IntegralInvariantGaussianCurvatureEstimator_RECURSES)
#error Recursive header files inclusion detected in IntegralInvariantGaussianCurvatureEstimator.h
#else // defined(IntegralInvariantGaussianCurvatureEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IntegralInvariantGaussianCurvatureEstimator_RECURSES

#if !defined IntegralInvariantGaussianCurvatureEstimator_h
/** Prevents repeated inclusion of headers. */
#define IntegralInvariantGaussianCurvatureEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/shapes/Shapes.h"

#include "DGtal/geometry/surfaces/DigitalSurfaceConvolver.h"
#include "DGtal/shapes/EuclideanShapesDecorator.h"

#include "DGtal/shapes/implicit/ImplicitBall.h"

#include "DGtal/math/EigenValues3D.h"
#include "DGtal/kernel/CCellFunctor.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

struct CurvatureInformations
{
  typedef double Value;
  typedef EigenValues3D< Value >::Vector3 Vector3;
  typedef SimpleMatrix< Value, 3, 3 > Matrix3x3;
  Value k1;
  Value k2;
  Vector3 values;
  Matrix3x3 vectors;

  friend std::ostream& operator <<(std::ostream&, const CurvatureInformations&);
  bool operator ==( const CurvatureInformations& other)
  {
    return ( k1 == other.k1 && k2 == other.k2 && values == other.values && vectors == other.vectors );
  }
  bool operator !=( const CurvatureInformations& other)
  {
    return !(this->operator ==( other));
  }
  bool operator <( const CurvatureInformations& other)
  {
    return (k1 < other.k1);
  }
};

template< typename Matrix2x2 >
class GaussianCurvatureFunctor2 : std::unary_function <double,double>
{
public:
  typedef double Value;

  GaussianCurvatureFunctor2(){}

  void init( const double & h, const double & r )
  {
    dh2 = h * h;
    d3_r = 3.0 / r;
    d1_r2 = 1.0 / ( r * r );
  }

  Value operator()(const Matrix2x2 & aInput)
  {
    Matrix2x2 cp_matrix = aInput;
    cp_matrix *= dh2;
    Value k = d3_r * ( M_PI_2 - d1_r2 * cp_matrix[ 0 ] );

    return k;
  }

private:
  Value dh2; /// h*h
  Value d3_r; /// 3/r
  Value d1_r2; /// 1/r^2

};

template< typename Matrix2x2 >
class PrincipalCurvatureFunctor2 : std::unary_function <double,double>
{
public:
  typedef double Quantity;
  typedef double Value;

  PrincipalCurvatureFunctor2(){}

  void init( const double & h, const double & r )
  {
    dh2 = h * h;
    d3_r = 3.0 / r;
    d1_r2 = 1.0 / ( r * r );
  }

  Value operator()( const Matrix2x2 & aInput, int nothing )
  {
    Matrix2x2 cp_matrix = aInput;
    cp_matrix *= dh2;
    Value k = d3_r * ( M_PI_2 - d1_r2 * cp_matrix[ 0 ] );

    trace.error() << "Unavailable yet." << std::endl;

    return k;
  }

private:
  Quantity dh2; /// h*h
  Quantity d3_r; /// 3/r
  Quantity d1_r2; /// 1/r^2

};

template< typename Matrix3x3 >
class GaussianCurvatureFunctor3 : std::unary_function <double,double>
{
public:
  typedef double Value;
  typedef EigenValues3D< Value >::Vector3 Vector3;

  GaussianCurvatureFunctor3(){}

  void init( const double & h, const double & r )
  {
    double r3 = r * r * r;
    double r6 = r3 * r3;
    d6_PIr6 = 6.0 / ( M_PI * r6 );
    d8_5r = 8.0 / ( 5.0 * r );
    double h2 = h * h;
    dh5 = h2 * h2 * h;
  }

  Value operator()(const Matrix3x3 & aInput)
  {
    Matrix3x3 cp_matrix = aInput;
    cp_matrix *= dh5;
    Value k1,k2;
    Matrix3x3 eigenVectors;
    Vector3 eigenValues;
    evalk1k2( cp_matrix, eigenVectors, eigenValues, k1, k2 );
    return k1 * k2;
  }

protected:
  void evalk1k2(
      Matrix3x3 & matrix,
      Matrix3x3 & eigenVectors,
      Vector3 & eigenValues,
      Value & k1,
      Value & k2 )
  {
    EigenValues3D< Value >::getEigenDecomposition( matrix, eigenVectors, eigenValues );

    ASSERT ( eigenValues[ 0 ] == eigenValues[ 0 ] ); // NaN
    ASSERT ( (eigenValues[ 0 ] <= eigenValues[ 2 ]) && (eigenValues[ 0 ] <= eigenValues[ 1 ]) && (eigenValues[ 1 ] <= eigenValues[ 2 ]) );

    k1 = d6_PIr6 * ( eigenValues[ 1 ] - ( 3.0 * eigenValues[ 2 ] )) + d8_5r;
    k2 = d6_PIr6 * ( eigenValues[ 2 ] - ( 3.0 * eigenValues[ 1 ] )) + d8_5r;
  }

private:
  Value dh5; /// h^5
  Value d6_PIr6; /// 6/(PI*r^6)
  Value d8_5r; /// 8/(5r)

};

template< typename Matrix3x3 >
class PrincipalCurvatureFunctor3 : std::unary_function <double,double>
{
public:
  typedef double Quantity;
  typedef EigenValues3D< Quantity >::Vector3 Vector3;
  typedef CurvatureInformations Value;

  PrincipalCurvatureFunctor3(){}

  void init( const double & h, const double & r )
  {
    double r3 = r * r * r;
    double r6 = r3 * r3;
    d6_PIr6 = 6.0 / ( M_PI * r6 );
    d8_5r = 8.0 / ( 5.0 * r );
    double h2 = h * h;
    dh5 = h2 * h2 * h;
  }

  Value operator()( const Matrix3x3 & aInput )
  {
    Matrix3x3 cp_matrix = aInput;
    cp_matrix *= dh5;
    Quantity k1,k2;
    Matrix3x3 eigenVectors;
    Vector3 eigenValues;
    evalk1k2( cp_matrix, eigenVectors, eigenValues, k1, k2 );
    Value result;
    result.k1 = k1;
    result.k2 = k2;
    result.values = eigenValues;
    result.vectors = eigenVectors;
    return result;
  }

protected:
  void evalk1k2(
      Matrix3x3 & matrix,
      Matrix3x3 & eigenVectors,
      Vector3 & eigenValues,
      Quantity & k1,
      Quantity & k2 )
  {
    EigenValues3D< Quantity >::getEigenDecomposition( matrix, eigenVectors, eigenValues );

    ASSERT ( eigenValues[ 0 ] == eigenValues[ 0 ] ); // NaN
    ASSERT ( (eigenValues[ 0 ] <= eigenValues[ 2 ]) && (eigenValues[ 0 ] <= eigenValues[ 1 ]) && (eigenValues[ 1 ] <= eigenValues[ 2 ]) );

    k1 = d6_PIr6 * ( eigenValues[ 1 ] - ( 3.0 * eigenValues[ 2 ] )) + d8_5r;
    k2 = d6_PIr6 * ( eigenValues[ 2 ] - ( 3.0 * eigenValues[ 1 ] )) + d8_5r;
  }

private:
  Quantity dh5; /// h^5
  Quantity d6_PIr6; /// 6/(PI*r^6)
  Quantity d8_5r; /// 8/(5r)

};

/////////////////////////////////////////////////////////////////////////////
// template class IntegralInvariantGaussianCurvatureEstimator
/**
* Description of template class 'IntegralInvariantMeanCurvatureEstimator' <p>
* \brief Aim: This class implement a Integral Invariant Gaussian curvature estimation.
*
* @see related article:
*       Coeurjolly, D.; Lachaud, J.O; Levallois, J., (2013). Integral based Curvature
*       Estimators in Digital Geometry. DGCI 2013. Retrieved from
*       https://liris.cnrs.fr/publis/?id=5866
*
* The algorithm we propose uses covariance matrix to approximate Gaussian curvature.
* To compute the covariance matrix on each surfel of the surface, we convolve a kernel (ImplicitBall) around
* the surface.
* The resulting covariance matrix gives us principal curvatures and principal directions by computing
* respectively eigenvectors and eigenvalues on it.
* Theorical multigrid convergence is proved with principal curvatures, with a convergence speed of O(h^1/3)
* with hypothesis about the shape geometry and the convolution kernel radius.
* Experimental results showed a multigrid convergence for principal curvatures.
*
* Some optimization is available when we set a range of 0-adjacent surfels to the estimator.
*
* @tparam TKSpace space in which the shape is defined.
* @tparam TShapeFunctor TFunctor a model of a functor for the shape ( f(x) ).
* @tparam dimension dimension of the shape. Let default value to use the correct specialization.
*
* @see exampleIntegralInvariantGaussianCurvature3D.cpp testIntegralInvariantGaussianCurvature3D.cpp
*/
template <typename TKSpace, typename TShapeFunctor, Dimension dimension = TKSpace::dimension>
class IntegralInvariantGaussianCurvatureEstimator
{
public:
  typedef TKSpace KSpace;
  typedef typename Z2i::Domain Domain;
  typedef typename KSpace::Space::RealPoint RealPoint;
  typedef typename Z2i::DigitalSet DigitalSet;
  typedef typename KSpace::SCell Spel;
  typedef typename KSpace::SurfelSet SurfelSet;
  typedef typename SurfelSet::const_iterator ConstIteratorKernel;

  typedef int Value;

  typedef TShapeFunctor ShapeSpelFunctor;
  typedef ConstValueCellFunctor<Value, Spel> KernelSpelFunctor;
  typedef ImplicitBall<Z2i::Space> KernelSupport;
  typedef EuclideanShapesMinus< KernelSupport, KernelSupport > EuclideanMinus;
  typedef GaussDigitizer< Z2i::Space, KernelSupport > DigitalShapeKernel;
  typedef GaussDigitizer< Z2i::Space, EuclideanMinus > DigitalShape;

  typedef DigitalSurfaceConvolver<ShapeSpelFunctor, KernelSpelFunctor, KSpace, DigitalShapeKernel> Convolver;

  typedef typename Convolver::CovarianceMatrix Matrix2x2;

  typedef GaussianCurvatureFunctor2< Matrix2x2 > ValuesFunctor;
  typedef PrincipalCurvatureFunctor2< Matrix2x2 > PrincipalCurvatureFunctor;

  typedef typename ValuesFunctor::Value Quantity;
  typedef typename PrincipalCurvatureFunctor::Value PrincipalCurvatures;

  typedef typename Convolver::PairIterators PairIterators;

  BOOST_CONCEPT_ASSERT (( CCellFunctor< ShapeSpelFunctor > ));

  // ----------------------- Standard services ------------------------------
public:
  /**
  * Constructor.
  *
  * @param space space in which the shape is defined.
  * @param f functor on spel of the shape.
  */
  IntegralInvariantGaussianCurvatureEstimator ( ConstAlias< KSpace > space, ConstAlias< ShapeSpelFunctor > f );

  /**
  * Destructor.
  */
  ~IntegralInvariantGaussianCurvatureEstimator()
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
  * -- Gaussian curvature --
  * Compute the integral invariant Gaussian curvature at surfel *it of a shape.
  *
  * @tparam SurfelIterator type of Iterator on a Surfel
  *
  * @param[in] it iterator of a surfel (from a shape) we want compute the integral invariant Gaussian curvature.
  *
  * @return quantity (Gaussian curvature) at position *it
  */
  template< typename SurfelIterator >
  Quantity eval ( const SurfelIterator & it );

  /**
  * -- Gaussian curvature --
  * Compute the integral invariant Gaussian curvature from two surfels (from *itb to *ite (exclude) ) of a shape.
  * Return the result on an OutputIterator (param).
  *
  * @tparam SurfelIterator type of Iterator on a Surfel
  * @tparam OutputIterator type of Iterator of a list of Quantity
  *
  * @param[in] ite iterator of the begin surfel on the shape we want compute the integral invariant Gaussian curvature.
  * @param[in] itb iterator of the end surfel (excluded) on the shape we want compute the integral invariant Gaussiaan curvature.
  * @param[out] result iterator of results of the computation.
  */
  template< typename SurfelIterator, typename OutputIterator >
  void eval ( const SurfelIterator & itb,
              const SurfelIterator & ite,
              OutputIterator & result );

  /**
  * -- Principal curvatures --
  * Compute the integral invariant Gaussian curvature on surfel *it of a shape.
  *
  * @tparam SurfelIterator iterator on a Surfel
  *
  * @param[in] it iterator of a surfel (from a shape) we want compute the integral invariant principal curvatures.
  *
  * @return a struct with principal curvatures value of Integral Invariant estimator at surfel *it, and eigenVectors
  * and eigenValues resulting of the covariance matrix (see above: struct CurvatureInformations )
  */
  template< typename SurfelIterator >
  PrincipalCurvatures evalPrincipalCurvatures ( const SurfelIterator & it );



  /**
  * -- Principal curvatures --
  * Compute the integral invariant Gaussian curvature from two surfels (from *itb to *ite (exclude) ) of a shape.
  * Return the result on an OutputIterator (param).
  *
  * @tparam SurfelIterator iterator on a Surfel
  * @tparam OutputIterator iterator of array of PrincipalCurvatures
  *
  * @param[in] ite iterator of the begin surfel on the shape where we compute the integral invariant principal curvatures.
  * @param[in] itb iterator of the end surfel (excluded) on the shape where we compute the integral invariant principal curvatures.
  * @param[out] result iterator of structs with principal curvatures value of Integral Invariant estimator at surfel *it, and eigenVectors
  * and eigenValues resulting of the covariance matrix (see above: struct CurvatureInformations )
  */
  template< typename SurfelIterator, typename OutputIterator >
  void evalPrincipalCurvatures ( const SurfelIterator & itb,
                                 const SurfelIterator & ite,
                                 OutputIterator & result );

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

  ValuesFunctor gaussFunctor; ///< Functor to transform covarianceMatrix to Quantity/PrincipalCurvatures
  PrincipalCurvatureFunctor princCurvFunctor;

private:

  /**
  * Copy constructor.
  * @param other the object to clone.
  * Forbidden by default.
  */
  IntegralInvariantGaussianCurvatureEstimator ( const IntegralInvariantGaussianCurvatureEstimator & other );

  /**
  * Assignment.
  * @param other the object to copy.
  * @return a reference on 'this'.
  * Forbidden by default.
  */
  IntegralInvariantGaussianCurvatureEstimator & operator= ( const IntegralInvariantGaussianCurvatureEstimator & other );

}; // end of class IntegralInvariantGaussianCurvatureEstimator

/**
* Specialization for dimension = 2
*/
template <typename TKSpace, typename TShapeFunctor>
class IntegralInvariantGaussianCurvatureEstimator<TKSpace, TShapeFunctor, 2>
{
public:
  typedef TKSpace KSpace;
  typedef typename Z2i::Domain Domain;
  typedef typename KSpace::Space::RealPoint RealPoint;
  typedef typename Z2i::DigitalSet DigitalSet;
  typedef typename KSpace::SCell Spel;
  typedef typename KSpace::SurfelSet SurfelSet;
  typedef typename SurfelSet::const_iterator ConstIteratorKernel;

  typedef int Value;

  typedef TShapeFunctor ShapeSpelFunctor;
  typedef ConstValueCellFunctor<Value, Spel> KernelSpelFunctor;
  typedef ImplicitBall<Z2i::Space> KernelSupport;
  typedef EuclideanShapesMinus< KernelSupport, KernelSupport > EuclideanMinus;
  typedef GaussDigitizer< Z2i::Space, KernelSupport > DigitalShapeKernel;
  typedef GaussDigitizer< Z2i::Space, EuclideanMinus > DigitalShape;

  typedef DigitalSurfaceConvolver<ShapeSpelFunctor, KernelSpelFunctor, KSpace, DigitalShapeKernel> Convolver;

  typedef typename Convolver::CovarianceMatrix Matrix2x2;

  typedef GaussianCurvatureFunctor2< Matrix2x2 > ValuesFunctor;
  typedef PrincipalCurvatureFunctor2< Matrix2x2 > PrincipalCurvatureFunctor;

  typedef typename ValuesFunctor::Value Quantity;
  typedef typename PrincipalCurvatureFunctor::Value PrincipalCurvatures;

  typedef typename Convolver::PairIterators PairIterators;

  BOOST_CONCEPT_ASSERT (( CCellFunctor< ShapeSpelFunctor > ));

  // ----------------------- Standard services ------------------------------
public:
  /**
  * Constructor.
  *
  * @param space space in which the shape is defined.
  * @param f functor on spel of the shape.
  */
  IntegralInvariantGaussianCurvatureEstimator ( ConstAlias< KSpace > space, ConstAlias< ShapeSpelFunctor > f );

  /**
  * Destructor.
  */
  ~IntegralInvariantGaussianCurvatureEstimator()
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
  * -- Gaussian curvature --
  * Compute the integral invariant Gaussian curvature at surfel *it of a shape.
  *
  * @tparam SurfelIterator type of Iterator on a Surfel
  *
  * @param[in] it iterator of a surfel (from a shape) we want compute the integral invariant Gaussian curvature.
  *
  * @return quantity (Gaussian curvature) at position *it
  */
  template< typename SurfelIterator >
  Quantity eval ( const SurfelIterator & it );

  /**
  * -- Gaussian curvature --
  * Compute the integral invariant Gaussian curvature from two surfels (from *itb to *ite (exclude) ) of a shape.
  * Return the result on an OutputIterator (param).
  *
  * @tparam SurfelIterator type of Iterator on a Surfel
  * @tparam OutputIterator type of Iterator of a list of Quantity
  *
  * @param[in] ite iterator of the begin surfel on the shape we want compute the integral invariant Gaussian curvature.
  * @param[in] itb iterator of the end surfel (excluded) on the shape we want compute the integral invariant Gaussiaan curvature.
  * @param[out] result iterator of results of the computation.
  */
  template< typename SurfelIterator, typename OutputIterator >
  void eval ( const SurfelIterator & itb,
              const SurfelIterator & ite,
              OutputIterator & result );

  /**
  * -- Principal curvatures --
  * Compute the integral invariant Gaussian curvature on surfel *it of a shape.
  *
  * @tparam SurfelIterator iterator on a Surfel
  *
  * @param[in] it iterator of a surfel (from a shape) we want compute the integral invariant principal curvatures.
  *
  * @return a struct with principal curvatures value of Integral Invariant estimator at surfel *it, and eigenVectors
  * and eigenValues resulting of the covariance matrix (see above: struct CurvatureInformations )
  */
  template< typename SurfelIterator >
  PrincipalCurvatures evalPrincipalCurvatures ( const SurfelIterator & it );



  /**
  * -- Principal curvatures --
  * Compute the integral invariant Gaussian curvature from two surfels (from *itb to *ite (exclude) ) of a shape.
  * Return the result on an OutputIterator (param).
  *
  * @tparam SurfelIterator iterator on a Surfel
  * @tparam OutputIterator iterator of array of PrincipalCurvatures
  *
  * @param[in] ite iterator of the begin surfel on the shape where we compute the integral invariant principal curvatures.
  * @param[in] itb iterator of the end surfel (excluded) on the shape where we compute the integral invariant principal curvatures.
  * @param[out] result iterator of structs with principal curvatures value of Integral Invariant estimator at surfel *it, and eigenVectors
  * and eigenValues resulting of the covariance matrix (see above: struct CurvatureInformations )
  */
  template< typename SurfelIterator, typename OutputIterator >
  void evalPrincipalCurvatures ( const SurfelIterator & itb,
                                 const SurfelIterator & ite,
                                 OutputIterator & result );

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

  ValuesFunctor gaussFunctor; ///< Functor to transform covarianceMatrix to Quantity/PrincipalCurvatures
  PrincipalCurvatureFunctor princCurvFunctor;

private:

  /**
  * Copy constructor.
  * @param other the object to clone.
  * Forbidden by default.
  */
  IntegralInvariantGaussianCurvatureEstimator ( const IntegralInvariantGaussianCurvatureEstimator & other );

  /**
  * Assignment.
  * @param other the object to copy.
  * @return a reference on 'this'.
  * Forbidden by default.
  */
  IntegralInvariantGaussianCurvatureEstimator & operator= ( const IntegralInvariantGaussianCurvatureEstimator & other );
}; // end of class IntegralInvariantGaussianCurvatureEstimator for dimension = 2

/**
* Specialization for dimension = 3
*/
template <typename TKSpace, typename TShapeFunctor>
class IntegralInvariantGaussianCurvatureEstimator<TKSpace, TShapeFunctor, 3>
{
public:
  typedef TKSpace KSpace;
  typedef typename Z3i::Domain Domain;
  typedef typename KSpace::Space::RealPoint RealPoint;
  typedef typename Z3i::DigitalSet DigitalSet;
  typedef typename KSpace::SCell Spel;
  typedef typename KSpace::SurfelSet SurfelSet;
  typedef typename SurfelSet::const_iterator ConstIteratorKernel;

  typedef int Value;

  typedef TShapeFunctor ShapeSpelFunctor;
  typedef ConstValueCellFunctor<Value, Spel> KernelSpelFunctor;
  typedef ImplicitBall<Z3i::Space> KernelSupport;
  typedef EuclideanShapesMinus< KernelSupport, KernelSupport > EuclideanMinus;
  typedef GaussDigitizer< Z3i::Space, KernelSupport > DigitalShapeKernel;
  typedef GaussDigitizer< Z3i::Space, EuclideanMinus > DigitalShape;

  typedef DigitalSurfaceConvolver<ShapeSpelFunctor, KernelSpelFunctor, KSpace, DigitalShapeKernel> Convolver;

  typedef typename Convolver::CovarianceMatrix Matrix3x3;

  typedef GaussianCurvatureFunctor3< Matrix3x3 > ValuesFunctor;
  typedef PrincipalCurvatureFunctor3< Matrix3x3 > PrincipalCurvatureFunctor;

  typedef typename ValuesFunctor::Value Quantity;
  typedef typename PrincipalCurvatureFunctor::Value PrincipalCurvatures;

  typedef typename EigenValues3D< Quantity >::Vector3 Vector3;

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
  IntegralInvariantGaussianCurvatureEstimator ( ConstAlias< KSpace > space, ConstAlias< ShapeSpelFunctor > f );

  /**
  * Destructor.
  */
  ~IntegralInvariantGaussianCurvatureEstimator()
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
  * -- Gaussian curvature --
  * Compute the integral invariant Gaussian curvature at surfel *it of a shape.
  *
  * @tparam SurfelIterator type of Iterator on a Surfel
  *
  * @param[in] it iterator of a surfel (from a shape) we want compute the integral invariant Gaussian curvature.
  *
  * @return quantity (Gaussian curvature) at surfel *it
  */
  template< typename SurfelIterator >
  Quantity eval ( const SurfelIterator & it );

  /**
  * -- Gaussian curvature --
  * Compute the integral invariant Gaussian curvature from two surfels (from *itb to *ite (exclude) ) of a shape.
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
              OutputIterator & result );

  /**
  * -- Principal curvatures --
  * Compute the integral invariant Gaussian curvature on surfel *it of a shape.
  *
  * @tparam SurfelIterator iterator on a Surfel
  *
  * @param[in] it iterator of a surfel (from a shape) we want compute the integral invariant principal curvatures.
  *
  * @return a struct with principal curvatures value of Integral Invariant estimator at surfel *it, and eigenVectors
  * and eigenValues resulting of the covariance matrix (see above: struct CurvatureInformations )
  */
  template< typename SurfelIterator >
  PrincipalCurvatures evalPrincipalCurvatures ( const SurfelIterator & it );



  /**
  * -- Principal curvatures --
  * Compute the integral invariant Gaussian curvature from two surfels (from *itb to *ite (exclude) ) of a shape.
  * Return the result on an OutputIterator (param).
  *
  * @tparam SurfelIterator iterator on a Surfel
  * @tparam OutputIterator iterator of array of PrincipalCurvatures
  *
  * @param[in] ite iterator of the begin surfel on the shape where we compute the integral invariant principal curvatures.
  * @param[in] itb iterator of the end surfel (excluded) on the shape where we compute the integral invariant principal curvatures.
  * @param[out] result iterator of structs with principal curvatures value of Integral Invariant estimator at surfel *it, and eigenVectors
  * and eigenValues resulting of the covariance matrix (see above: struct CurvatureInformations )
  */
  template< typename SurfelIterator, typename OutputIterator >
  void evalPrincipalCurvatures ( const SurfelIterator & itb,
                                 const SurfelIterator & ite,
                                 OutputIterator & result );

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

  std::vector< DigitalSet * > kernelsSet; ///< Array of shifting masks. Size = 27 for each shifting (0-adjacent and full kernel included)

  KernelSupport * kernel; ///< Euclidean kernel

  DigitalShapeKernel * digKernel; ///< Digital kernel

  const KernelSpelFunctor myKernelFunctor; ///< Kernel functor (on Spel)

  Convolver myConvolver; ///< Convolver

  double h; ///< precision of the grid

  double radius; ///< Euclidean radius of the kernel

  ValuesFunctor gaussFunctor; ///< Functor to transform covarianceMatrix to Quantity/PrincipalCurvatures
  PrincipalCurvatureFunctor princCurvFunctor;

private:

  /**
  * Copy constructor.
  * @param other the object to clone.
  * Forbidden by default.
  */
  IntegralInvariantGaussianCurvatureEstimator ( const IntegralInvariantGaussianCurvatureEstimator & other );

  /**
  * Assignment.
  * @param other the object to copy.
  * @return a reference on 'this'.
  * Forbidden by default.
  */
  IntegralInvariantGaussianCurvatureEstimator & operator= ( const IntegralInvariantGaussianCurvatureEstimator & other );

}; // end of specialization for dimension = 3









/**
* Overloads 'operator<<' for displaying objects of class 'IntegralInvariantGaussianCurvatureEstimator'.
* @param out the output stream where the object is written.
* @param object the object of class 'IntegralInvariantGaussianCurvatureEstimator' to write.
* @return the output stream after the writing.
*/
template <typename TKS, typename TSF, Dimension dimension>
std::ostream&
operator<< ( std::ostream & out, const IntegralInvariantGaussianCurvatureEstimator<TKS, TSF, dimension> & object );

template <typename TKS, typename TSF>
std::ostream&
operator<< ( std::ostream & out, const IntegralInvariantGaussianCurvatureEstimator<TKS, TSF, 2> & object );

template <typename TKS, typename TSF>
std::ostream&
operator<< ( std::ostream & out, const IntegralInvariantGaussianCurvatureEstimator<TKS, TSF, 3> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantGaussianCurvatureEstimator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IntegralInvariantGaussianCurvatureEstimator_h

#undef IntegralInvariantGaussianCurvatureEstimator_RECURSES
#endif // else defined(IntegralInvariantGaussianCurvatureEstimator_RECURSES)
