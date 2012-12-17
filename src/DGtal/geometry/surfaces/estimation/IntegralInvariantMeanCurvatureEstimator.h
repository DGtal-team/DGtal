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

#include "DGtal/shapes/parametric/Ball2D.h"
#include "DGtal/shapes/parametric/Ball3D.h"
//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// template class IntegralInvariantMeanCurvatureEstimator
/**
   * Description of template class 'IntegralInvariantMeanCurvatureEstimator' <p>
   * \brief Aim: This class implement a Integral Invariant mean curvature estimation.
   *
   * See related article:
   *       Coeurjolly, D.; Lachaud, J.O; Levallois, J., (2013). Integral based Curvature
   *       Estimators in Digital Geometry. DGCI 2013. Retrieved from
   *       https://liris.cnrs.fr/publis/?id=5866
   *
   * The algorithm we propose uses volume of a kernel (2D: Ball2D, 3D: Ball3D) to approximate mean curvature.
   * To compute the volume, we convolve a kernel around the surface and counting the number of cells belonging the shape.
   * Theorical multigrid convergence is proved, with a convergence speed of O(h^1/3).
   * Experimental results showed a multigrid convergence.
   *
   * Because this algorithm compute value on adjacent cell, we can optimize this step by uses previous
   * results and adding/removing subsets kernel corresponding to the deplacement of the kernel.
   * For example, if __(...)__ is the kernel, and the deplacement is to the right, we want ___[...]_
   * but some same cell from kernel are used. We have to keep same values (in ..) remove not used values (in -)
   * and add new values (in +) :  __(-[..)+]_
   * So we pre-compute each kernel masks (at initialization) to fastly iterate over the shape surface.
   *
   * @tparam TKSpace space in which the shape is defined.
   * @tparam TShapeFunctor TFunctor a model of a functor for the shape ( f(x) ).
   * @tparam dimension dimension of the shape. Let default value to use the correct specialization.
   *
   * @see exampleIntegralInvariantCurvature2D.cpp testIntegralInvariantMeanCurvature3D.cpp testIntegralInvariantCurvature2D.cpp
   */
template <typename TKSpace, typename TShapeFunctor, int dimension = TKSpace::dimension>
class IntegralInvariantMeanCurvatureEstimator
{
public:
  typedef TKSpace KSpace;
  typedef typename Z3i::Domain Domain;
  typedef typename KSpace::Space::RealPoint Point;
  typedef typename Z3i::DigitalSet DigitalSet;
  typedef typename KSpace::SCell Cell;
  typedef typename KSpace::SurfelSet SurfelSet;
  typedef typename SurfelSet::const_iterator ConstIteratorKernel;

  typedef double Quantity;
  typedef int Value;

  typedef TShapeFunctor ShapeCellFunctor;
  typedef ConstValueFunctor<Value> KernelCellFunctor;
  typedef DigitalSurfaceConvolver<ShapeCellFunctor, KernelCellFunctor, KSpace, ConstIteratorKernel> Convolver;

  typedef Ball3D<Z3i::Space> KernelSupport;

  // ----------------------- Standard services ------------------------------
public:
  /**
     * Constructor.
     *
     * @param space space in which the shape is defined.
     * @param f functor on cell of the shape.
     */
  IntegralInvariantMeanCurvatureEstimator ( const KSpace & space, const ShapeCellFunctor & f );

  /**
     * Destructor.
     */
  ~IntegralInvariantMeanCurvatureEstimator()
  {}

  // ----------------------- Interface --------------------------------------
public:

  /**
      * Initialise the IntegralInvariantMeanCurvatureEstimator with a specific kernel k, alpha, and grid step h.
      * Kernel radius = k*h^alpha
      * Diffuse the modification to the KernelFunctor.
      *
      * @param h precision of the grid
      * @param k constant of the kernel support
      * @param useSuggestedSize determine if you want to compute the best radius for the kernel support using your r and h. Default value to true.
      */
  void init ( const double _h, const double k, const double alpha = 1.0/3.0 );

  /**
      * Compute the integral invariant mean curvature to cell *it of a shape.
      *
      * @tparam ConstIteratorOnCells iterator on a Cell
      *
      * @param it iterator of a cell (from a shape) we want compute the integral invariant curvature.
      *
      * @return quantity of the result of Integral Invariant estimator at position *it
      */
  template< typename ConstIteratorOnCells > Quantity eval ( const ConstIteratorOnCells & it );

  /**
      * Compute the integral invariant mean curvature from two cells (from *itb to *ite (exclude) ) of a shape.
      * Return the result on an OutputIterator (param).
      *
      * @tparam ConstIteratorOnCells iterator on a Cell
      * @tparam OutputIterator
      *
      * @param ite iterator of the begin position on the shape where we compute the integral invariant curvature.
      * @param itb iterator of the end position (excluded) on the shape where we compute the integral invariant curvature.
      * @param result iterator of the result of the computation.
      */
  template< typename ConstIteratorOnCells, typename OutputIterator >
  void eval ( const ConstIteratorOnCells & itb,
              const ConstIteratorOnCells & ite,
              OutputIterator & result );

  /**
      * Compute the best radius for the kernel support (k*h^alpha)
      *
      * @param _h precision of the grid
      * @param k constant of the kernel support
      *
      * @return suggested size for the kernel support radius
      */
  static double suggestedSize( const double _h, const double k, const double alpha = 1.0/3.0 );

  /**
      * @return iterator of the begin spel of the kernel support
      */
  const ConstIteratorKernel & beginKernel() const;

  /**
      * @return iterator of the end spel of the kernel support
      */
  const ConstIteratorKernel & endKernel() const;

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
  /// array of shifting masks.
  std::vector< SurfelSet > kernels;
  /// array of begin/end iterator of shifting masks.
  std::vector< KernelIterators< ConstIteratorKernel > > kernelsIterators;

  /// origin spel of the kernel support
  Cell myOrigin;

  /// kernel functor
  const KernelCellFunctor myKernelFunctor;

  /// convolver
  Convolver myConvolver;

  /// precision of the grid
  float h;

  /// radius of the kernel (using k*h^alpha, @see suggestedSize())
  float radius;

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
  typedef typename KSpace::Space::RealPoint Point;
  typedef typename Z2i::DigitalSet DigitalSet;
  typedef typename KSpace::SCell Cell;
  typedef typename KSpace::SurfelSet SurfelSet;
  typedef typename SurfelSet::const_iterator ConstIteratorKernel;

  typedef double Quantity;
  typedef int Value;

  typedef TShapeFunctor ShapeCellFunctor;
  typedef ConstValueFunctor<Value> KernelCellFunctor;
  typedef DigitalSurfaceConvolver<ShapeCellFunctor, KernelCellFunctor, KSpace, ConstIteratorKernel> Convolver;

  typedef Ball2D<Z2i::Space> KernelSupport;

  // ----------------------- Standard services ------------------------------
public:
  /**
     * Constructor.
     *
     * @param space space in which the shape is defined.
     * @param f functor on cell of the shape.
     */
  IntegralInvariantMeanCurvatureEstimator ( const KSpace & space, const ShapeCellFunctor & f );

  /**
     * Destructor.
     */
  ~IntegralInvariantMeanCurvatureEstimator()
  {}

  // ----------------------- Interface --------------------------------------
public:

  /**
      * Initialise the IntegralInvariantMeanCurvatureEstimator with a specific kernel radius, and grid step.
      * Kernel radius = k*h^alpha
      * Diffuse the modification to the KernelFunctor.
      *
      * @param _h precision of the grid
      * @param k constant of the kernel support
      * @param alpha set the alpha to build the better radius for the kernel support (radius=k*h^alpha). Default value is 1/3.
      */
  void init ( const double _h, const double k, const double alpha = 1.0/3.0 );

  /**
      * Compute the integral invariant mean curvature to cell *it of a shape.
      *
      * @tparam ConstIteratorOnCells iterator on a Cell
      *
      * @param it iterator of a cell (from a shape) we want compute the integral invariant curvature.
      *
      * @return quantity of the result of Integral Invariant estimator at position *it
      */
  template< typename ConstIteratorOnCells > Quantity eval ( const ConstIteratorOnCells & it );


  /**
      * Compute the integral invariant mean curvature from two cells (from *itb to *ite (exclude) ) of a shape.
      * Return the result on an OutputIterator (param).
      *
      * @tparam ConstIteratorOnCells iterator on a Cell
      * @tparam OutputIterator
      *
      * @param ite iterator of the begin position on the shape where we compute the integral invariant curvature.
      * @param itb iterator of the end position (excluded) on the shape where we compute the integral invariant curvature.
      * @param result iterator of the result of the computation.
      */
  template< typename ConstIteratorOnCells, typename OutputIterator > void eval ( const ConstIteratorOnCells & itb,
                                                                                 const ConstIteratorOnCells & ite,
                                                                                 OutputIterator & result );

  /**
      * Compute the best radius for the kernel support (k*h^alpha)
      *
      * @param _h precision of the grid
      * @param k constant of the kernel support
      *
      * @return suggested size for the kernel support radius
      */
  static double suggestedSize( const double _h, const double k, const double alpha = 1.0/3.0 );

  /**
      * @return iterator of the begin spel of the kernel support
      */
  const ConstIteratorKernel & beginKernel() const;

  /**
      * @return iterator of the end spel of the kernel support
      */
  const ConstIteratorKernel & endKernel() const;

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

  /// array of shifting masks. Size = 9 for each shiftings (8-adjacence and full kernel included)
  std::vector< SurfelSet > kernels;
  /// array of begin/end iterator of shifting masks.
  std::vector< KernelIterators< ConstIteratorKernel > > kernelsIterators;

  /// origin spel of the kernel support.
  Cell myOrigin;

  /// kernel functor
  const KernelCellFunctor myKernelFunctor;

  /// convolver
  Convolver myConvolver;

  /// precision of the grid
  float h;

  /// radius of the kernel (using k*h^alpha, @see suggestedSize())
  float radius;

  /// kernel's radius-dependant variable. Used to compute IntegralInvariant.
  Quantity dh2; /// h*h
  Quantity d3_r; /// 3/r
  Quantity dPI_2; /// PI/2
  Quantity d1_r2; /// 1/r^2

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
  typedef typename KSpace::Space::RealPoint Point;
  typedef typename Z3i::DigitalSet DigitalSet;
  typedef typename KSpace::SCell Cell;
  typedef typename KSpace::SurfelSet SurfelSet;
  typedef typename SurfelSet::const_iterator ConstIteratorKernel;

  typedef double Quantity;
  typedef int Value;

  typedef TShapeFunctor ShapeCellFunctor;
  typedef ConstValueFunctor<Value> KernelCellFunctor;
  typedef DigitalSurfaceConvolver<ShapeCellFunctor, KernelCellFunctor, KSpace, ConstIteratorKernel> Convolver;

  typedef Ball3D<Z3i::Space> KernelSupport;

  // ----------------------- Standard services ------------------------------
public:
  /**
     * Constructor.
     *
     * @param space space in which the shape is defined.
     * @param f functor on cell of the shape.
     */
  IntegralInvariantMeanCurvatureEstimator ( const KSpace & space, const ShapeCellFunctor & f );

  /**
     * Destructor.
     */
  ~IntegralInvariantMeanCurvatureEstimator()
  {}

  // ----------------------- Interface --------------------------------------
public:

  /**
      * Initialise the IntegralInvariantMeanCurvatureEstimator with a specific kernel radius, and grid step.
      * Kernel radius = k*h^alpha
      * Diffuse the modification to the KernelFunctor.
      *
      * @param _h precision of the grid
      * @param k constant of the kernel support
      * @param alpha set the alpha to build the better radius for the kernel support (radius=k*h^alpha). Default value is 1/3.
      */
  void init ( const double _h, const double k, const double alpha = 1.0/3.0 );

  /**
      * Compute the integral invariant mean curvature to cell *it of a shape.
      *
      * @tparam ConstIteratorOnCells iterator on a Cell
      *
      * @param it iterator of a cell (from a shape) we want compute the integral invariant curvature.
      *
      * @return quantity of the result of Integral Invariant estimator at position *it
      */
  template<typename ConstIteratorOnCells> Quantity eval ( const ConstIteratorOnCells & it );

  /**
      * Compute the integral invariant mean curvature from two cells (from *itb to *ite (exclude) ) of a shape.
      * Return the result on an OutputIterator (param).
      *
      * @tparam ConstIteratorOnCells iterator on a Cell
      * @tparam OutputIterator
      *
      * @param ite iterator of the begin position on the shape where we compute the integral invariant curvature.
      * @param itb iterator of the end position (excluded) on the shape where we compute the integral invariant curvature.
      * @param result iterator of the result of the computation.
      */
  template< typename ConstIteratorOnCells, typename OutputIterator > void eval ( const ConstIteratorOnCells & itb,
                                                                                 const ConstIteratorOnCells & ite,
                                                                                 OutputIterator & result );

  /**
      * Compute the best radius for the kernel support (k*h^alpha)
      *
      * @param _h precision of the grid
      * @param k constant of the kernel support
      *
      * @return suggested size for the kernel support radius
      */
  static double suggestedSize( const double _h, const double k, const double alpha = 1.0/3.0 );

  /**
      * @return iterator of the begin spel of the kernel support
      */
  const ConstIteratorKernel & beginKernel() const;

  /**
      * @return iterator of the end spel of the kernel support
      */
  const ConstIteratorKernel & endKernel() const;

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

  /// array of shifting masks. Size = 27 for each shiftings (26-adjacence and full kernel included)
  std::vector< SurfelSet > kernels;
  /// array of begin/end iterator of shifting masks.
  std::vector< KernelIterators< ConstIteratorKernel > > kernelsIterators;

  /// origin spel of the kernel support.
  Cell myOrigin;

  /// kernel functor
  const KernelCellFunctor myKernelFunctor;

  /// convolver
  Convolver myConvolver;

  /// precision of the grid
  float h;

  /// radius of the kernel (using k*h^alpha, @see suggestedSize())
  float radius;

  /// kernel's radius-dependant variable. Used to compute IntegralInvariant.
  Quantity dh3; /// h*h*h
  Quantity d8_3r; /// 8/3r
  Quantity d_4_PIr4; /// 4/(PI*r^4)

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
template <typename TKS, typename TSF, int dimension>
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
