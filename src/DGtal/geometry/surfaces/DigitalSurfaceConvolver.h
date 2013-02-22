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
 * @file DigitalSurfaceConvolver.h
 * @brief Compute a convolution between a border on a nD-shape and a convolution kernel : (f*g)(t).
 * An optimization is available when you convolve your shape on adjacent cells using eval(itbegin, itend, output)
 *
 * @author Jeremy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2012/03/27
 *
 * This file is part of the DGtal library.
 *
 * @see IntegralInvariantMeanCurvatureEstimator.h IntegralInvariantGaussianCurvatureEstimator.h
 */

#if defined(DigitalSurfaceConvolver_RECURSES)
#error Recursive header files inclusion detected in DigitalSurfaceConvolver.h
#else // defined(DigitalSurfaceConvolver_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalSurfaceConvolver_RECURSES

#if !defined DigitalSurfaceConvolver_h
/** Prevents repeated inclusion of headers. */
#define DigitalSurfaceConvolver_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/SimpleMatrix.h"
#include "DGtal/base/ConstAlias.h"
#include "DGtal/base/Alias.h"
#include "DGtal/base/Clone.h"
#include "DGtal/kernel/CCellFunctor.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// template class DigitalSurfaceConvolver
/**
   * Description of class 'DigitalSurfaceConvolver' <p>
   *
   * Aim: Compute a convolution between a border on a nD-shape and a convolution kernel : (f*g)(t).
   * An optimization is available when you convolve your shape on adjacent cells using eval(itbegin, itend, output)
   *
   * @tparam TFunctor a model of a functor for the shape to convolve ( f(x) ).
   * @tparam TKernelFunctor a model of a functor for the convolution kernel ( g(x) ).
   * @tparam TKSpace space in which the shape is defined.
   * @tparam TKernelConstIterator iterator of cells inside the convolution kernel.
   */
template< typename TFunctor, typename TKernelFunctor, typename TKSpace, typename TKernelConstIterator, Dimension dimension = TKSpace::dimension >
class DigitalSurfaceConvolver
{
  // ----------------------- Types ------------------------------------------

public:

  typedef TFunctor Functor;
  typedef TKSpace KSpace;
  typedef TKernelFunctor KernelFunctor;

  typedef double Quantity;
  typedef PointVector< dimension, Quantity > VectorQuantity;
  typedef SimpleMatrix< Quantity, dimension, dimension > MatrixQuantity;
  typedef SimpleMatrix< double, dimension, dimension > CovarianceMatrix;

  typedef typename KSpace::SCell Cell;
  typedef typename KSpace::Space::Point Point;
  typedef TKernelConstIterator KernelConstIterator;

  typedef std::pair< KernelConstIterator, KernelConstIterator > PairIterators;

  BOOST_CONCEPT_ASSERT (( CCellFunctor< Functor > ));
//  BOOST_CONCEPT_ASSERT (( CCellFunctor< KernelFunctor > ));

  // ----------------------- Standard services ------------------------------

public:

  /**
       * Constructor.
       * 
       * @param f a functor f(x).
       * @param g a functor g(x).
       * @param space space in which the shape is defined.
       */
  DigitalSurfaceConvolver ( ConstAlias< Functor > f, ConstAlias< KernelFunctor > g, ConstAlias< KSpace > space );


  /**
       * Destructor.
       */
  ~DigitalSurfaceConvolver () {}

  // ----------------------- Interface --------------------------------------

public:

  /**
       * Initialize the convolver.
       *
       * @param itgbegin iterator of the first cell of the kernel support.
       * @param itgend iterator of the last cell of the kernel support (excluded).
       * @param kOrigin center of the kernel support.
       */
  void init ( Clone< KernelConstIterator > itgbegin, Clone< KernelConstIterator > itgend, Clone< Cell > kOrigin );

  /**
       * Intitialize the convolver using masks - allow to use the optimization with adjacent cells.
       *
       * @param itgbegin iterator of the first spel of the kernel support.
       * @param itgend iterator of the last spel of the kernel support (excluded).
       * @param kOrigin center of the kernel support.
       * @param mask Vector of PairIterators. They must be ordered using a trit ({0,1,2}) encoded array.
       * trit 0 => shifting_coord = -1
       * trit 1 => shifting_coord = 0
       * trit 2 => shifting_coord = 1
       * Example in 3D :      zyx                 x  y  z
       * mask[0] : base3(0) = 000 => shifting = {-1,-1,-1}
       * mask[5] : base3(5) = 012 => shifting = { 1, 0,-1}
       */
  void init ( Clone< KernelConstIterator > itgbegin, Clone< KernelConstIterator > itgend, Clone< Cell > kOrigin, Alias< std::vector< PairIterators > > mask );

  /**
       * Convolve the kernel at a given position.
       *
       * @param it (iterator of a) spel on the surface of the shape where the convolution is computed.
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape .
       *
       * @return the estimated quantity at *it : (f*g)(t)
       */
  template< typename ConstIteratorOnCells > Quantity eval ( const ConstIteratorOnCells & it );

  /**
       * Iterate the convolver between [itbegin, itend[.
       *
       * @param itbegin (iterator of the) first spel on the surface of the shape where the convolution is computed.
       * @param itend (iterator of the) last (excluded) spel on the surface of the shape where the convolution is computed.
       * @param result iterator of an array where estimates quantities are set ( the estimated quantity from *itbegin till *itend (excluded)).
       */
  template< typename ConstIteratorOnCells, typename OutputIterator >
  void eval ( const ConstIteratorOnCells & itbegin,
              const ConstIteratorOnCells & itend,
              OutputIterator & result );

  /**
       * Convolve the kernel at a given position and return a covariance matrix.
       *
       * @param it (iterator of a) spel on the surface of the shape where the covariance matrix is computed.
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape
       *
       * @return the covariance matrix at *it
       */
  template< typename ConstIteratorOnCells >
  CovarianceMatrix evalCovarianceMatrix ( const ConstIteratorOnCells & it );


  /**
       * Iterate the convolver between [itbegin, itend[ and return a covariance matrixfor each position.
       *
       * @param itbegin (iterator of the) first spel on the surface of the shape where the covariance matrix is computed.
       * @param itend (iterator of the) last (excluded) spel on the surface of the shape where the covariance matrix is computed.
       * @param result iterator of an array where estimates covariance matrix are set ( the covariance matrix from *itbegin till *itend (excluded)).
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape
       */
  template< typename ConstIteratorOnCells, typename OutputIterator >
  void evalCovarianceMatrix ( const ConstIteratorOnCells & itbegin,
                              const ConstIteratorOnCells & itend,
                              OutputIterator & result );

  /**
       * Checks the validity/consistency of the object.
       * @return 'true' if the object is valid, 'false' otherwise.
       */
  bool isValid () const;

  // ------------------------- Private Datas --------------------------------

private:

  /// Const ref of the shape functor
  const Functor & myFFunctor;

  /// Const ref of the kernel functor
  const KernelFunctor & myGFunctor;

  /// Const ref of the shape Kspace
  const KSpace & myKSpace;

  /// Copy of vector of iterators for kernel partial masks
  std::vector< PairIterators > myMask;

  /// Copy of the first iterator of the kernel support (Used to iterate over it)
  KernelConstIterator myItKernelBegin;
  /// Copy  of the last iterator of the kernel support (Used to iterate over it)
  KernelConstIterator myItKernelEnd;
  /// Copy of the origin cell of the kernel.
  Cell myKernelCellOrigin;

  bool isInit;
  bool isInitMask;

  // ------------------------- Hidden services ------------------------------

protected:
  /**
       * Constructor.
       * Forbidden by default (protected to avoid g++ warnings).
       */
  DigitalSurfaceConvolver ();

private:

  /**
       * Copy constructor.
       * @param other the object to clone.
       * Forbidden by default.
       */
  DigitalSurfaceConvolver ( const DigitalSurfaceConvolver & other );

  /**
       * Assignment.
       * @param other the object to copy.
       * @return a reference on 'this'.
       * Forbidden by default.
       */
  DigitalSurfaceConvolver & operator= ( const DigitalSurfaceConvolver & other );

  // ------------------------- Internals ------------------------------------

private:

}; // end of class DigitalSurfaceConvolver

template< typename TFunctor, typename TKernelFunctor, typename TKSpace, typename TKernelConstIterator >
class DigitalSurfaceConvolver< TFunctor, TKernelFunctor, TKSpace, TKernelConstIterator, 2 >
{
  // ----------------------- Types ------------------------------------------

public:

  typedef TFunctor Functor;
  typedef TKSpace KSpace;
  typedef TKernelFunctor KernelFunctor;

  typedef double Quantity;
  typedef PointVector< 2, Quantity > VectorQuantity;
  typedef SimpleMatrix< Quantity, 2, 2 > MatrixQuantity;
  typedef SimpleMatrix< double, 2, 2 > CovarianceMatrix;

  typedef typename KSpace::SCell Cell;
  typedef typename KSpace::Space::Point Point;
  typedef TKernelConstIterator KernelConstIterator;

  typedef std::pair< KernelConstIterator, KernelConstIterator > PairIterators;

  BOOST_CONCEPT_ASSERT (( CCellFunctor< Functor > ));
//  BOOST_CONCEPT_ASSERT (( CCellFunctor< KernelFunctor > ));

  // ----------------------- Standard services ------------------------------

public:

  /**
       * Constructor.
       *
       * @param f a functor f(x).
       * @param g a functor g(x).
       * @param space space in which the shape is defined.
       */
  DigitalSurfaceConvolver ( ConstAlias< Functor > f, ConstAlias< KernelFunctor > g, ConstAlias< KSpace > space );


  /**
       * Destructor.
       */
  ~DigitalSurfaceConvolver () {}

  // ----------------------- Interface --------------------------------------

public:

  /**
       * Initialize the convolver.
       *
       * @param itgbegin iterator of the first cell of the kernel support.
       * @param itgend iterator of the last cell of the kernel support (excluded).
       * @param kOrigin center of the kernel support.
       */
  void init ( Clone< KernelConstIterator > itgbegin, Clone< KernelConstIterator > itgend, Clone< Cell > kOrigin );

  /**
       * Intitialize the convolver using masks - allow to use the optimization with adjacent cells.
       *
       * @param itgbegin iterator of the first spel of the kernel support.
       * @param itgend iterator of the last spel of the kernel support (excluded).
       * @param kOrigin center of the kernel support.
       * @param mask Vector of iterators. They must be ordered using a trit ({0,1,2}) encoded array.
       * trit 0 => shifting_coord = -1
       * trit 1 => shifting_coord = 0
       * trit 2 => shifting_coord = 1
       * Example in 3D :      zyx                 x  y  z
       * mask[0] : base3(0) = 000 => shifting = {-1,-1,-1}
       * mask[5] : base3(5) = 012 => shifting = { 1, 0,-1}
       */
  void init ( Clone< KernelConstIterator > itgbegin, Clone< KernelConstIterator > itgend, Clone< Cell > kOrigin, Alias< std::vector< PairIterators > > mask );

  /**
       * Convolve the kernel at a given position.
       *
       * @param it (iterator of a) spel on the surface of the shape where the convolution is computed.
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape .
       *
       * @return the estimated quantity at *it : (f*g)(t)
       */
  template< typename ConstIteratorOnCells > Quantity eval ( const ConstIteratorOnCells & it );


  /**
       * Iterate the convolver between [itbegin, itend[.
       *
       * @param itbegin (iterator of the) first spel on the surface of the shape where the convolution is computed.
       * @param itend (iterator of the) last (excluded) spel on the surface of the shape where the convolution is computed.
       * @param result iterator of an array where estimates quantities are set ( the estimated quantity from *itbegin till *itend (excluded)).
       */
  template< typename ConstIteratorOnCells, typename OutputIterator >
  void eval ( const ConstIteratorOnCells & itbegin,
              const ConstIteratorOnCells & itend,
              OutputIterator & result );


  /**
       * Convolve the kernel at a given position and return a covariance matrix.
       *
       * @param it (iterator of a) spel on the surface of the shape where the covariance matrix is computed.
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape
       *
       * @return the covariance matrix at *it
       */
  template< typename ConstIteratorOnCells >
  CovarianceMatrix evalCovarianceMatrix ( const ConstIteratorOnCells & it );


  /**
       * Iterate the convolver between [itbegin, itend[ and return a covariance matrixfor each position.
       *
       * @param itbegin (iterator of the) first spel on the surface of the shape where the covariance matrix is computed.
       * @param itend (iterator of the) last (excluded) spel on the surface of the shape where the covariance matrix is computed.
       * @param result iterator of an array where estimates covariance matrix are set ( the covariance matrix from *itbegin till *itend (excluded)).
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape
       */
  template< typename ConstIteratorOnCells, typename OutputIterator >
  void evalCovarianceMatrix ( const ConstIteratorOnCells & itbegin,
                              const ConstIteratorOnCells & itend,
                              OutputIterator & result );

  /**
       * Checks the validity/consistency of the object.
       * @return 'true' if the object is valid, 'false' otherwise.
       */
  bool isValid () const;

  // ------------------------- Private Datas --------------------------------

private:

  /// Const ref of the shape functor
  const Functor & myFFunctor;

  /// Const ref of the kernel functor
  const KernelFunctor & myGFunctor;

  /// Const ref of the shape Kspace
  const KSpace & myKSpace;

  /// Copy of vector of iterators for kernel partial masks
  std::vector< PairIterators > myMask;

  /// Copy of the first iterator of the kernel support (Used to iterate over it)
  KernelConstIterator myItKernelBegin;
  /// Copy  of the last iterator of the kernel support (Used to iterate over it)
  KernelConstIterator myItKernelEnd;
  /// Copy of the origin cell of the kernel.
  Cell myKernelCellOrigin;

  bool isInit;
  bool isInitMask;

  // ------------------------- Hidden services ------------------------------

protected:
  /**
       * Constructor.
       * Forbidden by default (protected to avoid g++ warnings).
       */
  DigitalSurfaceConvolver ();

private:

  /**
       * Copy constructor.
       * @param other the object to clone.
       * Forbidden by default.
       */
  DigitalSurfaceConvolver ( const DigitalSurfaceConvolver & other );

  /**
       * Assignment.
       * @param other the object to copy.
       * @return a reference on 'this'.
       * Forbidden by default.
       */
  DigitalSurfaceConvolver & operator= ( const DigitalSurfaceConvolver & other );

  // ------------------------- Internals ------------------------------------

private:

}; // end of class DigitalSurfaceConvolver

template< typename TFunctor, typename TKernelFunctor, typename TKSpace, typename TKernelConstIterator >
class DigitalSurfaceConvolver< TFunctor, TKernelFunctor, TKSpace, TKernelConstIterator, 3 >
{
  // ----------------------- Types ------------------------------------------

public:

  typedef TFunctor Functor;
  typedef TKSpace KSpace;
  typedef TKernelFunctor KernelFunctor;

  typedef double Quantity;
  typedef PointVector< 3, Quantity > VectorQuantity;
  typedef SimpleMatrix< Quantity, 3, 3 > MatrixQuantity;
  typedef SimpleMatrix< double, 3, 3 > CovarianceMatrix;

  typedef typename KSpace::SCell Cell;
  typedef typename KSpace::Space::Point Point;
  typedef TKernelConstIterator KernelConstIterator;

  typedef std::pair< KernelConstIterator, KernelConstIterator > PairIterators;

  BOOST_CONCEPT_ASSERT (( CCellFunctor< Functor > ));
//  BOOST_CONCEPT_ASSERT (( CCellFunctor< KernelFunctor > ));

  // ----------------------- Standard services ------------------------------

public:

  /**
       * Constructor.
       *
       * @param f a functor f(x).
       * @param g a functor g(x).
       * @param space space in which the shape is defined.
       */
  DigitalSurfaceConvolver ( ConstAlias< Functor > f, ConstAlias< KernelFunctor > g, ConstAlias< KSpace > space );


  /**
       * Destructor.
       */
  ~DigitalSurfaceConvolver() {}

  // ----------------------- Interface --------------------------------------

public:

  /**
       * Initialize the convolver.
       *
       * @param itgbegin iterator of the first cell of the kernel support.
       * @param itgend iterator of the last cell of the kernel support (excluded).
       * @param kOrigin center of the kernel support.
       */
  void init ( Clone< KernelConstIterator > itgbegin, Clone< KernelConstIterator > itgend, Clone< Cell > kOrigin );

  /**
       * Intitialize the convolver using masks - allow to use the optimization with adjacent cells.
       *
       * @param itgbegin iterator of the first spel of the kernel support.
       * @param itgend iterator of the last spel of the kernel support (excluded).
       * @param kOrigin center of the kernel support.
       * @param mask Vector of iterators. They must be ordered using a trit ({0,1,2}) encoded array.
       * trit 0 => shifting_coord = -1
       * trit 1 => shifting_coord = 0
       * trit 2 => shifting_coord = 1
       * Example in 3D :      zyx                 x  y  z
       * mask[0] : base3(0) = 000 => shifting = {-1,-1,-1}
       * mask[5] : base3(5) = 012 => shifting = { 1, 0,-1}
       */
  void init ( Clone< KernelConstIterator > itgbegin, Clone< KernelConstIterator > itgend, Clone< Cell > kOrigin, Alias< std::vector< PairIterators > > mask );

  /**
       * Convolve the kernel at a given position.
       *
       * @param it (iterator of a) spel on the surface of the shape where the convolution is computed.
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape .
       *
       * @return the estimated quantity at *it : (f*g)(t)
       */
  template< typename ConstIteratorOnCells >
  Quantity eval ( const ConstIteratorOnCells & it );

  /**
       * Iterate the convolver between [itbegin, itend[.
       *
       * @param itbegin (iterator of the) first spel on the surface of the shape where the convolution is computed.
       * @param itend (iterator of the) last (excluded) spel on the surface of the shape where the convolution is computed.
       * @param result iterator of an array where estimates quantities are set ( the estimated quantity from *itbegin till *itend (excluded)).
       */
  template< typename ConstIteratorOnCells, typename OutputIterator >
  void eval ( const ConstIteratorOnCells & itbegin,
              const ConstIteratorOnCells & itend,
              OutputIterator & result );

  /**
       * Convolve the kernel at a given position and return a covariance matrix.
       *
       * @param it (iterator of a) spel on the surface of the shape where the covariance matrix is computed.
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape
       *
       * @return the covariance matrix at *it
       */
  template< typename ConstIteratorOnCells >
  CovarianceMatrix evalCovarianceMatrix ( const ConstIteratorOnCells & it );

  /**
       * Iterate the convolver between [itbegin, itend[ and return a covariance matrixfor each position.
       *
       * @param itbegin (iterator of the) first spel on the surface of the shape where the covariance matrix is computed.
       * @param itend (iterator of the) last (excluded) spel on the surface of the shape where the covariance matrix is computed.
       * @param result iterator of an array where estimates covariance matrix are set ( the covariance matrix from *itbegin till *itend (excluded)).
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape
       */
  template< typename ConstIteratorOnCells, typename OutputIterator >
  void evalCovarianceMatrix ( const ConstIteratorOnCells & itbegin,
                              const ConstIteratorOnCells & itend,
                              OutputIterator & result );

  /**
       * Checks the validity/consistency of the object.
       * @return 'true' if the object is valid, 'false' otherwise.
       */
  bool isValid() const;

  // ------------------------- Private Datas --------------------------------

private:

  /// Const ref of the shape functor
  const Functor & myFFunctor;

  /// Const ref of the kernel functor
  const KernelFunctor & myGFunctor;

  /// Const ref of the shape Kspace
  const KSpace & myKSpace;

  /// Copy of vector of iterators for kernel partial masks
  std::vector< PairIterators > myMask;

  /// Copy of the first iterator of the kernel support (Used to iterate over it)
  KernelConstIterator myItKernelBegin;
  /// Copy  of the last iterator of the kernel support (Used to iterate over it)
  KernelConstIterator myItKernelEnd;
  /// Copy of the origin cell of the kernel.
  Cell myKernelCellOrigin;

  bool isInit;
  bool isInitMask;

  // ------------------------- Hidden services ------------------------------

protected:
  /**
       * Constructor.
       * Forbidden by default (protected to avoid g++ warnings).
       */
  DigitalSurfaceConvolver ();

private:

  /**
       * Copy constructor.
       * @param other the object to clone.
       * Forbidden by default.
       */
  DigitalSurfaceConvolver ( const DigitalSurfaceConvolver & other );

  /**
       * Assignment.
       * @param other the object to copy.
       * @return a reference on 'this'.
       * Forbidden by default.
       */
  DigitalSurfaceConvolver & operator= ( const DigitalSurfaceConvolver & other );

  // ------------------------- Internals ------------------------------------

private:

}; // end of class DigitalSurfaceConvolver



/**
   * Overloads 'operator<<' for displaying objects of class 'DigitalSurfaceConvolver'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DigitalSurfaceConvolver' to write.
   * @return the output stream after the writing.
   */
template< typename TF,  typename TKF, typename TKS, typename TKCI, Dimension dimension >
std::ostream&
operator<< ( std::ostream & out, const DGtal::DigitalSurfaceConvolver< TF, TKF, TKS, TKCI, dimension > & object );

template< typename TF,  typename TKF, typename TKS, typename TKCI >
std::ostream&
operator<< ( std::ostream & out, const DGtal::DigitalSurfaceConvolver< TF, TKF, TKS, TKCI, 2 > & object );

template< typename TF,  typename TKF, typename TKS, typename TKCI >
std::ostream&
operator<< ( std::ostream & out, const DGtal::DigitalSurfaceConvolver<TF, TKF, TKS, TKCI, 3 > & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/DigitalSurfaceConvolver.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DigitalSurfaceConvolver_h

#undef DigitalSurfaceConvolver_RECURSES
#endif // else defined(DigitalSurfaceConvolver_RECURSES)
