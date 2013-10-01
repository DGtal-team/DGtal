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
 * @file DigitalSurfaceConvolver_0memory.h
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

#if defined(DigitalSurfaceConvolver_0memory_RECURSES)
#error Recursive header files inclusion detected in DigitalSurfaceConvolver_0memory.h
#else // defined(DigitalSurfaceConvolver_0memory_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalSurfaceConvolver_0memory_RECURSES

#if !defined DigitalSurfaceConvolver_0memory_h
/** Prevents repeated inclusion of headers. */
#define DigitalSurfaceConvolver_0memory_h

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
#include "DGtal/topology/CanonicSCellEmbedder.h"
#include "DGtal/topology/SCellsFunctors.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// template class DigitalSurfaceConvolver_0memory
/**
   * Description of class 'DigitalSurfaceConvolver_0memory' <p>
   *
   * Aim: Compute a convolution between a border on a nD-shape and a convolution kernel : (f*g)(t).
   * An optimization is available when you convolve your shape on adjacent cells using eval(itbegin, itend, output)
   *
   * @tparam TFunctor a model of a functor for the shape to convolve ( f(x) ).
   * @tparam TKernelFunctor a model of a functor for the convolution kernel ( g(x) ).
   * @tparam TKSpace space in which the shape is defined.
   * @tparam TKernelConstIterator iterator of cells inside the convolution kernel.
   */
template< typename TFunctor, typename TKernelFunctor, typename TKSpace, typename TKernelConstIterator, typename TDigitalShapeMasks, Dimension dimension = TKSpace::dimension >
class DigitalSurfaceConvolver_0memory
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

    typedef typename KSpace::SCell Spel;
    typedef typename KSpace::Space::Point Point;
    typedef typename KSpace::Space::RealPoint RealPoint;
    typedef TKernelConstIterator KernelConstIterator;

    typedef std::pair< KernelConstIterator, KernelConstIterator > PairIterators;
    typedef SCellToMidPoint< KSpace > Embedder;

    typedef TDigitalShapeMasks DigitalShapeMasks;

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
    DigitalSurfaceConvolver_0memory ( ConstAlias< Functor > f, ConstAlias< KernelFunctor > g, ConstAlias< KSpace > space );


    /**
       * Destructor.
       */
    ~DigitalSurfaceConvolver_0memory () {}

    // ----------------------- Interface --------------------------------------

public:

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
    void init ( Clone< RealPoint > kOrigin, Alias< std::vector< DigitalShapeMasks* > > mask );

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

protected:

    void computeCovarianceMatrix( const Quantity* aMomentMatrix, CovarianceMatrix & aCovarianceMatrix );
    void fillMoments( Quantity* aMomentMatrix, const Spel & aSpel, double orientation );
    template< typename Shape >
    double computeShiftFromShape( const Shape & shape, const double h, const Spel & aInnerSpel, const Spel & aOutterSpel );

    // ------------------------- Private Datas --------------------------------

private:

    /// Const ref of the shape functor
    const Functor & myFFunctor;

    /// Const ref of the kernel functor
    const KernelFunctor & myGFunctor;

    /// Const ref of the shape Kspace
    const KSpace & myKSpace;

    Embedder embedder;

    /// Copy of vector of iterators for kernel partial masks
    std::vector< DigitalShapeMasks* > myMask;

    /// Copy of the first iterator of the kernel support (Used to iterate over it)
    KernelConstIterator myItKernelBegin;
    /// Copy  of the last iterator of the kernel support (Used to iterate over it)
    KernelConstIterator myItKernelEnd;
    /// Copy of the origin cell of the kernel.
    RealPoint myKernelRealPointOrigin;

    bool isInit;
    bool isInitMask;

    // ------------------------- Hidden services ------------------------------

protected:
    /**
       * Constructor.
       * Forbidden by default (protected to avoid g++ warnings).
       */
    DigitalSurfaceConvolver_0memory ();

private:

    /**
       * Copy constructor.
       * @param other the object to clone.
       * Forbidden by default.
       */
    DigitalSurfaceConvolver_0memory ( const DigitalSurfaceConvolver_0memory & other );

    /**
       * Assignment.
       * @param other the object to copy.
       * @return a reference on 'this'.
       * Forbidden by default.
       */
    DigitalSurfaceConvolver_0memory & operator= ( const DigitalSurfaceConvolver_0memory & other );

    // ------------------------- Internals ------------------------------------

private:

}; // end of class DigitalSurfaceConvolver_0memory

template< typename TFunctor, typename TKernelFunctor, typename TKSpace, typename TKernelConstIterator, typename TDigitalShapeMasks >
class DigitalSurfaceConvolver_0memory< TFunctor, TKernelFunctor, TKSpace, TKernelConstIterator, TDigitalShapeMasks, 2 >
{
    // ----------------------- Types ------------------------------------------

public:

    typedef TFunctor Functor;
    typedef TKSpace KSpace;
    typedef TKernelFunctor KernelFunctor;
    typedef Z2i::Domain Domain;

    typedef double Quantity;
    typedef PointVector< 2, Quantity > VectorQuantity;
    typedef SimpleMatrix< Quantity, 2, 2 > MatrixQuantity;
    typedef SimpleMatrix< double, 2, 2 > CovarianceMatrix;

    typedef typename KSpace::SCell Spel;
    typedef typename KSpace::Point Point;
    typedef typename KSpace::Space::RealPoint RealPoint;
    typedef Z2i::DigitalSet::ConstIterator KernelConstIterator;

    typedef TDigitalShapeMasks DigitalShapeMasks;

    typedef std::pair< KernelConstIterator, KernelConstIterator > PairIterators;
    typedef SCellToMidPoint< KSpace > Embedder;

    //    BOOST_CONCEPT_ASSERT (( CCellFunctor< Functor > ));
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
    DigitalSurfaceConvolver_0memory ( ConstAlias< Functor > f, ConstAlias< KernelFunctor > g, ConstAlias< KSpace > space );


    /**
       * Destructor.
       */
    ~DigitalSurfaceConvolver_0memory () {}

    // ----------------------- Interface --------------------------------------

public:

    /**
       * Initialize the convolver.
       *
       * @param itgbegin iterator of the first cell of the kernel support.
       * @param itgend iterator of the last cell of the kernel support (excluded).
       * @param kOrigin center of the kernel support.
       */
    //void init ( Clone< KernelConstIterator > itgbegin, Clone< KernelConstIterator > itgend, Clone< Spel > kOrigin );

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

    void init ( Clone< Point > pOrigin, DigitalShapeMasks * fullKernel, std::vector< PairIterators > mask );

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
       * Convolve the kernel at a given position.
       *
       * @param it (iterator of a) spel on the surface of the shape where the convolution is computed.
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape .
       *
       * @return the estimated quantity at *it : (f*g)(t)
       */
    template< typename ConstIteratorOnCells, typename Shape >
    Quantity eval ( const ConstIteratorOnCells & it,
                    const Shape & shape,
                    const double h );

    /**
       * Iterate the convolver between [itbegin, itend[.
       *
       * @param itbegin (iterator of the) first spel on the surface of the shape where the convolution is computed.
       * @param itend (iterator of the) last (excluded) spel on the surface of the shape where the convolution is computed.
       * @param result iterator of an array where estimates quantities are set ( the estimated quantity from *itbegin till *itend (excluded)).
       */
    template< typename ConstIteratorOnCells, typename OutputIterator, typename EvalFunctor >
    void eval ( const ConstIteratorOnCells & itbegin,
                const ConstIteratorOnCells & itend,
                OutputIterator & result,
                EvalFunctor functor );

    /**
       * Iterate the convolver between [itbegin, itend[.
       *
       * @param itbegin (iterator of the) first spel on the surface of the shape where the convolution is computed.
       * @param itend (iterator of the) last (excluded) spel on the surface of the shape where the convolution is computed.
       * @param result iterator of an array where estimates quantities are set ( the estimated quantity from *itbegin till *itend (excluded)).
       */
    template< typename ConstIteratorOnCells, typename OutputIterator, typename Shape, typename EvalFunctor >
    void eval ( const ConstIteratorOnCells & itbegin,
                const ConstIteratorOnCells & itend,
                OutputIterator & result,
                const Shape & shape,
                const double h,
                EvalFunctor functor );

    /*template< typename ConstIteratorOnCells >
    void deprecated_eval ( const ConstIteratorOnCells & itbegin,
                           const ConstIteratorOnCells & itend,
                           const std::string & file );*/


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
       * Convolve the kernel at a given position and return a covariance matrix.
       *
       * @param it (iterator of a) spel on the surface of the shape where the covariance matrix is computed.
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape
       *
       * @return the covariance matrix at *it
       */
    template< typename ConstIteratorOnCells, typename Shape >
    CovarianceMatrix evalCovarianceMatrix ( const ConstIteratorOnCells & it,
                                            const Shape & shape,
                                            const double h = 1.0 );


    /**
       * Iterate the convolver between [itbegin, itend[ and return a covariance matrixfor each position.
       *
       * @param itbegin (iterator of the) first spel on the surface of the shape where the covariance matrix is computed.
       * @param itend (iterator of the) last (excluded) spel on the surface of the shape where the covariance matrix is computed.
       * @param result iterator of an array where estimates covariance matrix are set ( the covariance matrix from *itbegin till *itend (excluded)).
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape
       */
    template< typename ConstIteratorOnCells, typename OutputIterator, typename EvalFunctor >
    void evalCovarianceMatrix ( const ConstIteratorOnCells & itbegin,
                                const ConstIteratorOnCells & itend,
                                OutputIterator & result,
                                EvalFunctor functor );

    /**
       * Iterate the convolver between [itbegin, itend[ and return a covariance matrixfor each position.
       *
       * @param itbegin (iterator of the) first spel on the surface of the shape where the covariance matrix is computed.
       * @param itend (iterator of the) last (excluded) spel on the surface of the shape where the covariance matrix is computed.
       * @param result iterator of an array where estimates covariance matrix are set ( the covariance matrix from *itbegin till *itend (excluded)).
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape
       */
    template< typename ConstIteratorOnCells, typename OutputIterator, typename Shape, typename EvalFunctor >
    void evalCovarianceMatrix ( const ConstIteratorOnCells & itbegin,
                                const ConstIteratorOnCells & itend,
                                OutputIterator & result,
                                const Shape & shape,
                                const double h,
                                EvalFunctor functor );

    /*template< typename ConstIteratorOnCells >
    void deprecated_evalCovarianceMatrix ( const ConstIteratorOnCells & itbegin,
                                           const ConstIteratorOnCells & itend,
                                           const std::string & file );*/

    /**
       * Checks the validity/consistency of the object.
       * @return 'true' if the object is valid, 'false' otherwise.
       */
    bool isValid () const;

protected:

    void computeCovarianceMatrix( const Quantity* aMomentMatrix, CovarianceMatrix & aCovarianceMatrix );
    void fillMoments( Quantity* aMomentMatrix, const Spel & aSpel, double orientation );
    template< typename Shape >
    double computeShiftFromShape( const Shape & shape, const double h, const Spel & aInnerSpel, const Spel & aOutterSpel );

    static const int nbMoments;
    static Spel defaultInnerSpel;
    static Spel defaultOuterSpel;
    static Quantity defaultInnerMoments[ 6 ];
    static Quantity defaultOuterMoments[ 6 ];
    static Quantity defaultInnerSum;
    static Quantity defaultOuterSum;

    template< typename SurfelIterator >
    bool core_evalCovarianceMatrix ( const SurfelIterator & it,
                                     CovarianceMatrix & innerMatrix,
                                     CovarianceMatrix & outerMatrix,
                                     bool useLastResults = false,
                                     Spel & lastInnerSpel = defaultInnerSpel,
                                     Spel & lastOuterSpel = defaultOuterSpel,
                                     Quantity * lastInnerMoments = defaultInnerMoments,
                                     Quantity * lastOuterMoments = defaultOuterMoments );

    template< typename SurfelIterator >
    bool core_eval ( const SurfelIterator & it,
                     Quantity & innerSum,
                     Quantity & outerSum,
                     bool useLastResults = false,
                     Spel & lastInnerSpel = defaultInnerSpel,
                     Spel & lastOuterSpel = defaultOuterSpel,
                     Quantity & lastInnerSum = defaultInnerSum,
                     Quantity & lastOuterSum = defaultOuterSum );

    // ------------------------- Private Datas --------------------------------

private:
    const Dimension dimension;

    /// Const ref of the shape functor
    const Functor & myFFunctor;

    /// Const ref of the kernel functor
    const KernelFunctor & myGFunctor;

    /// Const ref of the shape Kspace
    const KSpace & myKSpace;

    Embedder embedder;

    /// Copy of vector of iterators for kernel partial masks
    std::vector< PairIterators > myMask;

    DigitalShapeMasks * kernel;

    /// Copy of the first iterator of the kernel support (Used to iterate over it)
    KernelConstIterator myItKernelBegin;
    /// Copy  of the last iterator of the kernel support (Used to iterate over it)
    KernelConstIterator myItKernelEnd;
    /// Copy of the origin cell of the kernel.
    Spel myKernelSpelOrigin;

    bool isInitMask;

    // ------------------------- Hidden services ------------------------------

protected:
    /**
       * Constructor.
       * Forbidden by default (protected to avoid g++ warnings).
       */
    DigitalSurfaceConvolver_0memory ();

private:

    /**
       * Copy constructor.
       * @param other the object to clone.
       * Forbidden by default.
       */
    DigitalSurfaceConvolver_0memory ( const DigitalSurfaceConvolver_0memory & other );

    /**
       * Assignment.
       * @param other the object to copy.
       * @return a reference on 'this'.
       * Forbidden by default.
       */
    DigitalSurfaceConvolver_0memory & operator= ( const DigitalSurfaceConvolver_0memory & other );

    // ------------------------- Internals ------------------------------------

private:

}; // end of class DigitalSurfaceConvolver_0memory

template< typename TFunctor, typename TKernelFunctor, typename TKSpace, typename TKernelConstIterator, typename TDigitalShapeMasks >
class DigitalSurfaceConvolver_0memory< TFunctor, TKernelFunctor, TKSpace, TKernelConstIterator, TDigitalShapeMasks, 3 >
{
    // ----------------------- Types ------------------------------------------

public:

    typedef TFunctor Functor;
    typedef TKSpace KSpace;
    typedef TKernelFunctor KernelFunctor;
    typedef Z3i::Domain Domain;

    typedef double Quantity;
    typedef PointVector< 3, Quantity > VectorQuantity;
    typedef SimpleMatrix< Quantity, 3, 3 > MatrixQuantity;
    typedef SimpleMatrix< double, 3, 3 > CovarianceMatrix;

    typedef typename KSpace::SCell Spel;
    typedef typename KSpace::Point Point;
    typedef typename KSpace::Space::RealPoint RealPoint;
    typedef Z3i::DigitalSet::ConstIterator KernelConstIterator;

    typedef TDigitalShapeMasks DigitalShapeMasks;

    typedef std::pair< KernelConstIterator, KernelConstIterator > PairIterators;
    typedef SCellToMidPoint< KSpace > Embedder;

    //    BOOST_CONCEPT_ASSERT (( CCellFunctor< Functor > ));
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
    DigitalSurfaceConvolver_0memory ( ConstAlias< Functor > f, ConstAlias< KernelFunctor > g, ConstAlias< KSpace > space );


    /**
       * Destructor.
       */
    ~DigitalSurfaceConvolver_0memory() {}

    // ----------------------- Interface --------------------------------------

public:

    /**
       * Initialize the convolver.
       *
       * @param itgbegin iterator of the first cell of the kernel support.
       * @param itgend iterator of the last cell of the kernel support (excluded).
       * @param kOrigin center of the kernel support.
       */
    //void init ( Clone< KernelConstIterator > itgbegin, Clone< KernelConstIterator > itgend, Clone< Spel > kOrigin );

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

    void init ( Clone< Point > pOrigin, DigitalShapeMasks * fullKernel, std::vector< PairIterators > mask );

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
       * Convolve the kernel at a given position.
       *
       * @param it (iterator of a) spel on the surface of the shape where the convolution is computed.
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape .
       *
       * @return the estimated quantity at *it : (f*g)(t)
       */
    template< typename ConstIteratorOnCells, typename Shape >
    Quantity eval ( const ConstIteratorOnCells & it,
                    const Shape & shape,
                    const double h );

    /**
       * Iterate the convolver between [itbegin, itend[.
       *
       * @param itbegin (iterator of the) first spel on the surface of the shape where the convolution is computed.
       * @param itend (iterator of the) last (excluded) spel on the surface of the shape where the convolution is computed.
       * @param result iterator of an array where estimates quantities are set ( the estimated quantity from *itbegin till *itend (excluded)).
       */
    template< typename ConstIteratorOnCells, typename OutputIterator, typename EvalFunctor  >
    void eval ( const ConstIteratorOnCells & itbegin,
                const ConstIteratorOnCells & itend,
                OutputIterator & result,
                EvalFunctor functor );

    /**
       * Iterate the convolver between [itbegin, itend[.
       *
       * @param itbegin (iterator of the) first spel on the surface of the shape where the convolution is computed.
       * @param itend (iterator of the) last (excluded) spel on the surface of the shape where the convolution is computed.
       * @param result iterator of an array where estimates quantities are set ( the estimated quantity from *itbegin till *itend (excluded)).
       */
    template< typename ConstIteratorOnCells, typename OutputIterator, typename Shape, typename EvalFunctor  >
    void eval ( const ConstIteratorOnCells & itbegin,
                const ConstIteratorOnCells & itend,
                OutputIterator & result,
                const Shape & shape,
                const double h,
                EvalFunctor functor );

    /*template< typename ConstIteratorOnCells >
    void deprecated_eval ( const ConstIteratorOnCells & itbegin,
                           const ConstIteratorOnCells & itend,
                           const std::string & file );*/


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
       * Convolve the kernel at a given position and return a covariance matrix.
       *
       * @param it (iterator of a) spel on the surface of the shape where the covariance matrix is computed.
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape
       *
       * @return the covariance matrix at *it
       */
    template< typename ConstIteratorOnCells, typename Shape >
    CovarianceMatrix evalCovarianceMatrix ( const ConstIteratorOnCells & it,
                                            const Shape & shape,
                                            const double h );

    /**
       * Iterate the convolver between [itbegin, itend[ and return a covariance matrixfor each position.
       *
       * @param itbegin (iterator of the) first spel on the surface of the shape where the covariance matrix is computed.
       * @param itend (iterator of the) last (excluded) spel on the surface of the shape where the covariance matrix is computed.
       * @param result iterator of an array where estimates covariance matrix are set ( the covariance matrix from *itbegin till *itend (excluded)).
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape
       */
    template< typename ConstIteratorOnCells, typename OutputIterator, typename EvalFunctor >
    void evalCovarianceMatrix ( const ConstIteratorOnCells & itbegin,
                                const ConstIteratorOnCells & itend,
                                OutputIterator & result,
                                EvalFunctor functor );

    /**
       * Iterate the convolver between [itbegin, itend[ and return a covariance matrixfor each position.
       *
       * @param itbegin (iterator of the) first spel on the surface of the shape where the covariance matrix is computed.
       * @param itend (iterator of the) last (excluded) spel on the surface of the shape where the covariance matrix is computed.
       * @param result iterator of an array where estimates covariance matrix are set ( the covariance matrix from *itbegin till *itend (excluded)).
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape
       */
    template< typename ConstIteratorOnCells, typename OutputIterator, typename EvalFunctor >
    void evalPrincipalCurvatures ( const ConstIteratorOnCells & itbegin,
                                   const ConstIteratorOnCells & itend,
                                   OutputIterator & result,
                                   EvalFunctor functor );

    /**
       * Iterate the convolver between [itbegin, itend[ and return a covariance matrixfor each position.
       *
       * @param itbegin (iterator of the) first spel on the surface of the shape where the covariance matrix is computed.
       * @param itend (iterator of the) last (excluded) spel on the surface of the shape where the covariance matrix is computed.
       * @param result iterator of an array where estimates covariance matrix are set ( the covariance matrix from *itbegin till *itend (excluded)).
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape
       */
    template< typename ConstIteratorOnCells, typename OutputIterator, typename Shape, typename EvalFunctor >
    void evalCovarianceMatrix ( const ConstIteratorOnCells & itbegin,
                                const ConstIteratorOnCells & itend,
                                OutputIterator & result,
                                const Shape & shape,
                                const double h,
                                EvalFunctor functor );

    /**
       * Iterate the convolver between [itbegin, itend[ and return a covariance matrixfor each position.
       * @warning Deprecated. use evalCovarianceMatrix() instead
       * @param itbegin (iterator of the) first spel on the surface of the shape where the covariance matrix is computed.
       * @param itend (iterator of the) last (excluded) spel on the surface of the shape where the covariance matrix is computed.
       * @param result iterator of an array where estimates covariance matrix are set ( the covariance matrix from *itbegin till *itend (excluded)).
       *
       * @tparam ConstIteratorOnCells iterator of a spel of the shape
       */
    /*template< typename ConstIteratorOnCells >
    void deprecated_evalCovarianceMatrix ( const ConstIteratorOnCells & itbegin,
                                           const ConstIteratorOnCells & itend,
                                           const std::string & file );*/

    /**
       * Checks the validity/consistency of the object.
       * @return 'true' if the object is valid, 'false' otherwise.
       */
    bool isValid() const;

protected:

    void computeCovarianceMatrix ( const Quantity* aMomentMatrix, CovarianceMatrix & aCovarianceMatrix );
    void fillMoments ( Quantity* aMomentMatrix, const Spel & aSpel, double orientation );
    template< typename Shape >
    double computeShiftFromShape ( const Shape & shape, const double h, const Spel & aInnerSpel, const Spel & aOutterSpel );

    static const int nbMoments;
    static Spel defaultInnerSpel;
    static Spel defaultOuterSpel;
    static Quantity defaultInnerMoments[ 10 ];
    static Quantity defaultOuterMoments[ 10 ];
    static Quantity defaultInnerSum;
    static Quantity defaultOuterSum;

    template< typename SurfelIterator >
    bool core_evalCovarianceMatrix ( const SurfelIterator & it,
                                     CovarianceMatrix & innerMatrix,
                                     CovarianceMatrix & outerMatrix,
                                     bool useLastResults = false,
                                     Spel & lastInnerSpel = defaultInnerSpel,
                                     Spel & lastOuterSpel = defaultOuterSpel,
                                     Quantity * lastInnerMoments = defaultInnerMoments,
                                     Quantity * lastOuterMoments = defaultOuterMoments );

    template< typename SurfelIterator >
    bool core_eval ( const SurfelIterator & it,
                     Quantity & innerSum,
                     Quantity & outerSum,
                     bool useLastResults = false,
                     Spel & lastInnerSpel = defaultInnerSpel,
                     Spel & lastOuterSpel = defaultOuterSpel,
                     Quantity & lastInnerSum = defaultInnerSum,
                     Quantity & lastOuterSum = defaultOuterSum );



    // ------------------------- Private Datas --------------------------------

private:
    const Dimension dimension;

    /// Const ref of the shape functor
    const Functor & myFFunctor;

    /// Const ref of the kernel functor
    const KernelFunctor & myGFunctor;

    /// Const ref of the shape Kspace
    const KSpace & myKSpace;

    Embedder embedder;

    /// Copy of vector of iterators for kernel partial masks
    std::vector< PairIterators > myMask;

    DigitalShapeMasks * kernel;

    /// Copy of the first iterator of the kernel support (Used to iterate over it)
    KernelConstIterator myItKernelBegin;
    /// Copy  of the last iterator of the kernel support (Used to iterate over it)
    KernelConstIterator myItKernelEnd;
    /// Copy of the origin cell of the kernel.
    Spel myKernelSpelOrigin;

    bool isInitMask;

    // ------------------------- Hidden services ------------------------------

protected:
    /**
       * Constructor.
       * Forbidden by default (protected to avoid g++ warnings).
       */
    DigitalSurfaceConvolver_0memory ();

private:

    /**
       * Copy constructor.
       * @param other the object to clone.
       * Forbidden by default.
       */
    DigitalSurfaceConvolver_0memory ( const DigitalSurfaceConvolver_0memory & other );

    /**
       * Assignment.
       * @param other the object to copy.
       * @return a reference on 'this'.
       * Forbidden by default.
       */
    DigitalSurfaceConvolver_0memory & operator= ( const DigitalSurfaceConvolver_0memory & other );

    // ------------------------- Internals ------------------------------------

private:

}; // end of class DigitalSurfaceConvolver_0memory



/**
   * Overloads 'operator<<' for displaying objects of class 'DigitalSurfaceConvolver_0memory'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DigitalSurfaceConvolver_0memory' to write.
   * @return the output stream after the writing.
   */
template< typename TF,  typename TKF, typename TKS, typename TKCI, typename TEM, Dimension dimension >
std::ostream&
operator<< ( std::ostream & out, const DGtal::DigitalSurfaceConvolver_0memory< TF, TKF, TKS, TKCI, TEM, dimension > & object );

template< typename TF,  typename TKF, typename TKS, typename TKCI, typename TEM >
std::ostream&
operator<< ( std::ostream & out, const DGtal::DigitalSurfaceConvolver_0memory< TF, TKF, TKS, TKCI, TEM, 2 > & object );

template< typename TF,  typename TKF, typename TKS, typename TKCI, typename TEM >
std::ostream&
operator<< ( std::ostream & out, const DGtal::DigitalSurfaceConvolver_0memory<TF, TKF, TKS, TKCI, TEM, 3 > & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/DigitalSurfaceConvolver_0memory.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DigitalSurfaceConvolver_0memory_h

#undef DigitalSurfaceConvolver_0memory_RECURSES
#endif // else defined(DigitalSurfaceConvolver_0memory_RECURSES)
