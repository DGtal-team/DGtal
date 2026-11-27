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
 * @file LambdaMaximalSegmentEstimator.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2026/11/26
 *
 * Header file for module LambdaMaximalSegmentEstimator.ih
 *
 * This file is part of the DGtal library.
 */

#if defined(LambdaMaximalSegmentEstimator_RECURSES)
#error Recursive header files inclusion detected in LambdaMaximalSegmentEstimator.h
#else // defined(LambdaMaximalSegmentEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define LambdaMaximalSegmentEstimator_RECURSES

#if !defined LambdaMaximalSegmentEstimator_h
/** Prevents repeated inclusion of headers. */
#define LambdaMaximalSegmentEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/base/IteratorCirculatorTraits.h"
#include "DGtal/base/IteratorFunctions.h"
#include "DGtal/geometry/curves/CForwardSegmentComputer.h"
#include "DGtal/geometry/curves/SegmentComputerHelpers.h"
#include "DGtal/geometry/curves/estimation/CSegmentComputerEstimator.h"
#include "DGtal/geometry/curves/estimation/CCurveLocalGeometricEstimator.h"
#include "DGtal/geometry/curves/estimation/FunctorsLambdaMST.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  namespace functors
  {
    /**
     * Eccentricity of the elements of a segment, when the elements are
     * points (e.g. the points of a 4-connected digital curve): the element
     * of index @a idx in a segment of @a n elements has eccentricity
     * \f$ idx / (n-1) \f$, i.e. its relative position between the two
     * extremities of the segment, as in \cite LachaudIVC2007.
     * The eccentricity of the two extremities of a segment is thus
     * 0 and 1.
     */
    struct PointEccentricity
    {
      /**
       * @param idx index of an element in a segment
       * @param n number of elements of the segment
       * @return eccentricity of the element, in [0,1]
       * (1/2 for a segment reduced to one element)
       */
      double operator() (std::size_t idx, std::size_t n) const
      {
        return ( n > 1 ) ? double( idx ) / double( n - 1 ) : 0.5;
      }
    };

    /**
     * Eccentricity of the elements of a segment, when the elements are
     * steps (e.g. the unit steps, or the surfels, of a 4-connected digital
     * curve): the element of index @a idx in a segment of @a n elements
     * has eccentricity \f$ (idx + 1/2) / n \f$, i.e. the relative position
     * of its middle between the two extremities of the segment.
     * The eccentricity of any element is thus strictly between 0 and 1.
     */
    struct StepEccentricity
    {
      /**
       * @param idx index of an element in a segment
       * @param n number of elements of the segment (at least 1)
       * @return eccentricity of the element, in ]0,1[
       */
      double operator() (std::size_t idx, std::size_t n) const
      {
        return ( double( idx ) + 0.5 ) / double( n );
      }
    };
  } // namespace functors

  /////////////////////////////////////////////////////////////////////////////
  // template class LambdaMaximalSegmentEstimator
  /**
   * Description of template class 'LambdaMaximalSegmentEstimator' <p>
   * \brief Aim: A model of concepts::CCurveLocalGeometricEstimator that assigns to
   * each element of a (sub)range a quantity estimated from all the maximal
   * segments passing through this element (the pencil of maximal segments),
   * weighted according to the position of the element in each of them
   * (\f$\lambda\f$-maximal segment estimator, \f$\lambda\f$-MST
   * \cite LachaudIVC2007).
   *
   * Let \f$ M_i \f$ be the maximal segments passing through an element
   * \f$ k \f$, \f$ q_i(k) \f$ the quantity estimated at \f$ k \f$ from
   * \f$ M_i \f$ by the segment computer estimator, and \f$ e_i(k) \f$ the
   * eccentricity of \f$ k \f$ in \f$ M_i \f$. The estimated quantity at
   * \f$ k \f$ is
   * \f[ \frac{\sum_i \lambda(e_i(k)) q_i(k)}{\sum_i \lambda(e_i(k))}, \f]
   * where \f$ \lambda \f$ is a mapping from \f$ [0,1] \f$ to
   * \f$ \mathbb{R}^+ \f$ with \f$ \lambda(0) = \lambda(1) = 0 \f$ and
   * \f$ \lambda > 0 \f$ elsewhere. If the sum of the weights is null
   * (e.g. at the extremities of an open curve, with the eccentricity
   * PointEccentricity), the estimated quantity is the (unweighted)
   * mean of the quantities \f$ q_i(k) \f$.
   *
   * The eccentricity of an element in a segment only depends on its index
   * and on the number of elements of the segment. It is given by the
   * functor @a TEccentricity: functors::PointEccentricity (default) if the
   * elements are points, as in \cite LachaudIVC2007, or
   * functors::StepEccentricity if the elements are steps.
   *
   * Both open ranges (iterators) and closed ranges (circulators) are
   * processed. The whole range [itb, ite) given to init() is used to
   * compute the maximal segments, even for the evaluation on a subrange.
   * The evaluation on a subrange of N elements runs in
   * \f$ O(N + \sum_i L_i) \f$, where the \f$ L_i \f$ are the numbers of
   * elements of the maximal segments passing through the subrange (i.e.
   * in \f$ O(N) \f$ for DSSs).
   *
   * The quantity must be a vector space over the real numbers: it must be
   * default constructible to zero, and support the sum of two quantities
   * and the product by a double on the left (e.g. the normalized tangent
   * vectors of TangentFromDSSEstimator). Note that the integer vectors of
   * TangentVectorFromDSSEstimator are not suitable (they are not normalized
   * and integer), and that averaging angles (as for
   * TangentAngleFromDSSEstimator) is valid only if they do not cross the
   * discontinuity of the angles. Moreover, the quantities must be
   * consistently oriented along the curve (e.g. the normal vectors of
   * NormalFromDCAEstimator, which always point toward the outside):
   * otherwise, opposite vectors cancel out.
   *
   * Here is a basic example of tangent estimation:
   * @code
   typedef ArithmeticalDSSComputer<ConstIterator, int, 4> SegmentComputer;
   typedef TangentFromDSSEstimator<SegmentComputer> SCEstimator;
   typedef LambdaMaximalSegmentEstimator<SegmentComputer, SCEstimator> Estimator;
   Estimator e( SegmentComputer(), SCEstimator() );
   e.init( itb, ite );
   e.eval( itb, ite, std::back_inserter( tangents ), h );
   * @endcode
   *
   * @tparam TSegmentComputer at least a model of concepts::CForwardSegmentComputer
   * @tparam TSCEstimator a model of concepts::CSegmentComputerEstimator
   * @tparam TLambdaFunction a functor mapping a double in [0,1] to a
   * non-negative double (e.g. functors::Lambda64Function (default),
   * functors::LambdaTriangleFunction)
   * @tparam TEccentricity a functor giving the eccentricity in [0,1] of an
   * element from its index in a segment and the number of elements of the
   * segment (functors::PointEccentricity (default) or
   * functors::StepEccentricity)
   *
   * @see MostCenteredMaximalSegmentEstimator
   * @see testLambdaMaximalSegmentEstimator.cpp
   */
  template <typename TSegmentComputer, typename TSCEstimator,
            typename TLambdaFunction = functors::Lambda64Function,
            typename TEccentricity = functors::PointEccentricity>
  class LambdaMaximalSegmentEstimator
  {

    BOOST_CONCEPT_ASSERT(( concepts::CForwardSegmentComputer<TSegmentComputer> ));
    BOOST_CONCEPT_ASSERT(( concepts::CSegmentComputerEstimator<TSCEstimator> ));
    BOOST_STATIC_ASSERT(( boost::is_same< TSegmentComputer,
                          typename TSCEstimator::SegmentComputer >::value ));

    // ----------------------- Types ------------------------------
  public:

    typedef TSegmentComputer SegmentComputer;
    typedef TSCEstimator SCEstimator;
    typedef TLambdaFunction LambdaFunction;
    typedef TEccentricity Eccentricity;
    typedef typename SegmentComputer::ConstIterator ConstIterator;
    typedef typename SCEstimator::Quantity Quantity;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Default constructor. Not valid.
     */
    LambdaMaximalSegmentEstimator() = delete;

    /**
     * Constructor.
     * @param aSegmentComputer a segment computer
     * @param aSCEstimator a segment computer estimator
     * @param aLambdaFunction a lambda function
     * @param anEccentricity an eccentricity functor
     */
    LambdaMaximalSegmentEstimator(const SegmentComputer& aSegmentComputer,
                                  const SCEstimator& aSCEstimator,
                                  const LambdaFunction& aLambdaFunction = LambdaFunction(),
                                  const Eccentricity& anEccentricity = Eccentricity());

    /**
     * Destructor.
     */
    ~LambdaMaximalSegmentEstimator() {};

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Initialisation.
     * @param itb begin iterator of the whole range
     * @param ite end iterator of the whole range
     * (itb == ite for a whole closed range given by circulators)
     */
    void init( const ConstIterator& itb, const ConstIterator& ite );

    /**
     * Unique estimation.
     * @param it any valid iterator of the range given to init()
     * @param h grid size (must be > 0).
     * @return the estimated quantity at *it
     */
    Quantity eval(const ConstIterator& it, const double h = 1.);

    /**
     * Estimation for a subrange [@a itb , @a ite ) of the range given
     * to init() (itb == ite for the whole closed range given by
     * circulators).
     *
     * @param itb subrange begin iterator
     * @param ite subrange end iterator
     * @param result output iterator on the estimated quantity
     * @param h grid size (must be > 0).
     *
     * @return the output iterator after the estimated quantities
     * from itb till ite (excluded)
     */
    template <typename OutputIterator>
    OutputIterator eval(const ConstIterator& itb, const ConstIterator& ite,
                        OutputIterator result, const double h = 1.);

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Private Data --------------------------------
  private:

    /** begin and end iterators of the whole range */
    ConstIterator myBegin, myEnd;

    /** segment computer used to compute the maximal segments */
    SegmentComputer mySC;

    /** object estimating the quantity from a segment computer */
    SCEstimator mySCEstimator;

    /** lambda function */
    LambdaFunction myLambda;

    /** eccentricity functor */
    Eccentricity myEccentricity;

    /** number of elements of the whole range (closed ranges only) */
    std::size_t mySize;

  }; // end of class LambdaMaximalSegmentEstimator

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/curves/estimation/LambdaMaximalSegmentEstimator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined LambdaMaximalSegmentEstimator_h

#undef LambdaMaximalSegmentEstimator_RECURSES
#endif // else defined(LambdaMaximalSegmentEstimator_RECURSES)
