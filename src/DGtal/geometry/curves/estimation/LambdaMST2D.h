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
 * @file LambdaMST2D.h
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 * @date 2014/10/03
 *
 * This file is part of the DGtal library.
 */

#if defined(LAMBDAMST2D_RECURSES)
#error Recursive header files inclusion detected in LambdaMST2D.h
#else // defined(LAMBDAMST2D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define LAMBDAMST2D_RECURSES

#if !defined LAMBDAMST2D_h
/** Prevents repeated inclusion of headers. */
#define LAMBDAMST2D_h

#include <algorithm>
#include <iterator>
#include <cmath>
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/CSpace.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/geometry/curves/CIncrementalSegmentComputer.h"
#include "DGtal/geometry/curves/estimation/FunctorsLambdaMST.h"

/**
 * Aim: Implement Lambda MST tangent estimator.
 */

namespace DGtal {
  template < typename TSpace, typename TSegmentation, typename Functor >
  class LambdaMST2DEstimator
  {
    ///Checking concepts
    BOOST_CONCEPT_ASSERT(( concepts::CSpace<TSpace> ));
    BOOST_STATIC_ASSERT(( TSpace::dimension == 2 ));
    BOOST_CONCEPT_ASSERT(( CIncrementalSegmentComputer<typename TSegmentation::SegmentComputer> ));
    // ----------------------- Types ------------------------------
  public: 
    typedef TSegmentation Segmentation;
    typedef typename TSegmentation::SegmentComputer SegmentComputer;
    typedef typename SegmentComputer::ConstIterator ConstIterator;
    typedef typename Functor::Value Value;
    typedef typename TSpace::RealVector RealVector;
    typedef typename TSpace::Point Point;
    
    // ----------------------- Interface --------------------------------------
  public:
    LambdaMST2DEstimator();
    
    /**
     * Initialisation.
     * @param itb, begin iterator
     * @param ite, end iterator
     */
    void init ( const ConstIterator & itb, const ConstIterator & ite );
    
    /**
     * @param segment - DSS segmentation algorithm
     */
    void attach ( const TSegmentation & aSC );
    
    /**
     * @param point to calculate A and B for it
     * @return A and B
     */
    RealVector eval ( const Point & point );
    
    /**
     * More efficient way to compute tangent for all points in a curve.
     * @param Containter to store results.
     */
    template < typename Containter >
    void eval ( std::back_insert_iterator < Containter > result );
    
    // ----------------------- Standard services ------------------------------
  public:
    
    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;
    // ------------------------- Internals ------------------------------------
  protected:
    
    // Accumulation of partial results
    template < typename Containter >
    void accumulate ( std::vector < Value > & outValues, std::back_insert_iterator < Containter > result );
    
    // ------------------------- Private Datas --------------------------------
  private:
    
    ConstIterator myBegin;
    ConstIterator myEnd;
    Functor myFunctor;
    const TSegmentation * dssSegments;
    
  }; // end of class LambdaTangentFromDSSEstimator 
  
  /**
   * Description of class 'LambdaTangentFromDSS' <p> Aim:
   */
  template<typename DSS, typename LambdaFunction>
  class TangentFromDSS2DFunctor
  {
    // ----------------------- Types ------------------------------
  public:
    typedef PointVector<2, double> RealVector;
    
    typedef struct t_Value
    {
      RealVector first;
      double second;
      t_Value () : second ( 0. ) {}
      t_Value & operator+= ( const t_Value & ch )
      {
	this->first += ch.first;
	this->second += ch.second;
	return *this;
      }
    } Value;
    
    
    // ----------------------- Interface --------------------------------------
  public:
    Value operator() ( const DSS& aDSS, const int & indexOfPointInDSS, const int & dssLen ) const
    {
      Value result;
      double norm = std::sqrt ( aDSS.a() * aDSS.a() + aDSS.b() * aDSS.b() );
      result.second = lambdaFunctor( (double)indexOfPointInDSS / (double)dssLen );
      result.first[0] = result.second * aDSS.a () / norm;
      result.first[1] = result.second * aDSS.b () / norm;
      return result;
    }
  private:
    // ------------------------- Private Datas --------------------------------
    LambdaFunction lambdaFunctor;
  };
  
  //-------------------------------------------------------------------------------------------
  
  // Template class LambdaMST2D
  /**
   * \brief Aim: Simplify creation of Lambda MST tangent estimator.
   *
   */
  template < typename DSSSegmentationComputer, typename LambdaFunction = functors::Lambda64Function >
  class LambdaMST2D: 
  public LambdaMST2DEstimator < Z2i::Space, DSSSegmentationComputer,
    TangentFromDSS2DFunctor < typename DSSSegmentationComputer::SegmentComputer, LambdaFunction > >
    {
      typedef LambdaMST2DEstimator < Z2i::Space, DSSSegmentationComputer,
      TangentFromDSS2DFunctor < typename DSSSegmentationComputer::SegmentComputer, LambdaFunction> > Super;
      
    public: 
      /**
       * Default Constructor.
       */
      LambdaMST2D (): Super() {}
    };
}// namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions
#include "DGtal/geometry/curves/estimation/LambdaMST2D.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined LAMBDAMST2D_h

#undef LAMBDAMST2D_RECURSES
#endif // else defined(LAMBDAMST2D_RECURSES)
