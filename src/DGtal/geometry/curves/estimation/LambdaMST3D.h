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
 * @file LambdaMST3D.h
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 * @date 2014/10/06
 *
 * This file is part of the DGtal library.
 */

#if defined(LAMBDAMST3D_RECURSES)
#error Recursive header files inclusion detected in LambdaMST3D.h
#else // defined(LAMBDAMST3D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define LAMBDAMST3D_RECURSES

#if !defined LAMBDAMST3D_h
/** Prevents repeated inclusion of headers. */
#define LAMBDAMST3D_h

#include <algorithm>
#include <iterator>
#include <cmath>
#include <map>
#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include "DGtal/kernel/CSpace.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/geometry/curves/CIncrementalSegmentComputer.h"
#include "DGtal/geometry/curves/estimation/FunctorsLambdaMST.h"

namespace DGtal {
  
  template < typename TSpace, typename TSegmentation, typename Functor >
  class LambdaMST3DEstimator
  {
  public: 
    ///Checking concepts
    BOOST_CONCEPT_ASSERT(( concepts::CSpace<TSpace> ));
    BOOST_STATIC_ASSERT(( TSpace::dimension == 3 ));
    BOOST_CONCEPT_ASSERT(( CIncrementalSegmentComputer<typename TSegmentation::SegmentComputer> ));
    // ----------------------- Types ------------------------------
  public:
    typedef TSegmentation Segmentation;
    typedef typename TSegmentation::SegmentComputer SegmentComputer;
    typedef typename SegmentComputer::ConstIterator ConstIterator;
    typedef typename Functor::Value Value;
    typedef typename TSpace::RealVector RealVector;
    typedef typename TSpace::Point Point;
    
    // ----------------------- Standard services ------------------------------
  public:
    LambdaMST3DEstimator();
    
    /**
     * Initialisation.
     * @param itb, begin iterator
     * @param ite, end iterator
     */
    void init ( const ConstIterator& itb, const ConstIterator& ite );
    
    /**
     * @param segment - DSS segmentation algorithm
     */
    void attach ( const TSegmentation & aSC );
    
    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;
    
    /**
     * @param point to calculate A and B for it
     * @return A and B
     */
    RealVector eval ( const Point & point );
    
    /**
     * @param result output iterator on the estimated quantity
     *
     * @return the estimated quantity
     * from itb till ite (excluded)
     */
    template < typename Containter >
    void eval ( std::back_insert_iterator < Containter > result );
    
    // ------------------------- Internals ------------------------------------
  protected:
    
    template < typename Containter >
    void accumulate ( std::multimap < Point, Value > & outValues, std::back_insert_iterator < Containter > result );
    
    // ------------------------- Private Datas --------------------------------
  private:
    ConstIterator myBegin;
    ConstIterator myEnd;
    const TSegmentation * dssSegments;
    Functor myFunctor;
    
  }; // end of class LambdaTangentFromDSSEstimator 
  
  /**
   * Description of class 'LambdaTangentFromDSS' <p> Aim:
   */
  template<typename DSS, typename LambdaFunction>
  class TangentFromDSS3DFunctor
  {
  public:
    // ----------------------- Types ------------------------------
    typedef PointVector<3, double> Vector;
    typedef struct t_Value
    {
      Vector first;
      double second = 0.0;
      t_Value & operator += ( const t_Value & ch )
      {
	this->first += ch.first;
	this->second += ch.second;
	return *this;
      }
    } Value;
    
    // ----------------------- Interface --------------------------------------
    Value operator() ( const DSS& aDSS, const int & indexOfPointInDSS, const int & dssLen ) const;
  private:
    // ------------------------- Private Datas --------------------------------
    LambdaFunction lambdaFunctor;
  };
  
  //-------------------------------------------------------------------------------------------
  
  // Template class LambdaMST3D
  /**
   * \brief Aim: Simplify creation of Lambda MST tangent estimator.
   *
   */
  template < typename DSSSegmentationComputer, typename lambdaFunction = functors::Lambda64Function>
  class LambdaMST3D:
  public LambdaMST3DEstimator<Z3i::Space, DSSSegmentationComputer,
    TangentFromDSS3DFunctor< typename DSSSegmentationComputer::SegmentComputer, lambdaFunction> >
    {
      typedef 
      LambdaMST3DEstimator<Z3i::Space, DSSSegmentationComputer,
      TangentFromDSS3DFunctor< typename DSSSegmentationComputer::SegmentComputer, lambdaFunction> > Super;
      
    public: 
      /**
       * Default Constructor.
       */
      LambdaMST3D() : Super() {}
    };
}// namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions
#include "DGtal/geometry/curves/estimation/LambdaMST3D.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined LAMBDAMST3D_h

#undef LAMBDAMST3D_RECURSES
#endif // else defined(LAMBDAMST3D_RECURSES)
