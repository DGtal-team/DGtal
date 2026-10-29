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
 * @file FunctorsLambdaMST.h
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, A3SI, France
 *
 * @date 2014/10/08
 *
 * This file is part of the DGtal library.
 */

#if defined(FunctorsLambdaMST_RECURSES)
#error Recursive header files inclusion detected in FunctorsLambdaMST.h
#else // defined(FunctorsLambdaMST_RECURSES)
/** Prevents recursive inclusion of headers. */
#define FunctorsLambdaMST_RECURSES

#if !defined FunctorsLambdaMST_h
/** Prevents repeated inclusion of headers. */
#define FunctorsLambdaMST_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <functional>
#include <iterator>
#include  <stdexcept>
#include "DGtal/base/Common.h"
#include "DGtal/base/IteratorCirculatorTraits.h"
#include "DGtal/base/CUnaryFunctor.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
/////////////////////////////////////////////////////////////////////////////
/**
* Description: Extension of namespace functors by functors related to L-MST.
* \brief Aim: Provide various lambda functions and others L-MST related functors.
* 
* A lambda function \f$\lambda()\f$ - maps from [0,1] \f$\in \mathbb{R}_+\f$ with F(0) = F(1) = 0
* and F() > 0 elsewhere and need to satisfy convexity/concavity property.
* For more information see J.-O. Lachaud et el \cite LachaudIVC2007.
* 
*/
namespace functors
{
  /**
   *  Polynomial lambda functor \cite LachaudIVC2007.
   * \f$ 64 ( -x^6 + 3 x^5 - 3 x^4 + x^3 ) \f$
   * 
   */
  struct Lambda64Function
  {
      double operator() (double x) const
      {
          double e2 = x * x;
          double e3 = e2 * x;
          return 64.0 * ( -e3 * e3 + 3.0 * e3 * e2 - 3.0 * e2 * e2 + e3 );
      }
  };
  /**
   * Sine Lambda functor \cite LachaudIVC2007.
   * \f$ \sin x \pi \f$
   * 
   */
  struct LambdaSinFromPiFunction
  {
      double operator() (double x) const
      {
          return std::sin ( M_PI * x );
      }
  };

  /**
   * Exponential Lambda functor \cite LachaudIVC2007.
   * \f$ \frac{2}{\exp ( 15 (x- \frac{1}{2} ) ) + \exp(15(\frac{1}{2}-x)) } \f$
   * 
   */
  struct LambdaExponentialFunction
  {
      double operator() (double x) const
      {
          return 2.0 / ( std::exp ( 15.0 * ( x - 0.5 ) ) + std::exp ( -15.0 * ( x - 0.5 ) ) );
      }
  };
}

/**
 * Description of class 'TangentFromDSS2DFunctor' -- model of CLMSTTangentFromDSS.
 * Aim: Provide a functor which calculate from digital straight segment 
 * its direction and eccentricity around a given point.
 * @tparam DSS digital straight segment recognition algorithm
 * @tparam LambdaFunction model of CLambdaFunctor @see CLambdaFunctor.h
 */
template<typename DSS, typename LambdaFunction>
class TangentFromDSS2DFunctor
{
  BOOST_CONCEPT_ASSERT(( concepts::CUnaryFunctor < LambdaFunction, double, double > ));
  // ----------------------- Types ------------------------------
public:
  typedef PointVector<2, double> RealVector;
  typedef DSS TDSS;
  
  struct Value
  {
    RealVector first;
    double second;
    Value () : second ( 0. ) {}
    Value & operator+= ( const Value & ch )
    {
      this->first += ch.first;
      this->second += ch.second;
      return *this;
    }
  };
  
  
  // ----------------------- Interface --------------------------------------
public:
  /**
   * Calculate a direction of the 2D DSS and an eccentricity of a given point in
   * this DSS.
   * @param aDSS digital straight segment
   * @param indexOfPointInDSS index of given point in aDSS
   * @param dssLen length of aDSS
   */
  Value operator() ( const TDSS& aDSS, const int & indexOfPointInDSS, const int & dssLen ) const
  {
    Value result;
    double norm = std::sqrt ( aDSS.a() * aDSS.a() + aDSS.b() * aDSS.b() );
    result.second = lambdaFunctor ( (double)indexOfPointInDSS / (double)dssLen );
    if ( norm > 0. )
    {
      result.first[0] = result.second * aDSS.b() / norm;
      result.first[1] = result.second * aDSS.a() / norm;
    }
    else
    {
      result.first[0] = 0.;
      result.first[1] = 0.;
    }
    return result;
  }
private:
  // ------------------------- Private Data --------------------------------
  LambdaFunction lambdaFunctor;
};

/**
 * Description of class 'TangentFromDSS3DFunctor' -- model of CLMSTTangentFromDSS.
 * Aim: Provide a functor which calculate from digital straight segment 
 * its direction and eccentricity around a given point.
 * @tparam DSS digital straight segment recognition algorithm
 * @tparam LambdaFunction model of CLambdaFunctor @see CLambdaFunctor.h
 */
template<typename DSS, typename LambdaFunction>
class TangentFromDSS3DFunctor
{
  BOOST_CONCEPT_ASSERT(( concepts::CUnaryFunctor < LambdaFunction, double, double > ));
public:
  // ----------------------- Types ------------------------------
  typedef PointVector<3, double> RealVector;
  typedef DSS TDSS;
  struct Value
  {
    RealVector first;
    double second;
    Value () : second ( 0. ) {}
    Value & operator += ( const Value & ch )
    {
      this->first += ch.first;
      this->second += ch.second;
      return *this;
    }
  };
  
  // ----------------------- Interface --------------------------------------
  /**
   * The direction of the 3D DSS (unit length) weighted by lambda ( num / den ).
   * @param aDSS digital straight segment
   * @param num numerator of the eccentricity of the point with respect to aDSS
   * @param den its denominator -- see LambdaMST3DEstimator::contribute for both
   */
  Value operator() ( const TDSS& aDSS, const unsigned int num, const unsigned int den ) const
  {
    Value result;
    typename DSS::Point3d directionZ3;
    RealVector direction;
    typename DSS::PointR3d intercept;
    typename DSS::PointR3d thickness;
    
    aDSS.getParameters ( directionZ3, intercept, thickness );
    direction[0] = directionZ3[0];
    direction[1] = directionZ3[1];
    direction[2] = directionZ3[2];
    
    result.second = lambdaFunctor ( (double)num / (double)den );
    
    double norm = direction.norm();
    if ( norm != 0. )
      direction /= norm;
    result.first = direction * result.second;
    return result;
  }
private:
  // ------------------------- Private Data --------------------------------
  //! 
  LambdaFunction lambdaFunctor;
};


/**
 * Description of class 'DSSMuteFilter' -- model of CLMSTDSSFilter that changes
 * nothing: every maximal segment is kept, and a segment enters only the
 * pencils of the points it contains (reach 0). This is the plain 3D
 * lambda-MST.
 */
template<typename DSS >
class DSSMuteFilter
{
public:
  typedef DSS DSSType;
  /**
   * @return false: no segment is removed from the pencils.
   */
  bool operator()( const DSSType & ) const
  {
     return false;
  }
  /**
   * @return 0: a segment enters only the pencils of the points it contains.
   */
  double reach ( const DSSType & ) const
  {
    return 0.;
  }
};

/**
 * Description of class 'DSSLengthLessEqualFilter' -- model of CLMSTDSSFilter
 * with a length threshold T, in points:
 *
 * - a maximal segment with fewer than T points is left out of every pencil
 *   (short segments carry unreliable directions);
 * - a segment of at least T points enters the pencil of the points it
 *   contains and also of every point up to T points beyond either of its
 *   ends, counted along the curve, so that a point whose own containing
 *   segments were all removed still receives the long segments next to it.
 *
 * A natural choice is T = l / 2, l being the number of points of the
 * shortest 2D maximal segment of the two projections of the curve onto the
 * coordinate planes containing its main axis. With that choice a long enough
 * segment always lies within T points of every point of a curve that is
 * monotone along one axis, so no pencil ends up empty there.
 */
template<typename DSS >
class DSSLengthLessEqualFilter
{
public:
  typedef DSS DSSType;
  DSSLengthLessEqualFilter ( ) : lenThreshold (0), initThreshold ( false ) { }
  /// @param threshold T, in points (see the class description for a natural choice).
  void init ( double threshold )
  {
    if ( threshold < 0. )
      throw std::runtime_error ( "The threshold has to be non-negative!" );
    lenThreshold = threshold;
    initThreshold = true;
  }
  /**
   * @param dss a maximal segment
   * @return true if @a dss has fewer than T points, i.e. it is removed from
   * every pencil.
   */
  bool operator()( const DSSType & dss ) const
  {
    if (! initThreshold )
      throw std::runtime_error ( "The filter has to be initialized!" );
    return std::distance ( dss.begin ( ), dss.end ( ) ) < lenThreshold;
  }
  /**
   * @return T: a kept segment also enters the pencils of the points up to T
   * points beyond either of its ends, counted along the curve.
   */
  double reach ( const DSSType & ) const
  {
    if (! initThreshold )
      throw std::runtime_error ( "The filter has to be initialized!" );
    return lenThreshold;
  }
private:
    double lenThreshold;
    bool initThreshold;
};
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined FunctorsLambdaMST_h

#undef FunctorsLambdaMST_RECURSES
#endif // else defined(FunctorsLambdaMST_RECURSES)
