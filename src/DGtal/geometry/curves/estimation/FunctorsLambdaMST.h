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
requires concepts::CUnaryFunctor < LambdaFunction, double, double >
class TangentFromDSS2DFunctor
{
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
  // ------------------------- Private Datas --------------------------------
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
requires concepts::CUnaryFunctor < LambdaFunction, double, double >
class TangentFromDSS3DFunctor
{
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
   * Calculate a direction of the 2D DSS and an eccentricity of a given point in
   * this DSS.
   * @param aDSS digital straight segment
   * @param indexOfPointInDSS index of given point in aDSS
   * @param dssLen length of aDSS
   */
  Value operator() ( const TDSS& aDSS, const unsigned int indexOfPointInDSS, const unsigned int dssLen ) const
  {
    Value result;
    typename DSS::Point3d directionZ3;
    RealVector direction;
    typename DSS::PointR3d intercept;
    typename DSS::PointR3d thikness;
    
    aDSS.getParameters ( directionZ3, intercept, thikness );
    direction[0] = directionZ3[0];
    direction[1] = directionZ3[1];
    direction[2] = directionZ3[2];
    
    result.second = lambdaFunctor ( (double)indexOfPointInDSS / (double)dssLen );
    
    double norm = direction.norm();
    if ( norm != 0. )
      direction /= norm;
    result.first = direction * result.second;
    return result;
  }
private:
  // ------------------------- Private Datas --------------------------------
  //! 
  LambdaFunction lambdaFunctor;
};


/**
 * Description of class 'DSSMuteFilter' -- model of CLMSTDSSFilter.
 * Aim: Provide a functor which does nothing i.e., always returns false.
 * @tparam DSS digital straight segment recognition algorithm
 */
template<typename DSS >
class DSSMuteFilter
{
public:
// ----------------------- Types ------------------------------
  typedef DSS DSSType;
  typedef typename IteratorCirculatorTraits< typename DSSType::ConstIterator >::Value Point;

  /// Always returns false
  bool operator()( const DSSType & ) const
  {
     return false;
  }

  /// Always returns false
  bool admissibility ( const DSSType &, const Point & ) const
  {
    return false;
  }

  /// When called always throws an exception
  long int position ( const DSSType &, const Point & ) const
  {
    throw std::runtime_error ( "You are not suppose to see this error!" );
  }
};


/**
 * Description of class 'DSSLengthLessEqualFilter' -- model of CLMSTDSSFilter.
 * Aim: Provide a functor which allow for filtering DSSes of length lower than
 * a given threshold.
 * @tparam DSS digital straight segment recognition algorithm
 */
template<typename DSS >
class DSSLengthLessEqualFilter
{
public:
// ----------------------- Types ------------------------------
  typedef DSS DSSType;
  typedef typename IteratorCirculatorTraits< typename DSSType::ConstIterator >::Value Point;

  DSSLengthLessEqualFilter ( ) : lenThreshold (0), initThreshold ( false ) { }

  /** Filter initialization method, which has to be used before using the filter
   * @param threshold length threshold used by the filter
   */
  void init ( double threshold )
  {
    if ( threshold < 0. )
      throw std::runtime_error ( "The threshold has to be positive!" );
    lenThreshold = threshold;
    initThreshold = true;
  }

  /**
   * @param dss - the DSS to be checked by the filter
   * @return true if the dss length is smaller than the threshold
   */
  bool operator()( const DSSType & dss ) const
  {
    if (! initThreshold )
      throw std::runtime_error ( "The filter has to be initialized!" );

    return std::distance ( dss.begin ( ), dss.end ( ) ) < lenThreshold;
  }

  /**
   * Checks if the DSS can be used for points that are not covered by any DSS longer than the threshold.
   * @param dss - the DSS to be checked if it is close enough to the point p
   * @param p - the point that is not covered by a long enough DSS
   * @return true if the DSS (see dss) is close enough to the point (see p)
   */
  bool admissibility ( const DSSType & dss, const Point & p ) const
  {
    return ( p - *dss.begin ( ) ).norm ( ) <= lenThreshold || ( p - *( dss.end ( ) - 1 ) ).norm ( ) <= lenThreshold;
  }

  /**
   * Returns position for an uncovered point i.e., point that is not covered by a DSS longer than the threshold,
   * and a DSS that is long enough and it is in the vicinity of the point.
   * @param dss - the DSS that does not cover the point but it is close enough to it
   * @param p - uncovered point
   * @return the position of p with respect to dss
   */
  long int position ( const DSSType & dss, const Point & p ) const
  {
    if ( ( p - *dss.begin ( ) ).norm ( ) <= lenThreshold )
      return 1;
    else if ( ( p - *( dss.end ( ) - 1 ) ).norm ( ) <= lenThreshold )
      return std::distance ( dss.begin ( ), dss.end ( ) ) + 1;
    else
      throw std::runtime_error ( "The DSS and the point are not admissible!" );
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
