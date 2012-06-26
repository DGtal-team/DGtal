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
 * @file SegmentComputerFunctor.h
 * @brief Various local estimators from segment computers.
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 *
 *
 * @date 2011/06/28
 *
 * This file is part of the DGtal library.
 *
 * @see testSegmentComputerFunctor.cpp
 */

#if defined(SegmentComputerFunctor_RECURSES)
#error Recursive header files inclusion detected in SegmentComputerFunctor.h
#else // defined(SegmentComputerFunctor_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SegmentComputerFunctor_RECURSES

#if !defined SegmentComputerFunctor_h
/** Prevents repeated inclusion of headers. */
#define SegmentComputerFunctor_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <cmath>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "boost/utility.hpp"

//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
  namespace detail
  {
  /////////////////////////////////////////////////////////////////////////////
  // class PosIndepScaleIndepSCFunctor
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of class 'PosIndepScaleIndepSCFunctor' <p> Aim: 
   * estimates a geometrical quantity from a segment computer. 
   * The estimation is neither position-dependant 
   * nor scale-dependant (e.g. tangent or normal 
   * estimation from 'straight' primitives). 
   *
   * @tparam TSegmentComputer a model of segment computer. 
   *
   * The computation is delegated to a functor.  
   *
   * @tparam Functor a functor
   */

  template <typename TSegmentComputer, typename Functor, 
	    typename ReturnType = typename Functor::Quantity>
  class PosIndepScaleIndepSCFunctor
  {

  public: 

    // ----------------------- inner type ------------------------------
    typedef TSegmentComputer SegmentComputer;
    typedef ReturnType Quantity;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() 
    {
      return true; 
    };

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Init method
     * Does nothing 
     */
    void init( const double& ) {};

    /**
     * Operator()
     *
     * @return the estimation
     *
     * @param aSC an instance of segment computer. 
     */
    Quantity operator()( const typename SegmentComputer::ConstIterator&, 
			 const SegmentComputer& aSC, 
			 const bool&,
			 const bool& ) const 
    {
      return this->operator()( aSC ); 
    };

    /**
     * Operator()
     *
     * @return the estimation
     *
     * @param aSC an instance of segment computer. 
     */
    Quantity operator()(const SegmentComputer& aSC) const 
    {
      Functor f; 
      return f( aSC ); 
    };

  }; // end of class PosIndepScaleIndepSCFunctor

  /////////////////////////////////////////////////////////////////////////////
  // class PosIndepScaleDepSCFunctor
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of class 'PosIndepScaleDepSCFunctor' <p> Aim: 
   * estimates a geometrical quantity from a segment computer. 
   * The estimation is not position-dependant,
   * but is scale-dependant (e.g. curvature or radius
   * estimation from 'circular' primitives). 
   *
   * @tparam TSegmentComputer a model of segment computer. 
   *
   * The computation is delegated to a functor.  
   *
   * @tparam Functor a functor
   */

  template <typename TSegmentComputer, typename Functor, 
	    typename ReturnType = typename Functor::Quantity>
  class PosIndepScaleDepSCFunctor
  {

  public: 

    // ----------------------- inner type ------------------------------
    typedef TSegmentComputer SegmentComputer;
    typedef ReturnType Quantity;

    // ----------------------- internal data ------------------------------
  public:
    double myH; 

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Default constructor.
     * NB: not valid.
     */
    PosIndepScaleDepSCFunctor()
      : myH( 0.0 )
    {
    }
    /**
     * Copy constructor.
     * @param other the object to copy.
     */
    PosIndepScaleDepSCFunctor( const PosIndepScaleDepSCFunctor& other )
      : myH( other.myH )
    {
    }
    /**
     * Assignement.
     * @param other the object to copy.
     */
    PosIndepScaleDepSCFunctor& operator=( const PosIndepScaleDepSCFunctor& other )
    {
      if (this != &other)
	{
	  myH = other.myH; 
	}
      return *this; 
    }
    /**
     * Destructor
     */
    ~PosIndepScaleDepSCFunctor() {}

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() 
    {
      return (myH > 0); 
    };

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Init method: set @e myH
     */
    void init( const double& aH ) 
    {
      myH = aH;
      ASSERT( isValid() ); 
    };

    /**
     * Operator()
     *
     * @return the estimation
     *
     * @param aSC an instance of segment computer. 
     */
    Quantity operator()( const typename SegmentComputer::ConstIterator&, 
			 const SegmentComputer& aSC, 
			 const bool&,
			 const bool& ) const 
    {
      return this->operator()( aSC ); 
    };

    /**
     * Operator()
     *
     * @return the estimation
     *
     * @param aSC an instance of segment computer. 
     */
    Quantity operator()(const SegmentComputer& aSC) const 
    {
      Functor f; 
      return f( aSC, myH ); 
    };

  }; // end of class PosIndepScaleDepSCFunctor

  /////////////////////////////////////////////////////////////////////////////
  // class PosDepScaleIndepSCFunctor
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of class 'PosDepScaleIndepSCFunctor' <p> Aim: 
   * estimates a geometrical quantity from a segment computer. 
   * The estimation is not scale dependant but position dependant
   * (e.g. tangent or normal estimation from high-order primitives). 
   *
   * @tparam TSegmentComputer a model of segment computer. 
   *
   * The computation is delegated to a functor.  
   *
   * @tparam Functor a functor
   */

  template <typename TSegmentComputer, typename Functor, 
	    typename ReturnType = typename Functor::Quantity>
  class PosDepScaleIndepSCFunctor
  {

  public: 

    // ----------------------- inner type ------------------------------
    typedef TSegmentComputer SegmentComputer;
    typedef ReturnType Quantity;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() 
    {
      return true; 
    };

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Init method
     * Does nothing 
     */
    void init( const double& ) {};

    /**
     * Operator()
     *
     * @return the estimation
     *
     * @param aSC an instance of segment computer. 
     */
    Quantity operator()( const typename SegmentComputer::ConstIterator&, 
			 const SegmentComputer& aSC, 
			 const bool&,
			 const bool& ) const 
    {
      return this->operator()( aSC ); 
    };

    /**
     * Operator()
     *
     * @return the estimation
     *
     * @param it position where the estimation has to be done.
     * @param aSC an instance of segment computer. 
     */
    Quantity operator()(const typename SegmentComputer::ConstIterator& it, 
			const SegmentComputer& aSC) const 
    {
      Functor f; 
      return f( it, aSC ); 
    };

  }; // end of class PosDepScaleIndepSCFunctor

  /////////////////////////////////////////////////////////////////////////////
  // class PosDepScaleDepSCFunctor
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of class 'PosDepScaleDepSCFunctor' <p> Aim: 
   * estimates a geometrical quantity from a segment computer. 
   * The estimation is both position-dependant and scale-dependant 
   * (typically distance of a point to an underlying curve). 
   *
   * @tparam TSegmentComputer a model of segment computer. 
   *
   * The computation is delegated to a functor.  
   *
   * @tparam Functor a functor
   */

  template <typename TSegmentComputer, typename Functor, 
	    typename ReturnType = typename Functor::Quantity>
  class PosDepScaleDepSCFunctor
  {

  public: 

    // ----------------------- inner type ------------------------------
    typedef TSegmentComputer SegmentComputer;
    typedef ReturnType Quantity;

    // ----------------------- internal data ------------------------------
  public:
    double myH; 

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Default constructor.
     * NB: not valid.
     */
    PosDepScaleDepSCFunctor()
      : myH( 0.0 )
    {
    }
    /**
     * Copy constructor.
     * @param other the object to copy.
     */
    PosDepScaleDepSCFunctor( const PosDepScaleDepSCFunctor& other )
      : myH( other.myH )
    {
    }
    /**
     * Assignement.
     * @param other the object to copy.
     */
    PosDepScaleDepSCFunctor& operator=( const PosDepScaleDepSCFunctor& other )
    {
      if (this != &other)
	{
	  myH = other.myH; 
	}
      return *this; 
    }
    /**
     * Destructor
     */
    ~PosDepScaleDepSCFunctor() {}

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() 
    {
      return (myH > 0); 
    };

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Init method: set @e myH
     */
    void init( const double& aH ) 
    {
      myH = aH;
      ASSERT( isValid() ); 
    };

    /**
     * Operator()
     *
     * @return the estimation
     *
     * @param aSC an instance of segment computer. 
     */
    Quantity operator()( const typename SegmentComputer::ConstIterator&, 
			 const SegmentComputer& aSC, 
			 const bool&,
			 const bool& ) const 
    {
      return this->operator()( aSC ); 
    };

    /**
     * Operator()
     *
     * @return the estimation
     *
     * @param it position where the estimation is performed
     * @param aSC an instance of segment computer. 
     */
    Quantity operator()(const typename SegmentComputer::ConstIterator& it, 
			const SegmentComputer& aSC) const 
    {
      Functor f; 
      return f( it, aSC, myH ); 
    };

  }; // end of class PosDepScaleDepSCFunctor

    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
  /**
   * Description of class 'TangentAngleFromDSS' <p> Aim: 
   * computes the tangent orientation of a DSS 
   * (angle in [-pi,+pi] radians between the tangent and the x-axis).
   */
    struct TangentAngleFromDSS
    {      
    public:
      typedef double Quantity; 


    /**
     * Operator()
     *
     * @return the angle of type double
     * (angle in [-pi,+pi] radians between the tangent and the x-axis).
     *
     * @param aDSS an instance of segment computer
     * devoted to the DSS recognition.
     *
     * @tparam DSS a model of segment computer,
     * which must have methods getA() and getB()
     * returning the y- and x-component of the tangent vector.
     */
      template<typename DSS>
      Quantity operator() (const DSS& aDSS) const 
      {
	Quantity a = (Quantity) NumberTraits<typename DSS::Integer>
	  ::castToInt64_t(aDSS.getA());      
	Quantity b = (Quantity) NumberTraits<typename DSS::Integer>
	  ::castToInt64_t(aDSS.getB());      

	return std::atan2(a,b);
      }
    }; 
  /**
   * Description of class 'NormalizedTangentVectorFromDSS' <p> Aim: 
   * computes the unit vector of a DSS 
   */
    struct NormalizedTangentVectorFromDSS
    {      
    public:
      typedef DGtal::PointVector<2,double> RealVector; 
      typedef RealVector Quantity;

    /**
     * Operator()
     *
     * @return normalized tangent 
     *
     * @param aDSS an instance of segment computer
     * devoted to the DSS recognition.

     * @tparam DSS a model of segment computer,
     * which must have methods getA() and getB()
     * returning the y- and x-component of the tangent vector.
     */
      template<typename DSS>
      Quantity operator() (const DSS& aDSS) const 
      {
	double x = NumberTraits<typename DSS::Integer>
	  ::castToDouble( aDSS.getB() ); 
	double y = NumberTraits<typename DSS::Integer>
	  ::castToDouble( aDSS.getA() );
	RealVector v(x,y); 
	double norm = v.norm(RealVector::L_2);
	v /= norm; 
	return v;
      }
    }; 
  /**
   * Description of class 'TangentVectorFromDSS' <p> Aim: 
   * computes the tangent vector of a DSS 
   */
    template<typename DSS>
    struct TangentVectorFromDSS
    {      
    public:
      typedef typename DSS::Vector Quantity;

    /**
     * Operator()
     *
     * @return tangent vector 
     *
     * @param aDSS an instance of segment computer
     * devoted to the DSS recognition.

     * @tparam DSS a model of segment computer,
     * which must have methods getA() and getB()
     * returning the y- and x-component of the tangent vector.
     */
      Quantity operator() (const DSS& aDSS) const 
      {
	return Quantity(aDSS.getB(), aDSS.getA());
      }
    }; 
  /**
   * Description of class 'CurvatureFromDCA' <p> Aim: 
   * computes the curvature from a GeometricalDCA
   * at a given grid step.
   *
   * @tparam isCCW boolean equal to 'true' 
   * for a scanning in a counter-clockwise (CCW) 
   * orientation, 'false' otherwise. 
   * For instance, the estimated curvature of 
   * a digital circle, scanned in a CCW (resp. CW)
   * orientation, is positive (resp. negative). 
   */
    template<bool isCCW = true>
    struct CurvatureFromDCA
    {      
    public:
      typedef double Quantity; 

    /**
     * Operator()
     *
     * @return curvature 
     *
     * @param aDCA an instance of segment computer
     * devoted to the DCA recognition.
     * @param aH grid step
     *
     * @tparam DCA a model of segment computer
     * devoted to the DCA recognition, 
     * basically GeometricalDCA.
     */
      template<typename DCA>
      Quantity operator() (const DCA& aDCA, const double& aH = 1.0) const 
      {
  	if ( aDCA.isStraight() )
  	  return 0.0; 
	else
	return ( aDCA.getSeparatingCircle().getCurvature() / aH );
      }
    }; 
    template<>
    struct CurvatureFromDCA<false>
    {      
    public:
      typedef double Quantity; 

      template<typename DCA>
      Quantity operator() (const DCA& aDCA, const Quantity& aH = 1.0) const 
      {
  	if ( aDCA.isStraight() )
  	  return 0.0; 
	else
	return - ( aDCA.getSeparatingCircle().getCurvature() / aH );
      }
    }; 
  /**
   * Description of class 'NormalVectorFromDCA' <p> Aim: 
   * estimates the normal at a given position from a GeometricalDCA. 
   */
    struct NormalVectorFromDCA
    {      
    public:
      typedef PointVector<2,double> Quantity; 


    /**
     * Operator()
     *
     * @return normal at @e it 
     *
     * @param it position where the estimation has to be done
     * @param aDCA an instance of segment computer
     * devoted to the DCA recognition.
     *
     * @tparam DCA a model of segment computer
     * devoted to the DCA recognition, 
     * basically GeometricalDCA.
     */
      template<typename DCA>
      Quantity operator() (const typename DCA::ConstIterator& it, 
  			   const DCA& aDCA) const 
      {
	typedef typename DCA::ConstIterator ConstIterator; 
	typedef typename DCA::Pair Pair; 
	typedef typename DCA::Point Point;
	typedef typename Point::Coordinate Coordinate; 
	
  	if ( !aDCA.isStraight() )
  	  {
  	    //separating circle center
  	    double c0, c1, r; 
  	    aDCA.getSeparatingCircle().getParameters(c0, c1, r);
  	    //point
	    Pair pair = *it; 
	    Point i = pair.first; 
	    Point o = pair.second;
	    double m0 = NumberTraits<Coordinate>::castToDouble(i[0]+o[0]) / 2.0; 
	    double m1 = NumberTraits<Coordinate>::castToDouble(i[1]+o[1]) / 2.0;
	    //normal vector 
	    double v0 = m0 - c0; 
	    double v1 = m1 - c1; 
	    //norm
	    double n = std::sqrt(v0*v0 + v1*v1); 
	    return Quantity( v0/n, v1/n );
  	  }
  	else
  	  {
	    //separating straight line and normal vector
	    double a, b, c; 
	    aDCA.getGeometricalDSSPtr()->getParameters(a, b, c);
	    //norm
	    double n = std::sqrt(a*a + b*b); 
  	    return Quantity( a/n, b/n ); 
  	  }
      }
    }; 

  /**
   * Description of class 'TangentVectorFromDCA' <p> Aim: 
   * estimates the tangent at a given position from a GeometricalDCA. 
   */
    struct TangentVectorFromDCA
    {      
    public:
      typedef PointVector<2,double> Quantity; 

    /**
     * Operator()
     *
     * @return tangent at @e it 
     *
     * @param it position where the estimation has to be done
     * @param aDCA an instance of segment computer
     * devoted to the DCA recognition.
     *
     * @tparam DCA a model of segment computer
     * devoted to the DCA recognition, 
     * basically GeometricalDCA.
     *
     * @see NormalVectorFromDCA
     */
      template<typename DCA>
      Quantity operator() (const typename DCA::ConstIterator& it, 
  			   const DCA& aDCA) const 
      {
	NormalVectorFromDCA f; 
	Quantity normal = f(it, aDCA); 
	return Quantity( normal[1], normal[0] ); 
      }
    }; 

  /**
   * Description of class 'DistanceFromDCA' <p> Aim: 
   * estimates the distance of a given pair of points
   * to the seperating circle of a DCA. 
   */
    struct DistanceFromDCA
    {      
    public:
      typedef std::pair<double,double> Quantity; 

    /**
     * Operator()
     *
     * @return distances (in a pair) of the 
     * inner and outer points pointed by @e it
     * to the separating circle of @e aDCA
     *
     * @param it position where the estimation has to be done
     * @param aDCA an instance of segment computer
     * devoted to the DCA recognition.
     * @param aH grid step
     *
     * @tparam DCA a model of segment computer
     * devoted to the DCA recognition, 
     * basically GeometricalDCA.
     */
      template<typename DCA>
      Quantity operator() (const typename DCA::ConstIterator& it, 
  			   const DCA& aDCA, const double& aH) const 
      {
	typedef typename DCA::ConstIterator ConstIterator; 
	typedef typename DCA::Pair Pair; 
	typedef typename DCA::Point Point;
	typedef typename Point::Coordinate Coordinate; 
	
  	if ( !aDCA.isStraight() )
  	  {
  	    //separating circle center
  	    double c0, c1, r; 
  	    aDCA.getSeparatingCircle().getParameters(c0, c1, r);
  	    //points
	    Pair pair = *it; 
	    Point i = pair.first; 
	    Point o = pair.second;
	    //distances
	    double distI0 = NumberTraits<Coordinate>::castToDouble(i[0]) - c0; 
	    double distI1 = NumberTraits<Coordinate>::castToDouble(i[1]) - c1;
	    double distI = std::sqrt( distI0*distI0 + distI1*distI1 ) - r; 
	    double distO0 = NumberTraits<Coordinate>::castToDouble(o[0]) - c0; 
	    double distO1 = NumberTraits<Coordinate>::castToDouble(o[1]) - c1;
	    double distO = std::sqrt( distO0*distO0 + distO1*distO1 ) - r; 
	    return Quantity( distI*aH, distO*aH );
  	  }
  	else
  	  {
	    //separating straight line
	    double a, b, c; 
	    aDCA.getGeometricalDSSPtr()->getParameters(a, b, c); 
	    //norm
	    double n = std::sqrt(a*a + b*b); 
  	    //points
	    Pair pair = *it; 
	    Point i = pair.first; 
	    Point o = pair.second;
	    //distances
	    double rI = NumberTraits<Coordinate>::castToDouble(i[0])*a + 
	      NumberTraits<Coordinate>::castToDouble(i[1])*b + c;
	    double distI = rI / n; 
	    double rO = NumberTraits<Coordinate>::castToDouble(o[0])*a + 
	      NumberTraits<Coordinate>::castToDouble(o[1])*b + c;
	    double distO = rO / n; 
  	    return Quantity( distI*aH, distO*aH ); 
  	  }
      }
    }; 

  }//namespace detail



  //-------------------------------------------------------------------------------------------
  /**
   * Description of class 'TangentFromDSSFunctor' <p> Aim: 
   * estimates the (normalized) tangent vector from a DSS
   * recognized by some segment computers. 
   *
   * @tparam DSSComputer a model of segment computer
   * devoted the DSS recognition
   */
  template <typename DSSComputer>
  class TangentFromDSSFunctor: 
    public detail::PosIndepScaleIndepSCFunctor<DSSComputer, detail::NormalizedTangentVectorFromDSS>
  {
    typedef 
    detail::PosIndepScaleIndepSCFunctor<DSSComputer, detail::NormalizedTangentVectorFromDSS> 
    Super; 

  public: 
    /**
     * Default Constructor.
     */
    TangentFromDSSFunctor(): Super() {};
    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    TangentFromDSSFunctor( const TangentFromDSSFunctor & other ): Super(other) {};
  }; 

  //-------------------------------------------------------------------------------------------
  /**
   * Description of class 'TangentVectorFromDSSFunctor' <p> Aim: 
   * estimates the (not normalized) tangent vector from the slope
   * parameters of a DSS recognized by a segment computer. 
   *
   * @tparam DSSComputer a model of segment computer
   * devoted the DSS recognition
   */
  template <typename DSSComputer>
  class TangentVectorFromDSSFunctor: 
    public detail::PosIndepScaleIndepSCFunctor<DSSComputer, detail::TangentVectorFromDSS<DSSComputer> >
  {
    typedef 
    detail::PosIndepScaleIndepSCFunctor<DSSComputer, detail::TangentVectorFromDSS<DSSComputer> > 
    Super; 

  public: 
    /**
     * Default Constructor.
     */
    TangentVectorFromDSSFunctor(): Super() {};
    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    TangentVectorFromDSSFunctor( const TangentVectorFromDSSFunctor & other ): Super(other) {};
  }; 

  //-------------------------------------------------------------------------------------------
  /**
   * Description of class 'TangentAngleFromDSSFunctor' <p> Aim: 
   * estimates the tangent angle from a DSS
   * recognized by some segment computers. 
   * (angle in [-pi,+pi] radians between the tangent and the x-axis).
   *
   * @tparam DSSComputer a model of segment computer
   * devoted the DSS recognition
   */
  template <typename DSSComputer>
  class TangentAngleFromDSSFunctor: 
    public detail::PosIndepScaleIndepSCFunctor<DSSComputer, detail::TangentAngleFromDSS>
  {
    typedef 
    detail::PosIndepScaleIndepSCFunctor<DSSComputer, detail::TangentAngleFromDSS> 
    Super; 

  public: 
    /**
     * Default Constructor.
     */
    TangentAngleFromDSSFunctor(): Super() {};
    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    TangentAngleFromDSSFunctor( const TangentAngleFromDSSFunctor & other ): Super(other) {};
  }; 

  //-------------------------------------------------------------------------------------------
  /**
   * Description of class 'CurvatureFromDCAFunctor' <p> Aim: 
   * estimates the curvature from a DCA
   * recognized by a segment computer, 
   * basically GeometricalDCA.
   *
   * @tparam DCAComputer a model of segment computer
   * devoted the DCA recognition
   *
   * @tparam isCCW boolean equal to 'true' (default)
   * for a scanning in a counter-clockwise (CCW) 
   * orientation, 'false' otherwise, i.e in a 
   * clockwise orientation (CW). 
   * For instance, the estimated curvature of 
   * a digital circle, scanned in a CCW (resp. CW)
   * orientation, is positive (resp. negative). 
   */
  template <typename DCAComputer, bool isCCW = true>
  class CurvatureFromDCAFunctor: 
    public detail::PosIndepScaleDepSCFunctor<DCAComputer, 
					     detail::CurvatureFromDCA<isCCW> >
  {
    typedef 
    detail::PosIndepScaleDepSCFunctor<DCAComputer, detail::CurvatureFromDCA<isCCW> > 
    Super; 

  public: 
    /**
     * Default Constructor.
     */
    CurvatureFromDCAFunctor(): Super() {};
    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    CurvatureFromDCAFunctor( const CurvatureFromDCAFunctor & other ): Super(other) {};
  }; 

  //-------------------------------------------------------------------------------------------
  /**
   * Description of class 'NormalFromDCAFunctor' <p> Aim: 
   * estimates the (normalized) normal vector at some position from a DCA
   * recognized by a segment computer, basically GeometricalDCA.
   *
   * @tparam DCAComputer a model of segment computer
   * devoted the DCA recognition
   */
  template <typename DCAComputer>
  class NormalFromDCAFunctor: 
    public detail::PosDepScaleIndepSCFunctor<DCAComputer, 
						detail::NormalVectorFromDCA>
  {
    typedef 
    detail::PosDepScaleIndepSCFunctor<DCAComputer, detail::NormalVectorFromDCA> 
    Super; 

  public: 
    /**
     * Default Constructor.
     */
    NormalFromDCAFunctor(): Super() {};
    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    NormalFromDCAFunctor( const NormalFromDCAFunctor & other ): Super(other) {};
  }; 

  //-------------------------------------------------------------------------------------------
  /**
   * Description of class 'TangentFromDCAFunctor' <p> Aim: 
   * estimates the (normalized) tangent vector at some position from a DCA
   * recognized by a segment computer, basically GeometricalDCA.
   *
   * @tparam DCAComputer a model of segment computer
   * devoted the DCA recognition
   */
  template <typename DCAComputer>
  class TangentFromDCAFunctor: 
    public detail::PosDepScaleIndepSCFunctor<DCAComputer, 
						detail::TangentVectorFromDCA>
  {
    typedef 
    detail::PosDepScaleIndepSCFunctor<DCAComputer, detail::TangentVectorFromDCA> 
    Super; 

  public: 
    /**
     * Default Constructor.
     */
    TangentFromDCAFunctor(): Super() {};
    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    TangentFromDCAFunctor( const TangentFromDCAFunctor & other ): Super(other) {};
  }; 

  //-------------------------------------------------------------------------------------------
  /**
   * Description of class 'DistanceFromDCAFunctor' <p> Aim: 
   * estimates the (Euclidean) distance of some points to 
   * the separating circle of a DCA recognized by a 
   * segment computer, basically GeometricalDCA.
   *
   * @tparam DCAComputer a model of segment computer
   * devoted the DCA recognition
   */
  template <typename DCAComputer>
  class DistanceFromDCAFunctor: 
    public detail::PosDepScaleDepSCFunctor<DCAComputer, 
					   detail::DistanceFromDCA>
  {
    typedef 
    detail::PosDepScaleDepSCFunctor<DCAComputer, detail::DistanceFromDCA> 
    Super; 

  public: 
    /**
     * Default Constructor.
     */
    DistanceFromDCAFunctor(): Super() {};
    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    DistanceFromDCAFunctor( const DistanceFromDCAFunctor & other ): Super(other) {};
  }; 

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
  namespace detail
  {
  /**
   * Description of class 'CurvatureFromDSSLength' <p> Aim: 
   * Computes the curvature @f$ k @f$ from the discrete length @f$ l @f$ of a DSS 
   * as follow: 
   * @f$ 1/k = l*l/8 + 1/2 @f$
   *
   * @note Adaption from 
   *  Coeurjolly, D. and Miguet, S. and Tougne, L.
   *  "Discrete Curvature Based on Osculating Circle Estimation", 
   * Proc. IWVF, LNCS, vol 2059, pp.303-312, 2001
   */
    struct CurvatureFromDSSLength
    {      
    public:
      typedef double Quantity; 

      template<typename DSS>
      Quantity operator() (const DSS& aDSS) const 
      {
	typedef typename DSS::Vector Vector; 
	//length
	Vector v = ( *aDSS.begin() - *boost::prior(aDSS.end()) ); 
	Quantity l = v.norm(Vector::L_2);
	//result
	return 1/( (l*l)/8 + 0.5 );  
      }
    }; 

  /**
   * Description of class 'CurvatureFromDSSLengthAndWidth' <p> Aim: 
   * computes the curvature @f$ k @f$ from 
   * the length  @f$ l @f$ and the width  @f$ w @f$ of a DSS as follow: 
   *  @f$ 1/k = (l*l)/(8*w) + w/2 @f$
   *
   * @note Adaption from 
   *  Coeurjolly, D. and Miguet, S. and Tougne, L.
   *  "Discrete Curvature Based on Osculating Circle Estimation", 
   * Proc. IWVF, LNCS, vol 2059, pp.303-312, 2001
   */
    struct CurvatureFromDSSLengthAndWidth
    {      
    public:
      typedef double Quantity; 

      template<typename DSS>
      Quantity operator() (const DSS& aDSS) const 
      {
	typedef typename DSS::Vector Vector; 
	//length
	Vector v = ( *aDSS.begin() - *boost::prior(aDSS.end()) ); 
	Quantity l = v.norm(Vector::L_2);
	//width
	Vector t( aDSS.getB(), aDSS.getA() );
	Quantity w = 1.0 / v.norm(Vector::L_2); 
	//result
	return 1.0/( (l*l)/(8*w) + w/2 ); 
      }
    }; 

  /////////////////////////////////////////////////////////////////////////////
  // class CurvatureFromDSSBaseFunctor
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of class 'CurvatureFromDSSBaseFunctor' <p> Aim: 
   * computes a curvature quantity from the length and/or the width of a DSS.
   *
   * @tparam DSSComputer a model of segment computer 
   * devoted to the DSS recognition.
   *
   * The computation is delegated to a functor. 
   *
   * @tparam Functor a model of unary functor
   * taking a DSS as input and returning a double
   *
   */

  template <typename DSSComputer, typename Functor = detail::CurvatureFromDSSLength >
  class CurvatureFromDSSBaseFunctor
  {

  public: 

    // ----------------------- inner type ------------------------------------
    typedef DSSComputer SegmentComputer; 
    typedef double Quantity;

    BOOST_CONCEPT_ASSERT(( CUnaryFunctor< Functor, SegmentComputer, Quantity > ));  


    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~CurvatureFromDSSBaseFunctor(){};

    /**
     * Default Constructor.
     */
    CurvatureFromDSSBaseFunctor(): myH(0.0) {};

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    CurvatureFromDSSBaseFunctor( const CurvatureFromDSSBaseFunctor & other ): myH( other.myH) {};

    /**
     * Assignement
     * @param other the object to clone.
     */
    CurvatureFromDSSBaseFunctor& operator=( const CurvatureFromDSSBaseFunctor & other )
    {
      if (this != &other)
	{
	  myH = other.myH;
	}
      return *this; 
    };

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() 
    {
      return (myH > 0); 
    };


    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Init method
     * @param aH grid step.
     */
    void init( const double& aH = 1.0  ) 
    {
      if (aH > 0)
   	myH = aH; 
      else 
   	{
   	  std::cerr << "[DGtal::CurvatureFromDSSBaseFunctor<DSSComputer>::init(const double& aH)]"
   		    << " ERROR. aH should be strictly greater than 0." << std::endl;
   	  throw InputException(); 
   	}
    };


    /**
     * Operator() 
     * 
     * @return the curvature at position @a it
     * @param it the position at which the curvature is estimated.
     * @param aDSS a DSSComputer. 
     * @param isExtendableAtBack a bool equal to 'true' if @a aDSS 
     * has connected points at the back
     * @param isExtendableAtFront a bool equal to 'true' if @a aDSS 
     * has connected points at the front  
     */
    Quantity operator()( const typename DSSComputer::ConstIterator& /*it*/, 
                      const DSSComputer& aDSS, 
                      const bool& isExtendableAtBack,
                      const bool& isExtendableAtFront ) const {

      //types
      typedef typename DSSComputer::Integer Integer; 
      typedef typename DSSComputer::ConstIterator ConstIterator; 

      //functor
      Functor f; 

      //curvature value
      Quantity k = 0;  

      //begin and end iterators
      //(back point on the first point)
      //(front point on the last point)
      ConstIterator back = aDSS.begin();  
      ConstIterator front = aDSS.end();
      --front; 
      ASSERT( back != front); 

      if (isExtendableAtBack) {
        if (isExtendableAtFront) {

          --back;
          ++front; 

          //parameters
          Integer mu = aDSS.getMu();
          Integer omega = aDSS.getOmega();

          //cases
          if ( (aDSS.getRemainder(*back)<=mu-1)&&
               (aDSS.getRemainder(*front)<=mu-1) ) {                //convex
            k = f(aDSS) / myH; 
          } else if ( (aDSS.getRemainder(*back)>=mu+omega)&&
                (aDSS.getRemainder(*front)>=mu+omega) ) {           //concave
            k = -f(aDSS) / myH; 
          } //else                                                  //inflection

        } else {

          --back;

          //parameters
          Integer mu = aDSS.getMu();
          Integer omega = aDSS.getOmega();

          //cases
          if ( (aDSS.getRemainder(*back)<=mu-1) ) {                //convex
            k = f(aDSS) / myH; 
          } else if ( (aDSS.getRemainder(*back)>=mu+omega) ) {     //concave
            k = -f(aDSS) / myH; 
          } //else                                                 //inflection

        }
      } else if (isExtendableAtFront) {

          ++front; 

          //parameters
          Integer mu = aDSS.getMu();
          Integer omega = aDSS.getOmega();

          //cases
          if ( (aDSS.getRemainder(*front)<=mu-1) ) {                //convex
            k = f(aDSS) / myH; 
          } else if ( (aDSS.getRemainder(*front)>=mu+omega) ) {     //concave
            k = -f(aDSS) / myH; 
          } //else                                                  //inflection

      } //else cannot be extended: k is set to 0

      return k;
    };


    // ------------------------- Internal --------------------------------
  private:

    /** grid step */
    double myH; 

 

  }; // end of class CurvatureFromDSSBaseFunctor

  }//namespace detail



  //-------------------------------------------------------------------------------------------
  /**
   * Description of class 'CurvatureFromDSSLengthFunctor' <p> Aim: 
   * estimates the curvature from a DSS
   * recognized by a segment computer.
   *
   * The curvature @f$ k @f$ is defined from the discrete length @f$ l @f$ 
   * of a DSS as follow: 
   * @f$ 1/k = l*l/8 + 1/2 @f$
   *
   * @note Adaption from 
   *  Coeurjolly, D. and Miguet, S. and Tougne, L.
   *  "Discrete Curvature Based on Osculating Circle Estimation", 
   * Proc. IWVF, LNCS, vol 2059, pp.303-312, 2001
   *
   * In this approach, the DSS is viewed as the chord 
   * at a distance h (the grid step) to the osculating circle. 
   * Unfortunately, maximal DSS are in general too short.
   *
   * @tparam DSSComputer a model of segment computer
   * devoted the DSS recognition
   */
  template <typename DSSComputer>
  class CurvatureFromDSSLengthFunctor: 
    public detail::CurvatureFromDSSBaseFunctor<DSSComputer, detail::CurvatureFromDSSLength >
  {

    typedef detail::CurvatureFromDSSBaseFunctor<DSSComputer, detail::CurvatureFromDSSLength > Super; 
 
  public: 
    /**
     * Default Constructor.
     */
    CurvatureFromDSSLengthFunctor(): Super() {};
    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    CurvatureFromDSSLengthFunctor( const CurvatureFromDSSLengthFunctor & other ): Super(other) {};
  }; 

  //-------------------------------------------------------------------------------------------
  /**
   * Description of class 'CurvatureFromDSSLengthFunctor' <p> Aim: 
   * estimates the curvature from a DSS
   * recognized by a segment computer.
   *
   * The curvature @f$ k @f$ is defined from 
   * the length  @f$ l @f$ and the width  @f$ w @f$ of a DSS as follow: 
   *  @f$ 1/k = (l*l)/(8*w) + w/2 @f$
   *
   * @note Adaption from 
   *  Coeurjolly, D. and Miguet, S. and Tougne, L.
   *  "Discrete Curvature Based on Osculating Circle Estimation", 
   * Proc. IWVF, LNCS, vol 2059, pp.303-312, 2001
   *
   * @tparam DSSComputer a model of segment computer
   * devoted the DSS recognition
   */
  template <typename DSSComputer>
  class CurvatureFromDSSFunctor: 
    public detail::CurvatureFromDSSBaseFunctor<DSSComputer, detail::CurvatureFromDSSLengthAndWidth >
  {

    typedef detail::CurvatureFromDSSBaseFunctor<DSSComputer, detail::CurvatureFromDSSLengthAndWidth > Super; 
 
  public: 
    /**
     * Default Constructor.
     */
    CurvatureFromDSSFunctor(): Super() {};
    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    CurvatureFromDSSFunctor( const CurvatureFromDSSFunctor & other ): Super(other) {};
  }; 


} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SegmentComputerFunctor_h

#undef SegmentComputerFunctor_RECURSES
#endif // else defined(SegmentComputerFunctor_RECURSES)
