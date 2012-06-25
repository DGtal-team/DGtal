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
 * @brief Computes tangent, tangent angle, curvature from DSS.
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 *
 *
 * @date 2011/06/28
 *
 * Header file for module SegmentComputerFunctor.cpp
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
   * estimation from straight primitives). 
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
   * estimation from circular primitives). 
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
     * @param it position where the estimation is done.
     * @param aSC an instance of segment computer. 
     */
    Quantity operator()(const typename SegmentComputer::ConstIterator& it, 
			const SegmentComputer& aSC) const 
    {
      Functor f; 
      return f( it, aSC ); 
    };

  }; // end of class PosDepScaleIndepSCFunctor

  /**
   * Description of class 'TangentAngleFromDSS' <p> Aim: 
   * computes the tangent orientation of a DSS 
   * (angle in [-pi,+pi] radians between the tangent and the x-axis).
   */
    struct TangentAngleFromDSS
    {      
    public:
      typedef double Quantity; 

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

      Quantity operator() (const DSS& aDSS) const 
      {
	return Quantity(aDSS.getB(), aDSS.getA());
      }
    }; 
  /**
   * Description of class 'CurvatureFromDCA' <p> Aim: 
   * computes the curvature from a GeometricalDCA
   * at a given grid step
   */
    struct CurvatureFromDCA
    {      
    public:
      typedef double Quantity; 

      template<typename DCA>
      Quantity operator() (const DCA& aDCA, const Quantity& aH = 1.0) const 
      {
  	if ( aDCA.isStraight() )
  	  return 0.0; 
	else
	return ( aDCA.getSeparatingCircle().getCurvature() / aH );
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
	    double n = std::sqrt(v0*v0 + v1*v1); 
	    return Quantity( v0/n, v1/n );
  	  }
  	else
  	  {
	    double a, b, c; 
	    aDCA.getGeometricalDSSPtr()->getParameters(a, b, c); 
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

      template<typename DCA>
      Quantity operator() (const typename DCA::ConstIterator& it, 
  			   const DCA& aDCA) const 
      {
	NormalVectorFromDCA f; 
	Quantity normal = f(it, aDCA); 
	return Quantity( normal[1], normal[0] ); 
      }
    }; 

  }//namespace detail



  //-------------------------------------------------------------------------------------------
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
  template <typename DCAComputer>
  class CurvatureFromDCAFunctor: 
    public detail::PosIndepScaleDepSCFunctor<DCAComputer, 
						detail::CurvatureFromDCA>
  {
    typedef 
    detail::PosIndepScaleDepSCFunctor<DCAComputer, detail::CurvatureFromDCA> 
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
  template <typename DCAComputer>
  class NormalFromDCAFunctor: 
    public detail::PosDepScaleIndepSCFunctor<DCAComputer, 
						detail::NormalVectorFromDCA>
  {
    typedef 
    detail::PosDepScaleIndepSCFunctor<DCAComputer, detail::TangentVectorFromDCA> 
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
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
  namespace detail
  {
  /**
   * Description of class 'CurvatureFromDSSLength' <p> Aim: 
   * computes the curvature k from the length l of a DSS as follow: 
   * 1/k = l*l/8 + 1/2
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
   * computes the curvature k from 
   * the length l and the width w of a DSS as follow: 
   * 1/k = (l*l)/(8*w) + w/2
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
   * computes a curvature quantity from a DSS.
   *
   * @tparam DSSComputer a model of DSS
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
