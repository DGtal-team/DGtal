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
  /////////////////////////////////////////////////////////////////////////////
  // class TangentFromDSSBaseFunctor
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of class 'TangentFromDSSBaseFunctor' <p> Aim: 
   * computes the tangent vector of a DSS. 
   *
   * @tparam DSSComputer a model of straight segment computer, 
   * having @a getA() and @a getB() methods. 
   *
   * The computation is delegated to a functor.  
   *
   * @tparam Functor a functor
   */

  template <typename DSSComputer, typename Functor, typename ReturnType = typename Functor::Quantity>
  class TangentFromDSSBaseFunctor
  {

  public: 

    // ----------------------- inner type ------------------------------
    typedef DSSComputer SegmentComputer;
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
     * @return the tangent estimated at @a it
     *
     * @param it the position at which the tangent is estimated.
     * @param aDSS a DSSComputer. 
     */
    Quantity operator()( const typename DSSComputer::ConstIterator& /*it*/, 
			 const DSSComputer& aDSS, 
			 const bool&,
			 const bool& ) const 
    {
      return this->operator()( aDSS ); 
    };

    /**
     * Operator()
     *
     * @return the tangent estimated from @a aDSS
     *
     * @param aDSS a DSSComputer. 
     */
    Quantity operator()(const DSSComputer& aDSS) const 
    {
      Functor f; 
      return f( aDSS ); 
    };

  }; // end of class TangentFromDSSFunctor


  }//namespace detail



  //-------------------------------------------------------------------------------------------
  template <typename DSSComputer>
  class TangentFromDSSFunctor: 
    public detail::TangentFromDSSBaseFunctor<DSSComputer, detail::NormalizedTangentVectorFromDSS>
  {
    typedef detail::TangentFromDSSBaseFunctor<DSSComputer, detail::NormalizedTangentVectorFromDSS> Super; 

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
    public detail::TangentFromDSSBaseFunctor<DSSComputer, detail::TangentVectorFromDSS<DSSComputer> >
  {
    typedef detail::TangentFromDSSBaseFunctor<DSSComputer, detail::TangentVectorFromDSS<DSSComputer> > Super; 

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
    public detail::TangentFromDSSBaseFunctor<DSSComputer, detail::TangentAngleFromDSS>
  {
    typedef detail::TangentFromDSSBaseFunctor<DSSComputer, detail::TangentAngleFromDSS> Super; 

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
