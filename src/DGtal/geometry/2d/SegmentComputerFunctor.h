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





//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class TangentFromDSSFunctor
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of class 'TangentFromDSSFunctor' <p> Aim: 
   * computes the normalized tangent vector of a DSS 
   *
   * @tparam DSSComputer a model for concept CSegmentComputer.
   */

  template <typename DSSComputer>
  class TangentFromDSSFunctor
  {

  public: 

    // ----------------------- inner type ------------------------------
    typedef DGtal::PointVector<2,double> RealVector;
    typedef RealVector Value;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~TangentFromDSSFunctor(){};

    /**
     * Default Constructor.
     */
    TangentFromDSSFunctor(){};

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    TangentFromDSSFunctor( const TangentFromDSSFunctor &  ) {};



    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Operator() 
     * @return the tangent at [aPoint]
     * @param aPoint the point at which the tangent is estimated.
     * @param aDSS a DSSComputer. 
     * @param isExtendableAtBack a bool equal to 'true' if [aDSS] can 
     * be extended at back and false otherwise. 
     * @param isExtendableAtFront a bool equal to 'true' if [aDSS] can 
     * be extended at front and false otherwise.  
     */
    Value operator()( const typename DSSComputer::Point& , 
                      const DSSComputer& aDSS, 
                      const double& = 1 ,
                      const bool& = false ,
                      const bool& = false) const {

      double x = NumberTraits<typename DSSComputer::Integer>
      ::castToDouble( aDSS.getB() ); 
      double y = NumberTraits<typename DSSComputer::Integer>
      ::castToDouble( aDSS.getA() );
      RealVector v(x,y); 
      double norm = v.norm(RealVector::L_2);
      v /= norm; 
      return v;
    };

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const
    {
      return true;
    };



    // ------------------------- Public Datas --------------------------------
  public:



    // ------------------------- Internal --------------------------------
  private:

 

  }; // end of class TangentFromDSSFunctor

  /////////////////////////////////////////////////////////////////////////////
  // class TangentAngleFromDSSFunctor
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of class 'TangentAngleFromDSSFunctor' <p> Aim: 
   * computes the tangent orientation (in radians) from the 
   * getA() and getB() methods returning the components of the main
   * direction vector of a segment 
   *
   * @tparam DSSComputer a model for concept CSegmentComputer.
   */

  template <typename DSSComputer>
  class TangentAngleFromDSSFunctor
  {

  public: 

    // ----------------------- inner type ------------------------------
  typedef double Value;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~TangentAngleFromDSSFunctor(){};

    /**
     * Default Constructor.
     */
    TangentAngleFromDSSFunctor(){};

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    TangentAngleFromDSSFunctor( const TangentAngleFromDSSFunctor & other ) {};





    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Operator() 
     * @return the tangent orientation at [aPoint]
     * (angle in [-pi,+pi] radians between the tangent and the x-axis).
     * @param aPoint the point at which the tangent is estimated.
     * @param aDSS a DSSComputer. 
     * @param isExtendableAtBack a bool equal to 'true' if [aDSS] can 
     * be extended at back and false otherwise. 
     * @param isExtendableAtFront a bool equal to 'true' if [aDSS] can 
     * be extended at front and false otherwise.  
     */
    Value operator()( const typename DSSComputer::Point& , 
                      const DSSComputer& aDSS, 
                      const double& = 1/*h = 1*/, 
                      const bool& = false/*isExtendableAtBack = false*/,
                      const bool& = false/*isExtendableAtFront = false*/) const {

      Value a = (Value) NumberTraits<typename DSSComputer::Integer>
                        ::castToInt64_t(aDSS.getA());      
      Value b = (Value) NumberTraits<typename DSSComputer::Integer>
                        ::castToInt64_t(aDSS.getB());      

      return std::atan2(a,b);
    };

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const
    {
      return true;
    };



    // ------------------------- Public Datas --------------------------------
  public:



    // ------------------------- Internal --------------------------------
  private:

 

  }; // end of class TangentAngleFromDSSFunctor


  /////////////////////////////////////////////////////////////////////////////
  // class CurvatureFromDSSLengthFunctor
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of class 'CurvatureFromDSSLengthFunctor' <p> Aim: 
   * computes the curvature k from the length l of a DSS as follow: 
   * 1/k = l*l/8 + 1/2
   *
   * @note Adaption from 
   *  Coeurjolly, D. and Miguet, S. and Tougne, L.
   *  "Discrete Curvature Based on Osculating Circle Estimation", 
   * Proc. IWVF, LNCS, vol 2059, pp.303-312, 2001
   *
   * @tparam DSSComputer a model for concept CSegmentComputer.
   */

  template <typename DSSComputer>
  class CurvatureFromDSSLengthFunctor
  {

  public: 

    // ----------------------- inner type ------------------------------
    typedef double Value;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~CurvatureFromDSSLengthFunctor(){};

    /**
     * Default Constructor.
     */
    CurvatureFromDSSLengthFunctor(){};

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    CurvatureFromDSSLengthFunctor( const CurvatureFromDSSLengthFunctor & /*other*/ ) {};



    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Operator() 
     * @return the curvature at [aPoint]
     * @param aPoint the point at which the curvature is estimated.
     * @param aDSS a DSSComputer. 
     * @param isExtendableAtBack a bool equal to 'true' if [aDSS] can 
     * be extended at back and false otherwise. 
     * @param isExtendableAtFront a bool equal to 'true' if [aDSS] can 
     * be extended at front and false otherwise.  
     */
    Value operator()( const typename DSSComputer::Point& /*aPoint*/, 
                      const DSSComputer& aDSS, 
                      const double& h = 1, 
                      const bool& isExtendableAtBack = false,
                      const bool& isExtendableAtFront = false) const {

      //types
      typedef typename DSSComputer::Integer Integer; 
      typedef typename DSSComputer::ConstIterator ConstIterator; 

      //curvature value
      Value k = 0;  

      //begin and end iterators
      //(back point on the first point)
      //(front point after the last point)
      ConstIterator front = aDSS.getFront();
      ConstIterator back = aDSS.getBack();  

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
            k = getValue( getLength(aDSS) )/h; 
          } else if ( (aDSS.getRemainder(*back)>=mu+omega)&&
                (aDSS.getRemainder(*front)>=mu+omega) ) {           //concave
            k = -getValue( getLength(aDSS) )/h; 
          } //else                                                  //inflection

        } else {

          --back;

          //parameters
          Integer mu = aDSS.getMu();
          Integer omega = aDSS.getOmega();

          //cases
          if ( (aDSS.getRemainder(*back)<=mu-1) ) {                //convex
            k = getValue( getLength(aDSS) )/h; 
          } else if ( (aDSS.getRemainder(*back)>=mu+omega) ) {     //concave
            k = -getValue( getLength(aDSS) )/h; 
          } //else                                                 //inflection

        }
      } else if (isExtendableAtFront) {

          ++front; 

          //parameters
          Integer mu = aDSS.getMu();
          Integer omega = aDSS.getOmega();

          //cases
          if ( (aDSS.getRemainder(*front)<=mu-1) ) {                //convex
            k = getValue( getLength(aDSS) )/h; 
          } else if ( (aDSS.getRemainder(*front)>=mu+omega) ) {     //concave
            k = -getValue( getLength(aDSS) )/h; 
          } //else                                                  //inflection

      } //else cannot be extended: k is set to 0

      return k;
    };

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const
    {
      return true;
    };



    // ------------------------- Public Datas --------------------------------
  public:



    // ------------------------- Internal --------------------------------
  private:

   /*
   * @param aDSS a DSSComputer. 
   * @return the length l of a DSS
   * defined as the length of the straight segment 
   * linking the two ends of the DSS
   */
   Value getLength(const DSSComputer& aDSS) const {
      typedef typename DSSComputer::Vector Vector; 
      Vector v(aDSS.getFrontPoint() - aDSS.getBackPoint()); 
      return v.norm(Vector::L_2); 
   }

   /*
   * @param the length l
   * @return the curvature k from the length l of a DSS as follow: 
   * 1/k = l*l/8 + 1/2
   */
   Value getValue(const Value& l = 1) const {
      return 1/( (l*l)/8 + 0.5 ); 
   }
 

  }; // end of class CurvatureFromDSSLengthFunctor

  /////////////////////////////////////////////////////////////////////////////
  // class CurvatureFromDSSFunctor
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of class 'CurvatureFromDSSFunctor' <p> Aim: 
   * computes the curvature k from 
   * the length l and the width w of a DSS as follow: 
   * 1/k = (l*l)/(8*w) + w/2
   *
   * @note Adaption from 
   *  Coeurjolly, D. and Miguet, S. and Tougne, L.
   *  "Discrete Curvature Based on Osculating Circle Estimation", 
   * Proc. IWVF, LNCS, vol 2059, pp.303-312, 2001
   *
   * @tparam DSSComputer a model for concept CSegmentComputer.
   */

  template <typename DSSComputer>
  class CurvatureFromDSSFunctor
  {

  public: 

    // ----------------------- inner type ------------------------------
    typedef double Value;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~CurvatureFromDSSFunctor(){};

    /**
     * Default Constructor.
     */
    CurvatureFromDSSFunctor(){};

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    CurvatureFromDSSFunctor( const CurvatureFromDSSFunctor & /*other*/ ) {};



    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Operator() 
     * @return the curvature at [aPoint]
     * @param aPoint the point at which the curvature is estimated.
     * @param aDSS a DSSComputer. 
     * @param isExtendableAtBack a bool equal to 'true' if [aDSS] can 
     * be extended at back and false otherwise. 
     * @param isExtendableAtFront a bool equal to 'true' if [aDSS] can 
     * be extended at front and false otherwise.  
     */
    Value operator()( const typename DSSComputer::Point& /*aPoint*/, 
                      const DSSComputer& aDSS, 
                      const double& h = 1, 
                      const bool& isExtendableAtBack = false,
                      const bool& isExtendableAtFront = false) const {

      //types
      typedef typename DSSComputer::Integer Integer; 
      typedef typename DSSComputer::ConstIterator ConstIterator; 

      //curvature value
      Value k = 0;  

      //begin and end iterators
      //(back point on the first point)
      //(front point after the last point)
      ConstIterator front = aDSS.getFront();
      ConstIterator back = aDSS.getBack();  

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
            k = getValue( getLength(aDSS), getWidth(aDSS) )/h; 
          } else if ( (aDSS.getRemainder(*back)>=mu+omega)&&
                (aDSS.getRemainder(*front)>=mu+omega) ) {           //concave
            k = -getValue( getLength(aDSS), getWidth(aDSS) )/h; 
          } //else                                                  //inflection

        } else {

          --back;

          //parameters
          Integer mu = aDSS.getMu();
          Integer omega = aDSS.getOmega();

          //cases
          if ( (aDSS.getRemainder(*back)<=mu-1) ) {                //convex
            k = getValue( getLength(aDSS), getWidth(aDSS) )/h; 
          } else if ( (aDSS.getRemainder(*back)>=mu+omega) ) {     //concave
            k = -getValue( getLength(aDSS), getWidth(aDSS) )/h; 
          } //else                                                 //inflection

        }
      } else if (isExtendableAtFront) {

          ++front; 

          //parameters
          Integer mu = aDSS.getMu();
          Integer omega = aDSS.getOmega();

          //cases
          if ( (aDSS.getRemainder(*front)<=mu-1) ) {                //convex
            k = getValue( getLength(aDSS), getWidth(aDSS) )/h; 
          } else if ( (aDSS.getRemainder(*front)>=mu+omega) ) {     //concave
            k = -getValue( getLength(aDSS), getWidth(aDSS) )/h; 
          } //else                                                  //inflection

      } //else cannot be extended: k is set to 0

      return k;
    };

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const
    {
      return true;
    };



    // ------------------------- Public Datas --------------------------------
  public:



    // ------------------------- Internal --------------------------------
  private:

   /*
   * @param aDSS a DSSComputer. 
   * @return the length l of a DSS
   * defined as the length of the straight segment 
   * linking the two ends of the DSS
   */
   Value getLength(const DSSComputer& aDSS) const {
      typedef typename DSSComputer::Vector Vector; 
      Vector v(aDSS.getFrontPoint() - aDSS.getBackPoint()); 
      return v.norm(Vector::L_2); 
   }

   /*
   * @param aDSS a DSSComputer. 
   * @return the width w of a DSS
   * defined as 1/sqrt(a*a + b*b)
   */
   Value getWidth(const DSSComputer& aDSS) const {
      typedef typename DSSComputer::Vector Vector; 
      Vector v( aDSS.getB(), aDSS.getA() ); 
      return 1/v.norm(Vector::L_2); 
   }


   /*
   * @param the length l
   * @param the width w
   * @return the curvature k from 
   * the length l and the width w of a DSS as follow: 
   * 1/k = (l*l)/(8*w) + w/2
   */
   Value getValue(const Value& l = 1, const Value& w = 1) const {
      return 1/( (l*l)/(8*w) + w/2 ); 
   }
 

  }; // end of class CurvatureFromDSSLengthFunctor

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SegmentComputerFunctor_h

#undef SegmentComputerFunctor_RECURSES
#endif // else defined(SegmentComputerFunctor_RECURSES)
