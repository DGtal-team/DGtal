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
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 *
 * @date 2011/06/28
 *
 * Header file for module SegmentComputerFunctor.cpp
 *
 * This file is part of the DGtal library.
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

#include "DGtal/base/BasicTypes.h"
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
   * computes the tangent orientation (in radians) from the 
   * getA() and getB() returning the components of the main
   * direction vector of a segment 
   *
   * Example :
   * @code 

   * @endcode
   */

  template <typename DSSComputer, typename TRealVector>
  class TangentFromDSSFunctor
  {

  public: 

    // ----------------------- inner type ------------------------------
    typedef TRealVector Value;
    typedef TRealVector RealVector;

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
    TangentFromDSSFunctor( const TangentFromDSSFunctor & other ) {};

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
//    TangentFromDSSFunctor & operator=( const TangentFromDSSFunctor & other ) { 
//     return *this;
//    };




    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Operator() 
     * @return the tangent at [aPoint]
     * @param aDSS a DSSComputer. 
     * @param aPoint the point at which the tangent is estimated. 
     */
    Value operator()( const typename DSSComputer::Point& aPoint, 
                      const DSSComputer& aDSS ) const {

      double x = IntegerTraits<typename DSSComputer::Integer>
      ::castToDouble( aDSS.getB() ); 
      double y = IntegerTraits<typename DSSComputer::Integer>
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
   * getA() and getB() returning the components of the main
   * direction vector of a segment 
   *
   * Example :
   * @code 

   * @endcode
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

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
//    TangentAngleFromDSSFunctor & operator=( const TangentAngleFromDSSFunctor & other ) { 
//     return *this;
//    };




    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Operator() 
     * @return the tangent orientation at [aPoint]
     * (angle in [-pi,+pi] radians between the tangent and the x-axis).
     * @param aDSS a DSSComputer. 
     * @param aPoint the point at which the tangent orientation is estimated. 
     */
    Value operator()( const typename DSSComputer::Point& aPoint, 
                      const DSSComputer& aDSS ) const {

      Value a = (Value) IntegerTraits<typename DSSComputer::Integer>
                        ::castToInt64_t(aDSS.getA());      
      Value b = (Value) IntegerTraits<typename DSSComputer::Integer>
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





} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
//#if defined(INLINE)
//#include "DGtal/geometry/2d/SegmentComputerFunctor.ih"
//#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SegmentComputerFunctor_h

#undef SegmentComputerFunctor_RECURSES
#endif // else defined(SegmentComputerFunctor_RECURSES)
