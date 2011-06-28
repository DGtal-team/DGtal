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
  // class TangentFromArithmeticalDSSFunctor
  /////////////////////////////////////////////////////////////////////////////
  /**
   * Description of class 'TangentFromArithmeticalDSSFunctor' <p> Aim: describes a 4-connected
   * oriented interpixel curve, closed or open. For instance, the
   * topological boundary of a  simply connected digital set is a
   * closed grid curve. This object provides several ranges, such as
   * PointsRange used to get the (integer) coordinates of the grid
   * points (or pointels) of the grid curve. 
   *
   * Example :
   * @code 

   * @endcode
   */

  template <typename ArithmeticalDSS>
  class TangentFromArithmeticalDSSFunctor
  {

  public: 

    // ----------------------- inner types ------------------------------
  typedef double Value;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~TangentFromArithmeticalDSSFunctor(){};


    /**
     * Default Constructor.
     */
    TangentFromArithmeticalDSSFunctor(){};


    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    TangentFromArithmeticalDSSFunctor( const TangentFromArithmeticalDSSFunctor & other ) {};

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
//    TangentFromArithmeticalDSSFunctor & operator=( const TangentFromArithmeticalDSSFunctor & other ) { 
//     return *this;
//    };




    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Operator() 
     * @return the tangent orientation (in radians).
     * @param aDSS an arithmeticalDSS. 
     */
    Value operator()( const ArithmeticalDSS& aDSS ) const {

      Value a = (Value) IntegerTraits<typename ArithmeticalDSS::Integer>
                        ::castToInt64_t(aDSS.getA());      
      Value b = (Value) IntegerTraits<typename ArithmeticalDSS::Integer>
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

 



  }; // end of class TangentFromArithmeticalDSSFunctor





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
