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

/** 
 * @file SternBrocot.cpp
 * 
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 * @author Xavier Provençal (\c xavier.provencal@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/03/07
 *
 * Implementation of methods defined in SternBrocot.h 
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/arithmetic/SternBrocot.h"
///////////////////////////////////////////////////////////////////////////////

#include <iostream>


///////////////////////////////////////////////////////////////////////////////
// class Common
///////////////////////////////////////////////////////////////////////////////

/** DGtal Global variables
*
**/
namespace DGtal
{
  template <typename TInteger, typename TQuotient>
  DGtal::SternBrocot<TInteger, TQuotient>*
  DGtal::SternBrocot<TInteger, TQuotient>::singleton = 0;

  template <>
  SternBrocot<DGtal::int32_t,DGtal::int32_t>*
  SternBrocot<DGtal::int32_t,DGtal::int32_t>::singleton = 0;

  template <>
  SternBrocot<DGtal::int64_t,DGtal::int32_t>*
  SternBrocot<DGtal::int64_t,DGtal::int32_t>::singleton = 0;

  template <>
  SternBrocot<DGtal::int64_t,DGtal::int64_t>*
  SternBrocot<DGtal::int64_t,DGtal::int64_t>::singleton = 0;

#ifdef WITH_BIGINTEGER
  template <>
  SternBrocot<DGtal::BigInteger,DGtal::int32_t>*
  SternBrocot<DGtal::BigInteger,DGtal::int32_t>::singleton = 0;

  template <>
  SternBrocot<DGtal::BigInteger,DGtal::int64_t>*
  SternBrocot<DGtal::BigInteger,DGtal::int64_t>::singleton = 0;

  template <>
  SternBrocot<DGtal::BigInteger,DGtal::BigInteger>*
  SternBrocot<DGtal::BigInteger,DGtal::BigInteger>::singleton = 0;

#endif

//   template<>
//   DGtal::SternBrocot<int32_t,int32_t>::Quotient
//   DGtal::SternBrocot<int32_t,int32_t>::nbFractions = 0;
//   template<>
//   DGtal::SternBrocot<int64_t,int32_t>::Quotient
//   DGtal::SternBrocot<int64_t,int32_t>::nbFractions = 0;
//   template<>
//   DGtal::SternBrocot<int64_t,int64_t>::Quotient
//   DGtal::SternBrocot<int64_t,int64_t>::nbFractions = 0;

//   // Constant definitions in SternBrocot specializations.
//   // DGtal::SternBrocot<int32_t,int32_t>
//   template<>
//   DGtal::SternBrocot<int32_t,int32_t>::Node 
//   DGtal::SternBrocot<int32_t,int32_t>::myVirtualZeroOverOne
//   ( 0, 1, 0, -2, 
//     0, 0, 0, &myOneOverZero,
//     &myOneOverZero );

//   template<>
//   DGtal::SternBrocot<int32_t,int32_t>::Node 
//   DGtal::SternBrocot<int32_t,int32_t>::myOneOverZero
//   ( 1, 0, 0, -1, 
//     // &myVirtualZeroOverOne, 0, &myOneOverOne, 0,
//     &myZeroOverOne, 0, &myOneOverOne, 0,
//     &myZeroOverOne );

//   template<>
//   DGtal::SternBrocot<int32_t,int32_t>::Node 
//   DGtal::SternBrocot<int32_t,int32_t>::myZeroOverOne
//   ( 0, 1, 0, 0, 
//     // &myVirtualZeroOverOne, &myOneOverZero, 
//     &myZeroOverOne, &myOneOverZero, 
//     0, &myOneOverOne,
//     &myOneOverZero );

//   template<>
//   DGtal::SternBrocot<int32_t,int32_t>::Node 
//   DGtal::SternBrocot<int32_t,int32_t>::myOneOverOne
//   ( 1, 1, 1, 0, 
//     &myZeroOverOne, &myOneOverZero, 0, 0,
//     &myOneOverOne );

//   // Constant definitions in SternBrocot specializations.
//   // DGtal::SternBrocot<int64_t,int32_t>
//   template<>
//   DGtal::SternBrocot<int64_t,int32_t>::Node 
//   DGtal::SternBrocot<int64_t,int32_t>::myVirtualZeroOverOne
//   ( 0, 1, 0, -2, 
//     0, 0, 0, &myOneOverZero,
//     &myOneOverZero );

//   template<>
//   DGtal::SternBrocot<int64_t,int32_t>::Node 
//   DGtal::SternBrocot<int64_t,int32_t>::myOneOverZero
//   ( 1, 0, 0, -1, 
//     // &myVirtualZeroOverOne, 0, &myOneOverOne, 0,
//     &myZeroOverOne, 0, &myOneOverOne, 0,
//     &myZeroOverOne );

//   template<>
//   DGtal::SternBrocot<int64_t,int32_t>::Node 
//   DGtal::SternBrocot<int64_t,int32_t>::myZeroOverOne
//   ( 0, 1, 0, 0, 
//     // &myVirtualZeroOverOne, &myOneOverZero, 
//     &myZeroOverOne, &myOneOverZero, 
//     0, &myOneOverOne,
//     &myOneOverZero );

//   template<>
//   DGtal::SternBrocot<int64_t,int32_t>::Node 
//   DGtal::SternBrocot<int64_t,int32_t>::myOneOverOne
//   ( 1, 1, 1, 0, 
//     &myZeroOverOne, &myOneOverZero, 0, 0,
//     &myOneOverOne );

//   // Constant definitions in SternBrocot specializations.
//   // DGtal::SternBrocot<int64_t,int64_t>
//   template<>
//   DGtal::SternBrocot<int64_t,int64_t>::Node 
//   DGtal::SternBrocot<int64_t,int64_t>::myVirtualZeroOverOne
//   ( 0, 1, 0, -2, 
//     0, 0, 0, &myOneOverZero,
//     &myOneOverZero );

//   template<>
//   DGtal::SternBrocot<int64_t,int64_t>::Node 
//   DGtal::SternBrocot<int64_t,int64_t>::myOneOverZero
//   ( 1, 0, 0, -1, 
//     // &myVirtualZeroOverOne, 0, &myOneOverOne, 0,
//     &myZeroOverOne, 0, &myOneOverOne, 0,
//     &myZeroOverOne );

//   template<>
//   DGtal::SternBrocot<int64_t,int64_t>::Node 
//   DGtal::SternBrocot<int64_t,int64_t>::myZeroOverOne
//   ( 0, 1, 0, 0, 
//     // &myVirtualZeroOverOne, &myOneOverZero, 
//     &myZeroOverOne, &myOneOverZero, 
//     0, &myOneOverOne,
//     &myOneOverZero );

//   template<>
//   DGtal::SternBrocot<int64_t,int64_t>::Node 
//   DGtal::SternBrocot<int64_t,int64_t>::myOneOverOne
//   ( 1, 1, 1, 0, 
//     &myZeroOverOne, &myOneOverZero, 0, 0,
//     &myOneOverOne );

// #ifdef WITH_BIGINTEGER
//   template<>
//   DGtal::SternBrocot<BigInteger,int64_t>::Quotient
//   DGtal::SternBrocot<BigInteger,int64_t>::nbFractions = 0;
//   template<>
//   DGtal::SternBrocot<BigInteger,int32_t>::Quotient
//   DGtal::SternBrocot<BigInteger,int32_t>::nbFractions = 0;
//   template<>
//   DGtal::SternBrocot<BigInteger,BigInteger>::Quotient
//   DGtal::SternBrocot<BigInteger,BigInteger>::nbFractions = 0;

//   // Constant definitions in SternBrocot specializations.
//   // DGtal::SternBrocot<BigInteger,int64_t>
//   template<>
//   DGtal::SternBrocot<BigInteger,int64_t>::Node 
//   DGtal::SternBrocot<BigInteger,int64_t>::myVirtualZeroOverOne
//   ( 0, 1, 0, -2, 
//     0, 0, 0, &myOneOverZero,
//     &myOneOverZero );

//   template<>
//   DGtal::SternBrocot<BigInteger,int64_t>::Node 
//   DGtal::SternBrocot<BigInteger,int64_t>::myOneOverZero
//   ( 1, 0, 0, -1, 
//     // &myVirtualZeroOverOne, 0, &myOneOverOne, 0,
//     &myZeroOverOne, 0, &myOneOverOne, 0,
//     &myZeroOverOne );

//   template<>
//   DGtal::SternBrocot<BigInteger,int64_t>::Node 
//   DGtal::SternBrocot<BigInteger,int64_t>::myZeroOverOne
//   ( 0, 1, 0, 0, 
//     // &myVirtualZeroOverOne, &myOneOverZero, 
//     &myZeroOverOne, &myOneOverZero, 
//     0, &myOneOverOne,
//     &myOneOverZero );

//   template<>
//   DGtal::SternBrocot<BigInteger,int64_t>::Node 
//   DGtal::SternBrocot<BigInteger,int64_t>::myOneOverOne
//   ( 1, 1, 1, 0, 
//     &myZeroOverOne, &myOneOverZero, 0, 0,
//     &myOneOverOne );

//   // Constant definitions in SternBrocot specializations.
//   // DGtal::SternBrocot<BigInteger,int32_t>
//   template<>
//   DGtal::SternBrocot<BigInteger,int32_t>::Node 
//   DGtal::SternBrocot<BigInteger,int32_t>::myVirtualZeroOverOne
//   ( 0, 1, 0, -2, 
//     0, 0, 0, &myOneOverZero,
//     &myOneOverZero );

//   template<>
//   DGtal::SternBrocot<BigInteger,int32_t>::Node 
//   DGtal::SternBrocot<BigInteger,int32_t>::myOneOverZero
//   ( 1, 0, 0, -1, 
//     // &myVirtualZeroOverOne, 0, &myOneOverOne, 0,
//     &myZeroOverOne, 0, &myOneOverOne, 0,
//     &myZeroOverOne );

//   template<>
//   DGtal::SternBrocot<BigInteger,int32_t>::Node 
//   DGtal::SternBrocot<BigInteger,int32_t>::myZeroOverOne
//   ( 0, 1, 0, 0, 
//     // &myVirtualZeroOverOne, &myOneOverZero, 
//     &myZeroOverOne, &myOneOverZero, 
//     0, &myOneOverOne,
//     &myOneOverZero );

//   template<>
//   DGtal::SternBrocot<BigInteger,int32_t>::Node 
//   DGtal::SternBrocot<BigInteger,int32_t>::myOneOverOne
//   ( 1, 1, 1, 0, 
//     &myZeroOverOne, &myOneOverZero, 0, 0,
//     &myOneOverOne );

//   // Constant definitions in SternBrocot specializations.
//   // DGtal::SternBrocot<BigInteger,BigInteger>
//   template<>
//   DGtal::SternBrocot<BigInteger,BigInteger>::Node 
//   DGtal::SternBrocot<BigInteger,BigInteger>::myVirtualZeroOverOne
//   ( 0, 1, 0, -2, 
//     0, 0, 0, &myOneOverZero,
//     &myOneOverZero );

//   template<>
//   DGtal::SternBrocot<BigInteger,BigInteger>::Node 
//   DGtal::SternBrocot<BigInteger,BigInteger>::myOneOverZero
//   ( 1, 0, 0, -1, 
//     // &myVirtualZeroOverOne, 0, &myOneOverOne, 0,
//     &myZeroOverOne, 0, &myOneOverOne, 0,
//     &myZeroOverOne );

//   template<>
//   DGtal::SternBrocot<BigInteger,BigInteger>::Node 
//   DGtal::SternBrocot<BigInteger,BigInteger>::myZeroOverOne
//   ( 0, 1, 0, 0, 
//     // &myVirtualZeroOverOne, &myOneOverZero, 
//     &myZeroOverOne, &myOneOverZero, 
//     0, &myOneOverOne,
//     &myOneOverZero );

//   template<>
//   DGtal::SternBrocot<BigInteger,BigInteger>::Node 
//   DGtal::SternBrocot<BigInteger,BigInteger>::myOneOverOne
//   ( 1, 1, 1, 0, 
//     &myZeroOverOne, &myOneOverZero, 0, 0,
//     &myOneOverOne );



//   // const DGtal::BigInteger SternBrocot<DGtal::BigInteger>::ONE = 1;
//   // const DGtal::BigInteger SternBrocot<DGtal::BigInteger>::ZERO = 0;
// #endif 

}
